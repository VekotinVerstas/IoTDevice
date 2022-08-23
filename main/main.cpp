/*******************************************************************************
 *
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 *******************************************************************************/
// NOTES:
// Älä koskaan lataa akkua TTGO Lora v1.6 laudalla! ( akku syttyy tuleen ). T-Beam lataa akkua ok.  

// The LoRaWAN frequency and the radio chip must be configured by running 'idf.py menuconfig'.
// Go to Components / The Things Network, select the appropriate values and save.

#include <driver/gpio.h>
#include <esp_event.h>
#include <esp_sleep.h>
#include <freertos/FreeRTOS.h>
#include <nvs_flash.h>
#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <driver/rtc_io.h>
#include <driver/uart.h>
#include <driver/gpio.h>

#include <TheThingsNetwork.h>

//#include <nmea_parser.h>

//#include <freertos/task.h>

#include <nmea.h>
#include <gpgll.h>
#include <gpgga.h>
#include <gprmc.h>
#include <gpgsa.h>
#include <gpvtg.h>
#include <gptxt.h>
#include <gpgsv.h>

#include <freertos/task.h>
#include <driver/rmt.h>
#include <led_strip.h>
#include "sdkconfig.h"
#include "identity.h"

void uart_setup();
time_t read_and_parse_nmea(void*);

static const char *TAG = "IoTdeviceLed";

#define RMT_TX_CHANNEL RMT_CHANNEL_2

// Pins and other resources
#define TTN_SPI_HOST      HSPI_HOST
#define TTN_SPI_DMA_CHAN  1
#define TTN_PIN_SPI_SCLK  5
#define TTN_PIN_SPI_MOSI  27
#define TTN_PIN_SPI_MISO  19
#define TTN_PIN_NSS       18
#define TTN_PIN_RXTX      TTN_NOT_CONNECTED
#define TTN_PIN_RST       TTN_NOT_CONNECTED
#define TTN_PIN_DIO0      26
#define TTN_PIN_DIO1      33

#define GPS_UART_NUM            UART_NUM_1
#define GPS_UART_RX_PIN         34
#define GPS_UART_TX_PIN         12
#define GPS_UART_RX_BUF_SIZE    (1024)

// LedStrip pin 25 set in menuconfig

static TheThingsNetwork ttn;
const unsigned TX_INTERVAL = 60;
const uint8_t loraMsgLen = 14;
const uint8_t loraMaxRate = 4; // 4 msg / minute
// This counter value is retained (in RTC memory) during deep sleep
RTC_DATA_ATTR int counter_in_rtc_mem;
RTC_DATA_ATTR time_t send_time_rtc;
RTC_DATA_ATTR uint8_t send_count_rtc;

// RTC functionality can be used in pins: 0,2,4,12-15,25-27,32-39.

//External buttons
const gpio_num_t ext_wakeup_pin_1 = GPIO_NUM_15;
const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;

const gpio_num_t ext_wakeup_pin_2 = GPIO_NUM_35;
const uint64_t ext_wakeup_pin_2_mask = 1ULL << ext_wakeup_pin_2;

const gpio_num_t ext_wakeup_pin_3 = GPIO_NUM_14;
const uint64_t ext_wakeup_pin_3_mask = 1ULL << ext_wakeup_pin_3;

const gpio_num_t ext_wakeup_pin_4 = GPIO_NUM_13;
const uint64_t ext_wakeup_pin_4_mask = 1ULL << ext_wakeup_pin_4;

const gpio_num_t ext_wakeup_pin_5 = GPIO_NUM_2;
const uint64_t ext_wakeup_pin_5_mask = 1ULL << ext_wakeup_pin_5;

const gpio_num_t ext_wakeup_pin_6 = GPIO_NUM_4;
const uint64_t ext_wakeup_pin_6_mask = 1ULL << ext_wakeup_pin_6;

//Internal button. Not in use at normal build
const gpio_num_t ext_wakeup_pin_x = GPIO_NUM_38;
const uint64_t ext_wakeup_pin_x_mask = 1ULL << ext_wakeup_pin_x;

//Led pins
const gpio_num_t ext_led_pin_1 = GPIO_NUM_32;
const gpio_num_t ext_led_pin_2 = GPIO_NUM_33;

uint8_t buttons = 0b00000000;

//Lora return message
void messageReceived(const uint8_t *message, size_t length, ttn_port_t port)  //Lora messages
{
    printf("Message of %d bytes received on port %d:", length, port);
    for (int i = 0; i < length; i++)
        printf(" %02x", message[i]);
    printf("\n");
}

void blink_task(void *pvParameter)
{
 uint8_t color = 10;
 gpio_pad_select_gpio(ext_led_pin_1);
 gpio_set_direction (ext_led_pin_1,GPIO_MODE_OUTPUT);
 gpio_pad_select_gpio(ext_led_pin_2);
 gpio_set_direction (ext_led_pin_2,GPIO_MODE_OUTPUT);
 
 ESP_LOGI(TAG, "LED start");
 
 while(1)
  {
    gpio_set_level(ext_led_pin_1,0);
    gpio_set_level(ext_led_pin_2,1);
    vTaskDelay(1000/portTICK_RATE_MS);
    gpio_set_level(ext_led_pin_1,1);
    gpio_set_level(ext_led_pin_2,0);
    vTaskDelay(1000/portTICK_RATE_MS);
    if(color>=30) color=10;
    else color++;
    //ESP_LOGI(TAG, "LED done");
  }
}

extern "C" void app_main(void)
{
    void* loraMsg;
    esp_err_t err;
    time_t gpsTime=0;

    //Prosess for conrolling state of the eternal led's
    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    //Led strip chan 2, pin 25, 8 leds. Config at menuconfig
    led_strip_t* strip = led_strip_init(RMT_TX_CHANNEL, CONFIG_EXAMPLE_RMT_TX_GPIO, CONFIG_EXAMPLE_STRIP_LED_NUMBER);
    if (!strip)
     {
     ESP_LOGE(TAG, "install WS2812 driver failed");
     }
    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));

    // Set some leds ( just to show that it works, needs plan what to do with leds
    strip->set_pixel(strip, 0, 10, 0, 0);
    //strip->set_pixel(strip, 1, 0, 30, 0);
    //strip->set_pixel(strip, 2, 0, 0, 30);
    //strip->set_pixel(strip, 3, 30, 0, 0);
    //strip->set_pixel(strip, 4, 0, 30, 0);
    //strip->set_pixel(strip, 5, 0, 0, 30);
    // Flush RGB values to LEDs
    ESP_ERROR_CHECK(strip->refresh(strip, 100));

    // Initialize the GPIO ISR handler service
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    ESP_ERROR_CHECK(err);

    // Initialize the NVS (non-volatile storage) for saving and restoring the keys
    err = nvs_flash_init();
    ESP_ERROR_CHECK(err);

    // Initialize SPI bus
    spi_bus_config_t spi_bus_config;
    spi_bus_config.miso_io_num = TTN_PIN_SPI_MISO;
    spi_bus_config.mosi_io_num = TTN_PIN_SPI_MOSI;
    spi_bus_config.sclk_io_num = TTN_PIN_SPI_SCLK;
    spi_bus_config.quadwp_io_num = -1;
    spi_bus_config.quadhd_io_num = -1;
    spi_bus_config.max_transfer_sz = 0;
    err = spi_bus_initialize(TTN_SPI_HOST, &spi_bus_config, TTN_SPI_DMA_CHAN);
    ESP_ERROR_CHECK(err);

    // Configure the SX127x pins
    ttn.configurePins(TTN_SPI_HOST, TTN_PIN_NSS, TTN_PIN_RXTX, TTN_PIN_RST, TTN_PIN_DIO0, TTN_PIN_DIO1);

    // The below line can be commented after the first run as the data is saved in NVS
    ttn.provision(devEui, appEui, appKey);

    // Register callback for received messages
    ttn.onMessage(messageReceived);

    //    ttn.setMaxTxPower(14);
    //    vTaskDelay(10000 / portTICK_PERIOD_MS);
    
    if (ttn.resumeAfterDeepSleep())
    {
        printf("Resumed from deep sleep.\n");
	switch (esp_sleep_get_wakeup_cause())
	{
           case ESP_SLEEP_WAKEUP_EXT1:
	   {
              uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
              if (wakeup_pin_mask != 0)
	      {
                  int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
                  printf("Wake up from GPIO %d\n", pin);
		  switch (pin)
		   {
		   case ext_wakeup_pin_1:
		     buttons |= (1 << 0);;
		     break;
		   case ext_wakeup_pin_2:
		     buttons |= (1 << 1);;
		     break;
		   case ext_wakeup_pin_3:
		     buttons |= (1 << 2);;
		     break;
		   case ext_wakeup_pin_4:
		     buttons |= (1 << 3);;
		     break;
		   case ext_wakeup_pin_5:
		     buttons |= (1 << 4);;
		     break;
		   case ext_wakeup_pin_6:
		     buttons |= (1 << 5);;
		     break;
		     //default:
		     // default statements
		   }
		  
              }
	     else
	     {
                 printf("Wake up from GPIO (all high gpio38 mode)\n");
		 buttons |= (1 << 7);;

             }
          break;
          }

	default:
	  break;
       }
    }
    else
    {
        printf("Joining...\n");
        if (ttn.join())
        {
            printf("Joined.\n");
        }
        else
        {
            printf("Join failed. Goodbye\n");
            return;
        }
    }

    /* register event handler for NMEA parser library */
    //nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL);
    loraMsg = malloc(loraMsgLen);
    gpsTime=read_and_parse_nmea(loraMsg);
    /*
	loraMsg[0]=(uint8_t)gps->date.year;
	loraMsg[1]=gps->date.month;
	loraMsg[2]=gps->date.day;
	loraMsg[3]=gps->tim.hour;
	loraMsg[4]=gps->tim.minute;
	loraMsg[5]=gps->tim.second;
	memcpy( (void *)&loraMsg[6], &gps->latitude, 4 );
	memcpy( (void *)&loraMsg[10], &gps->longitude, 4 );
	memcpy( (void *)&loraMsg[14], &gps->altitude, 4 );
	memcpy( (void *)&loraMsg[18], &gps->speed, 4 );
    */
    
    *(((uint8_t*)loraMsg)+12) = 80;
    *(((uint8_t*)loraMsg)+13) = buttons;

    printf("Sending message...\n");
    counter_in_rtc_mem++;
    if( (gpsTime - send_time_rtc) < 60 ) // if less than minute from periods first send
     {
     send_count_rtc++;
     if( send_count_rtc > loraMaxRate ) // Too many send requests 
      {
      // Wait until end of sending period
      printf("Send delay...\n");
      vTaskDelay(((gpsTime - send_time_rtc)*1000) / portTICK_RATE_MS);
      printf("Send delay off...\n");
      // Start new period 
      send_count_rtc = 1;
      send_time_rtc = gpsTime;
      }
     }
    else  // First send in period
     {
     // Start new period 
     send_count_rtc = 1;
     send_time_rtc = gpsTime;
     }
    //sprintf(message, "IotDev %d", counter_in_rtc_mem);
    TTNResponseCode res = ttn.transmitMessage((uint8_t*)loraMsg, loraMsgLen);
    printf("Payload: ");
    for( int cp=0; cp<loraMsgLen; cp++ )
     {
     printf("%02x", *(((uint8_t*)loraMsg)+cp));
     }
    printf("\n");
    printf(res == kTTNSuccessfulTransmission ? "%d bytes sent.\n" : "Transmission failed.\n", loraMsgLen);
    buttons=0;
    free(loraMsg);
    
    // Wait until TTN communication is idle and save state
    ttn.waitForIdle();
    ttn.prepareForDeepSleep();

    //Presleep tasks
    //Before sleep, applications must disable WiFi and BT ( esp_bluedroid_disable, esp_bt_controller_disable, esp_wifi_stop).

    //internal pulldown resistors need to be manaualy enabled before sleep in rtc level
    rtc_gpio_pulldown_en(ext_wakeup_pin_1);
    rtc_gpio_pullup_dis(ext_wakeup_pin_1);
    rtc_gpio_pulldown_en(ext_wakeup_pin_2);
    rtc_gpio_pullup_dis(ext_wakeup_pin_2);
    rtc_gpio_pulldown_en(ext_wakeup_pin_3);
    rtc_gpio_pullup_dis(ext_wakeup_pin_3);
    rtc_gpio_pulldown_en(ext_wakeup_pin_4);
    rtc_gpio_pullup_dis(ext_wakeup_pin_4);
    rtc_gpio_pulldown_en(ext_wakeup_pin_5);
    rtc_gpio_pullup_dis(ext_wakeup_pin_5);
    rtc_gpio_pulldown_en(ext_wakeup_pin_6);
    rtc_gpio_pullup_dis(ext_wakeup_pin_6);
    
    // RTC domain needs to be on during sleep GPIO int to work. This keeps ULP side cpu running thus externel pulldown might be the thing.
    ESP_ERROR_CHECK( esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON) );


    printf("Enabling EXT1 wakeup on pins GPIO%d, GPIO%d, GPIO%d, GPIO%d, GPIO%d, GPIO%d\n", ext_wakeup_pin_1, ext_wakeup_pin_2, ext_wakeup_pin_3, ext_wakeup_pin_4, ext_wakeup_pin_5, ext_wakeup_pin_6);
    /*RTC functionality can be used in pins: 0,2,4,12-15,25-27,32-39.
    ESP_EXT1_WAKEUP_ALL_LOW: wake up when all selected GPIOs are low
    ESP_EXT1_WAKEUP_ANY_HIGH: wake up when any of the selected GPIOs is high*/
    ESP_ERROR_CHECK( esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask | ext_wakeup_pin_2_mask | ext_wakeup_pin_3_mask | ext_wakeup_pin_4_mask | ext_wakeup_pin_5_mask | ext_wakeup_pin_6_mask, ESP_EXT1_WAKEUP_ANY_HIGH) ); // Normal production buttons ( internal button can not be here )

    //Internal button
    //ESP_ERROR_CHECK( esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_x_mask, ESP_EXT1_WAKEUP_ALL_LOW) ); //For demo use with internal button ( it has wrong polarity for normal ESP_EXT1_WAKEUP_ANY_HIGH ).
    
    // Schedule wake up
    //esp_sleep_enable_timer_wakeup(TX_INTERVAL * 1000000LL);

    //Go to sleep
    printf("Going to deep sleep...\n");
    esp_deep_sleep_start();
    //vTaskDelay(60000 / portTICK_RATE_MS);
}

void uart_setup()
{
    uart_config_t uart_config = {
            .baud_rate = 9600,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	    .rx_flow_ctrl_thresh = 122
    };
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM,
                    GPS_UART_TX_PIN, GPS_UART_RX_PIN,
                    UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, GPS_UART_RX_BUF_SIZE * 2, 0, 0, NULL, 0));
}

// Open UART to GPS module port and read and parse data
time_t read_and_parse_nmea(void* nmeaData)
{
    // Configure a temporary buffer for the incoming data
    char *buffer = (char*) malloc(GPS_UART_RX_BUF_SIZE + 1);
    char fmt_buf[32];
    size_t total_bytes = 0;
    int gotFix=0;
    time_t aika=0;

    uart_setup();
    
    while (gotFix==0) {
        // Read data from the UART
        int read_bytes = uart_read_bytes(GPS_UART_NUM,
                (uint8_t*) buffer + total_bytes,
                GPS_UART_RX_BUF_SIZE - total_bytes, 100 / portTICK_RATE_MS);
        if (read_bytes <= 0) {
            continue;
        }

        nmea_s *data;
        total_bytes += read_bytes;

        // find start (a dollar sign)
        char* start = (char*)memchr(buffer, '$', total_bytes);
        if (start == NULL) {
            total_bytes = 0;
            continue;
        }

        // find end of line
        char* end = (char*)memchr(start, '\r', total_bytes - (start - buffer));
        if (NULL == end || '\n' != *(++end)) {
            continue;
        }
        end[-1] = NMEA_END_CHAR_1;
        end[0] = NMEA_END_CHAR_2;

        // handle data
        data = nmea_parse(start, end - start + 1, 0);
        if (data == NULL) {
            printf("Failed to parse the sentence!\n");
            printf("  Type: %.5s (%d)\n", start+1, nmea_get_type(start));
        } else {
            if (data->errors != 0) {
                printf("WARN: The sentence struct contains parse errors!\n");
            }

            if (NMEA_GPGGA == data->type) {
                nmea_gpgga_s *gpgga = (nmea_gpgga_s *) data;
                printf("GPGGA Number of satellites: %d\n", gpgga->n_satellites);
                printf("GPGGA Altitude: %f %c\n", gpgga->altitude, gpgga->altitude_unit);
            }
	    /*
            if (NMEA_GPGLL == data->type) {
                printf("GPGLL sentence\n");
                nmea_gpgll_s *pos = (nmea_gpgll_s *) data;
                printf("Longitude:\n");
                printf("  Degrees: %d\n", pos->longitude.degrees);
                printf("  Minutes: %f\n", pos->longitude.minutes);
                printf("  Cardinal: %c\n", (char) pos->longitude.cardinal);
                printf("Latitude:\n");
                printf("  Degrees: %d\n", pos->latitude.degrees);
                printf("  Minutes: %f\n", pos->latitude.minutes);
                printf("  Cardinal: %c\n", (char) pos->latitude.cardinal);
                strftime(fmt_buf, sizeof(fmt_buf), "%H:%M:%S", &pos->time);
                printf("Time: %s\n", fmt_buf);
            }
	    */
            if (NMEA_GPRMC == data->type) {	  
                printf("GPRMC sentence\n");
                nmea_gprmc_s *posRMC = (nmea_gprmc_s *) data;
		if( posRMC->valid )
		 {
		 gotFix=1;
		 printf("Longitude:\n");
		 printf("  Degrees: %d\n", posRMC->longitude.degrees); //int 4 bytes
		 printf("  Minutes: %f\n", posRMC->longitude.minutes); //double 8 bytes
		 printf("  LongGoogle: %d\n", (int)((posRMC->longitude.degrees+(posRMC->longitude.minutes/60))*100000)); //double 8 bytes
		 printf("  LongLora: %d\n", (int)((posRMC->longitude.degrees+(posRMC->longitude.minutes/60))*100000)*100/256); //double 8 bytes

		 printf("  Cardinal: %c\n", (char) posRMC->longitude.cardinal); //N,E,S,W,\0
		 printf("Latitude:\n");
		 printf("  Degrees: %d\n", posRMC->latitude.degrees);
		 printf("  Minutes: %f\n", posRMC->latitude.minutes);
		 printf("  LatGoogle: %d\n", (int)((posRMC->latitude.degrees+(posRMC->latitude.minutes/60))*100000));
		 printf("  LatLora: %d\n", (int)((posRMC->latitude.degrees+(posRMC->latitude.minutes/60))*100000)*100/256);
		 *(char*)nmeaData=(char)10;
		 *(int*)(nmeaData+1) = (int)((posRMC->latitude.degrees+(posRMC->latitude.minutes/60))*100000)*100/256;
		 *(int*)(nmeaData+4) = (int)((posRMC->longitude.degrees+(posRMC->longitude.minutes/60))*100000)*100/256;
		 printf("  Cardinal: %c\n", (char) posRMC->latitude.cardinal);
		 //strftime(fmt_buf, sizeof(fmt_buf), "%d %b %T %Y", &posRMC->date_time);
		 *(char*)(nmeaData+7)=(char)20;
		 aika = mktime(&posRMC->date_time);
		 printf("Date & Time: %lu\n", aika);
		 *(int*)(nmeaData+8) = (int)aika;

		 printf("Speed, in Knots: %f\n", posRMC->gndspd_knots);
		 printf("Track, in degrees: %f\n", posRMC->track_deg);
		 printf("Magnetic Variation:\n");
		 printf("  Degrees: %f\n", posRMC->magvar_deg);
		 printf("  Cardinal: %c\n", (char) posRMC->magvar_cardinal);
		 double adjusted_course = posRMC->track_deg;
		 }
	    }
	    else
	     {
	     printf("GPRMC no valid fix!\n");
	     }
	    /*
            if (NMEA_GPGSA == data->type) {
                nmea_gpgsa_s *gpgsa = (nmea_gpgsa_s *) data;
		if((gotFix==0) && (gpgsa->fixtype > 2) ) gotFix=1;
                printf("GPGSA Sentence:\n");
                printf("  Mode: %c\n", gpgsa->mode);
                printf("  Fix:  %d\n", gpgsa->fixtype);
                printf("  PDOP: %.2lf\n", gpgsa->pdop);
                printf("  HDOP: %.2lf\n", gpgsa->hdop);
                printf("  VDOP: %.2lf\n", gpgsa->vdop);
            }
	    
            if (NMEA_GPGSV == data->type) {
                nmea_gpgsv_s *gpgsv = (nmea_gpgsv_s *) data;

                printf("GPGSV Sentence:\n");
                printf("  Num: %d\n", gpgsv->sentences);
                printf("  ID:  %d\n", gpgsv->sentence_number);
                printf("  SV:  %d\n", gpgsv->satellites);
                printf("  #1:  %d %d %d %d\n", gpgsv->sat[0].prn, gpgsv->sat[0].elevation, gpgsv->sat[0].azimuth, gpgsv->sat[0].snr);
                printf("  #2:  %d %d %d %d\n", gpgsv->sat[1].prn, gpgsv->sat[1].elevation, gpgsv->sat[1].azimuth, gpgsv->sat[1].snr);
                printf("  #3:  %d %d %d %d\n", gpgsv->sat[2].prn, gpgsv->sat[2].elevation, gpgsv->sat[2].azimuth, gpgsv->sat[2].snr);
                printf("  #4:  %d %d %d %d\n", gpgsv->sat[3].prn, gpgsv->sat[3].elevation, gpgsv->sat[3].azimuth, gpgsv->sat[3].snr);
            }

            if (NMEA_GPTXT == data->type) {
                nmea_gptxt_s *gptxt = (nmea_gptxt_s *) data;

                printf("GPTXT Sentence:\n");
                printf("  ID: %d %d %d\n", gptxt->id_00, gptxt->id_01, gptxt->id_02);
                printf("  %s\n", gptxt->text);
            }
	    
            if (NMEA_GPVTG == data->type) {
                nmea_gpvtg_s *gpvtg = (nmea_gpvtg_s *) data;

                printf("GPVTG Sentence:\n");
                printf("  Track [deg]:   %.2lf\n", gpvtg->track_deg);
                printf("  Speed [kmph]:  %.2lf\n", gpvtg->gndspd_kmph);
                printf("  Speed [knots]: %.2lf\n", gpvtg->gndspd_knots);
            }
	    */
            nmea_free(data);
        }

        // buffer empty?
        if (end == buffer + total_bytes) {
            total_bytes = 0;
            continue;
        }

        // copy rest of buffer to beginning
        if (buffer != memmove(buffer, end, total_bytes - (end - buffer))) {
            total_bytes = 0;
            continue;
        }

        total_bytes -= end - buffer;
    }
    uart_driver_delete(GPS_UART_NUM);
    free(buffer);
    return(aika);
}
