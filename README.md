# IoTDevice

IoTDevice to send GPS location and button status via Lora

https://github.com/manuelbl/ttn-esp32/wiki/Get-Started has decent docs for used lora library use. This is used mostly same way ;)

In plaformio do something else ;)

In shell setup
1. Install esp-idf ( https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-macos-setup.html )
2. git clone --recursive git@github.com:VekotinVerstas/IoTDevice.git
4. . ./export.sh at esp-idf dir to export esp-idf commands ( do not run in default shell init scrips as it messes up a lot [in python virtual])
5. at main/ copy identity_example.h to identity.h and edit for your Lora keys 
6. at project root iun "idf.py menuconfig" to initialize config
7. at project root run "idf.py build flash monitor" ( or supset of options to build, flash and monitor the device code )

This is at early dev state and likely to change all over

UBlox config tool and https://github.com/VekotinVerstas/serialforward can be used to access Neo 6m GPS module in windows.
