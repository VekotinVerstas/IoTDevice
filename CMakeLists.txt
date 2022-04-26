cmake_minimum_required(VERSION 3.5)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

#set(EXTRA_COMPONENT_DIRS )

# Update the below line to match the path to the ttn-esp32 library,
# e.g. list(APPEND EXTRA_COMPONENT_DIRS "/Users/me/Documents/ttn-esp32")
list(APPEND EXTRA_COMPONENT_DIRS "../.." "$ENV{IDF_PATH}/examples/common_components/led_strip")

#add_definitions(-DLMIC_ENABLE_event_logging=1)

project(IoTDevice)
