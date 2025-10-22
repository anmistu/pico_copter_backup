cmake_minimum_required(VERSION 3.13)
include_directories(/usr/include/eigen3)


include(pico_sdk_import.cmake)

project(pico_copter C CXX ASM)
set(CMAKE_C_STNDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

pico_sdk_init()
include_directories(/Users/itoukouhei/pico/eigen)

add_executable(pico_copter
  pico_copter.cpp
  sensor.cpp
  ekf.cpp
  lsm9ds1_reg.c
  pwm.cpp
  radio.cpp
  control.cpp
  modules/ina219.cpp
  modules/spi_ws2812.cpp
)

target_link_libraries(pico_copter 
  hardware_spi 
  hardware_pwm 
  hardware_uart 
  pico_stdlib
  hardware_i2c
  hardware_pio
  hardware_dma
  pico_multicore)


target_include_directories(pico_copter PRIVATE ${CMAKE_CURRENT_LIST_DIR}/modules)

pico_enable_stdio_usb(pico_copter 1)
pico_enable_stdio_uart(pico_copter 0)

pico_add_extra_outputs(pico_copter)


include_directories(${CMAKE_CURRENT_LIST_DIR}/modules/altitude)

# Include ToF modules and ST driver
include_directories(${CMAKE_CURRENT_LIST_DIR}/modules/tof)
include_directories(${CMAKE_CURRENT_LIST_DIR}/third_party/ST_VL53L1X)

# Add ToF and ST sources
target_sources(pico_copter PRIVATE
  modules/altitude_kf.cpp
  modules/tof/tof_bridge.cpp
  modules/tof/tof_vl53l1x.cpp
  third_party/ST_VL53L1X/VL53L1X_api.c
  third_party/ST_VL53L1X/vl53l1_platform.c
)
# --- WS2812 LED strip ---
target_sources(pico_copter PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/modules/ws2812/WS2812.cpp
    ${CMAKE_CURRENT_LIST_DIR}/modules/rgbled/rgbled.cpp
)

# 必要ならインクルードパス
target_include_directories(pico_copter PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/modules/ws2812
    ${CMAKE_CURRENT_LIST_DIR}/modules/rgbled
)
pico_generate_pio_header(pico_copter
    ${CMAKE_CURRENT_LIST_DIR}/modules/ws2812/WS2812.pio
)
