
cmake_minimum_required(VERSION 3.13)
add_library(Adafruit_BusIO
Adafruit_BusIO_Register.cpp
Adafruit_I2CDevice.cpp
Adafruit_SPIDevice.cpp
)
target_include_directories(Adafruit_BusIO INTERFACE .)