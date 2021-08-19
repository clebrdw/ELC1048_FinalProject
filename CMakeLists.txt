cmake_minimum_required(VERSION 3.20)
project(ELC1048_FinalProject)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)
include_directories(dev)
include_directories(dev/ESP32_FreeRTOS)
include_directories(dev/LoRaMultipleCommunicationInterrupt)
include_directories(dist)
include_directories(dist/lora_client)
include_directories(dist/lora_server)

add_executable(ELC1048_FinalProject
        dev/ESP32_FreeRTOS/ESP32_FreeRTOS.ino
        dev/LoRaMultipleCommunicationInterrupt/LoRaMultipleCommunicationInterrupt.ino
        dist/lora_client/lora_client.ino
        dist/lora_server/lora_server.ino
        sensor.ino)
