# CleanSpeed 6.0 Firmware
All of the code that drives AERO's CleanSpeed 6.0 for the years of 2023-2024

## Overview
Each board is managed by an ESP32-S3, in the form of the official ESP32-S3 Dev-Kit-C V1 from Espressif. Its operations are managed by FreeRTOS and is clocked at 240Mhz.

## Tractive Core
### Description
This core manages all driver input and crictical functionalities like battery status and cooling
### I/O
#### Inputs
- Accelorator pedal
- Brake pedal 
- Ready to drive button
- Drive mode control button
- Fault reset buttons
#### Outputs
- Ready to drive LED
- Ready to drive buzzer
- Drive mode LED
- Cooling managment
### Interfaces
- CAN (TWAI)
- Serial Bus (1x)
### Libraries
- TWAI

## Telemetry Core
### Description
This core manages all non-essential sensor data, the ARDAN (our custom remote data aquisition system), and communications to the driver's HUD
### I/O
#### Inputs
- Wheel dampers
- Suspension strain gauges
- Tire temperature sensors
- Wheel speed sensors (hall effect)
- Steering wheel deflection
#### Outputs
- Serial to LoRa
- Serial to Driver HUD
### Interfaces
- Serial Bus (4x)
### Libraries
- TWAI
- LoRa
- GPS
- IMU
