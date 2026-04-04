/**
 * @file pinConfig.h
 * @author dominic gasperini 
 * @brief this file holds the pin layout for the board I/O
 * @version 1.6
 * @date 2024-01-22
 */

/*
===============================================================================================
                                    Includes 
===============================================================================================
*/

#pragma once
#include <esp_pm.h>

/*
===========================================================
                    Power Configuration
===========================================================
*/

esp_pm_config_esp32s3_t power_configuration {
    .max_freq_mhz = 240,   
    .min_freq_mhz = 240,
    .light_sleep_enable = false,
};

/*
===========================================================
                    Pins
===========================================================
*/

// Sensors
#define PUMP_TEMP_IN_PIN                    1
#define PUMP_TEMP_OUT_PIN                   2
#define BMS_FAULT_PIN                       15
#define IMD_FAULT_PIN                       16
#define VICORE_FAULT_PIN                    12

#define FR_HALL_EFFECT_PIN                  47
#define FL_HALL_EFFECT_PIN                  21
#define BR_HALL_EFFECT_PIN                  20
#define BL_HALL_EFFECT_PIN                  19

//Inputs
#define PEDAL_0_PIN                         4
#define PEDAL_1_PIN                         5
#define FRONT_BRAKE_PIN                     6
#define REAR_BRAKE_PIN                      7

#define COAST_REGEN_PIN                     17
#define BRAKE_REGEN_PIN                     18

#define START_BUTTON_PIN                    48
#define DRIVE_MODE_BUTTON_PIN               40
#define TRACTION_CONTROL_SWITCH_PIN         8

//TWAI
#define TWAI_RX_PIN                         42
#define TWAI_TX_PIN                         41

//Telemetry Connection
#define I2C_TX_PIN                          36
#define I2C_RX_PIN                          35

//Outputs
#define RTD_LED_PIN                         11
#define DRIVE_MODE_LED_PIN                  39

#define BMS_FAULT_IND_PIN                   10
#define IMD_FAULT_IND_PIN                   9
#define TSSI_FAULT_IND_PIN                  3

#define VICORE_ENABLE_PIN                   13

#define BUZZER_PIN                          14

#define BRAKE_LIGHT_PIN                     37

#define FAN_ENABLE_PIN                      38

/*
===========================================================
                    Configs
===========================================================
*/

constexpr float TRAC_CON_ENABLE_BIAS =       0.1;   // the activation threshold of traction control expressed as a percentage difference between front and rear wheel speeds
constexpr float TRAC_CON_CORNER_ENABLE =     0.7;   // percent difference between rear wheel speeds needed to enable cornering traction control
constexpr float TRAC_CON_MAX_MOD =           0.75;        // maximum power reduction percentage that traction control can apply
constexpr float TRAC_CON_MOD_STEP =          0.01;      // the step in which each iteration of traction control modified the throttle response
constexpr float TRAC_CON_STEP_INCREASE_MOD = 3; // the decrease in traction control management as a multiple of the increasing step

constexpr float TIRE_DIAMETER =              20.0;         // diameter of the vehicle's tires in inches
constexpr float WHEEL_RPM_CALC_THRESHOLD =   5; // the number of times the hall effect sensor is tripped before calculating vehicle speed

constexpr float BRAKE_LIGHT_THRESHOLD =      50; // the threshold that must be crossed for the brake to be considered active

constexpr float PEDAL0_MIN =                 0;       // minimum value pedal 0 can read as
constexpr float PEDAL0_MAX =                 255;     // maximum value pedal 0 can read as
constexpr float PEDAL1_MIN =                 0;       // minimum value pedal 1 can read as
constexpr float PEDAL1_MAX =                 255;     // maximum value pedal 1 can read as
constexpr float PEDAL_DEADBAND =             15; // ~5% of PEDAL_MAX
constexpr float PEDAL_DELTA =                0.15;  // difference between pedal value for cutoff threshold computed by percent difference

constexpr int MAX_TORQUE =                   225;        // MAX TORQUE RINEHART CAN ACCEPT, DO NOT EXCEED 230!!!
constexpr float MAX_REVERSE_TORQUE =         25; // for limiting speed while in reverse

constexpr float MIN_BUS_VOLTAGE =            150; // min bus voltage

constexpr float COOLING_ENABLE_THRESHOLD =   35;  // in degrees C
constexpr float COOLING_DISABLE_THRESHOLD =  30; // in degrees C

constexpr float PRECHARGE_FLOOR =            0.8;  // percentage of bus voltage rinehart should be at
constexpr float BUZZER_DURATION =            2000; // in milliseconds

constexpr uint16_t TELEMETRY_CORE_I2C_ADDR = 0x10; // address in hex
constexpr unsigned long SERIAL_BAUD_RATE =   9600;        // baud rate
constexpr uint32_t I2C_FREQUENCY =           100000;         // frequency of bus

// TWAI
constexpr int RINE_MOTOR_INFO_ADDR =         0x0A5;    // get motor information from rinehart
constexpr int RINE_VOLT_INFO_ADDR =          0x0A7;     // get rinehart electrical information
constexpr int RINE_BUS_INFO_ADDR =           0x0AA;      // get rinehart relay information
constexpr int RINE_MOTOR_CONTROL_ADDR =      0x0C0; // motor command address
constexpr int RINE_BUS_CONTROL_ADDR =        0x0C1;   // control rinehart relay states
constexpr int BMS_GEN_DATA_ADDR =            0x6B0;  // important BMS data
constexpr int BMS_CELL_DATA_ADDR =           0x6B2; // cell data

// tasks
constexpr TickType_t IO_WRITE_REFRESH_RATE =         9;         // measured in ticks (RTOS ticks interrupt at 1 kHz)
constexpr TickType_t IO_READ_REFRESH_RATE =          2;          // measured in ticks (RTOS ticks interrupt at 1 kHz)
constexpr TickType_t TWAI_WRITE_REFRESH_RATE =       8;       // measured in ticks (RTOS ticks interrupt at 1 kHz)
constexpr TickType_t TWAI_READ_REFRESH_RATE =        8;        // measured in ticks (RTOS ticks interrupt at 1 kHz)
constexpr TickType_t PRECHARGE_REFRESH_RATE =        225;      // measured in ticks (RTOS ticks interrupt at 1 kHz)
constexpr TickType_t TELEMETRY_UPDATE_REFRESH_RATE = 9; // measured in ticks (RTOS ticks interrupt at 1 kHz)
constexpr TickType_t DEBUG_REFRESH_RATE =            1000;         // measured in ticks (RTOS ticks interrupt at 1 kHz)

constexpr float TWAI_BLOCK_DELAY =           1; // time to block to complete function call in FreeRTOS ticks

constexpr uint32_t TASK_STACK_SIZE =         20000; // in bytesF

// debug
constexpr bool ENABLE_DEBUG = true; // master debug message control