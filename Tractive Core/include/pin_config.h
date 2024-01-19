/**
 * @file pinConfig.h
 * @author dominic gasperini 
 * @brief this file holds the pin layout for the board I/O
 * @version 1.5
 * @date 2024-01-19
 */


/*
===============================================================================================
                                    Includes 
===============================================================================================
*/


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
                    Pin Definitions
===========================================================
*/


// Sensors
#define COOLING_IN_TEMP_PIN                 1
#define COOLING_OUT_TEMP_PIN                2
#define BMS_FAULT_PIN                       16
#define IMD_FAULT_PIN                       15
#define VICORE_FAULT_PIN                    13

#define FR_HALL_EFFECT_PIN                  47
#define FL_HALL_EFFECT_PIN                  21
#define BR_HALL_EFFECT_PIN                  20
#define BL_HALL_EFFECT_PIN                  19


// Inputs
#define PEDAL_0_PIN                         4
#define PEDAL_1_PIN                         5
#define FRONT_BRAKE_PIN                     6
#define REAR_BRAKE_PIN                      7

#define COAST_REGEN_PIN                     14
#define BRAKE_REGEN_PIN                     18

#define START_BUTTON_PIN                    35
#define DRIVE_MODE_BUTTON_PIN               40
#define TRACTION_CONTROL_SWITCH_PIN         8


// TWAI
#define TWAI_RX_PIN                         42
#define TWAI_TX_PIN                         41


// Outputs
#define RTD_LED_PIN                         11
#define DRIVE_MODE_LED_PIN                  39

#define BMS_FAULT_LED_PIN                   10     
#define IMD_FAULT_LED_PIN                   9

#define VICORE_ENABLE_PIN                   12

#define BUZZER_PIN                          17

#define BRAKE_LIGHT_PIN                     38

#define FAN_ENABLE_PIN                      37