/**
 * @file pinConfig.h
 * @author dominic gasperini 
 * @brief this file holds the pin layout for the board I/O
 * @version 1.0
 * @date 2024-01-23 
 */


/*
============================================================
                        Includes 
============================================================
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
#define FR_SUSPENSION_DAMPER_PIN        4
#define FR_TIRE_TEMP_PIN                5
#define FR_STRAIN_1_PIN                 6
#define FR_STRAIN_2_PIN                 7

#define FL_SUSPENSION_DAMPER_PIN        15
#define FL_TIRE_TEMP_PIN                16
#define FL_STRAIN_1_PIN                 17
#define FL_STRAIN_2_PIN                 18

#define BR_SUSPENSION_DAMPER_PIN        9
#define BR_TIRE_TEMP_PIN                10
#define BR_STRAIN_1_PIN                 11
#define BR_STRAIN_2_PIN                 12

#define BL_SUSPENSION_DAMPER_PIN        13
#define BL_TIRE_TEMP_PIN                17
#define BL_STRAIN_1_PIN                 1
#define BL_STRAIN_2_PIN                 2

#define STEERING_DEFLECTION_PIN         8


// Tractive Connection
#define I2C_TX_PIN                      41
#define I2C_RX_PIN                      42


// HUD Connection
#define RPI_TX_PIN                      48
#define RPI_RX_PIN                      47


// Advanced Sensors
#define GPS_TX_PIN                      40
#define GPS_RX_PIN                      39

#define IMU_RX_PIN                      38
#define IMU_TX_PIN                      37

#define LORA_TX_PIN                     36
#define LORA_RX_PIN                     35