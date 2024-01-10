/**
 * @file main.cpp
 * @author dominic gasperini
 * @brief telemetry core
 * @version 0.9
 * @date 2023-08-01
 * 
 * @ref https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/libraries.html#apis
 */


/*
===============================================================================================
                                    Includes 
===============================================================================================
*/


#include <Arduino.h>
#include "rtc.h"
#include "rtc_clk_common.h"

#include <data_types.h>
#include <pin_config.h>


/*
===============================================================================================
                                    Definitions
===============================================================================================
*/


// tasks & timers
#define IO_READ_INTERVAL                200000      // 0.1 seconds in microseconds
#define SERIAL_UPDATE_INTERVAL          250000      // 0.25 seconds in microseconds
#define TASK_STACK_SIZE                 4096        // in bytes

// debug
#define ENABLE_DEBUG                    true       // master debug message control
#if ENABLE_DEBUG
  #define MAIN_LOOP_DELAY               1000        // delay in main loop
#else
  #define MAIN_LOOP_DELAY               1
#endif


/*
===============================================================================================
                                  Global Variables
===============================================================================================
*/


/**
 * @brief debugger structure used for organizing debug information
 * 
 */
Debugger debugger = {
  // debug toggle
  .debugEnabled = ENABLE_DEBUG,
  .serial_debugEnabled = false,
  .IO_debugEnabled = false,
  .scheduler_debugEnable = true,

  // I/O data
  .IO_data = {},

  // precharge data
  .prechargeState = PRECHARGE_OFF,

  // scheduler data
  .ioReadTaskCount = 0,
  .serialReadTaskCount = 0,
  .serialWriteTaskCount = 0,
};


/**
 * @brief the dataframe that describes the entire state of the car
 * 
 */
TelemetryCoreData telemetryCoreData = {
  // tractive data
  .tractiveCoreData = {
    .tractive = {
      .readyToDrive = false,
      .enableInverter = false,

      .prechargeState = PRECHARGE_OFF,

      .rinehartVoltage = 0.0f,
      .commandedTorque = 0,

      .driveDirection = false,    // forward is false | reverse is true (we run backwards)
      .driveMode = ECO,
      
      .currentSpeed = 0.0f,

      .tractionControlEnable = true,
      .tractionControlModifier = 1.00f,

      .coastRegen = 0,
      .brakeRegen = 0,
    },

    // sensor data
    .criticalSensors = {
      .imdFault = true,
      .bmsFault = true,

      .coolingTempIn = 0.0f,
      .coolingTempOut = 0.0f,
      .vicoreTemp = 0.0f,

      .glvReading = 0.0f,

      .frontWheelsSpeed = 0.0f,
      .frontWheelSpeedCount = 0,
      .frontWheelSpeedTime = 0,

      .brWheelSpeed = 0.0f,
      .brWheelSpeedCount = 0,
      .brWheelSpeedTime = 0,

      .blWheelSpeed = 0.0f,
      .blWheelSpeedCount = 0,
      .blWheelSpeedTime = 0,
    },

    // inputs
    .inputs = {
      .pedal0 = 0,
      .pedal1 = 0,

      .frontBrake = 0,
      .rearBrake = 0,
    },

    // outputs
    .outputs = {
      .driveModeLED = ECO,

      .brakeLightEnable = false,

      .fansEnable = false,

      .buzzerEnable = false,
    },

    // orion
    .orion = {
      .batteryChargeState = 0,

      .busVoltage = 0,

      .packCurrent = 0.0f,

      .minCellVoltage = 0.0f,
      .maxCellVoltage = 0.0f,
      .minCellTemp = 0.0f,
      .maxCellTemp = 0.0f,
    }
  },

  // telemetry data

  // wheel damper sensors
  .dampers = {
    .frSuspensionDamper = 0,
    .flSuspensionDamper = 0,
    .brSuspensionDamper = 0,
    .blSuspensionDamper = 0,
  },

  // tire temperature sensors
  .tireTemp = {
    .frTireTemp = 0,
    .flTireTemp = 0,
    .brTireTemp = 0,
    .blTireTemp = 0,
  },

  // suspension strain sensors
  .strain = {
    .frStrain1 = 0,
    .flStrain1 = 0,
    .brStrain1 = 0,
    .blStrain1 = 0,

    .frStrain2 = 0,
    .flStrain2 = 0,
    .brStrain2 = 0,
    .blStrain2 = 0,
  },

  // steering wheel sensors
  .steering = {
    .steeringWheelDeflection = 0,
  },

  // inertial monitoring unit data
  .imu = {
    .xAcceleration = 0.0f,
    .yAcceleration = 0.0f,
    .zAcceleration = 0.0f,
    .xGyro = 0.0f,
    .yGyro = 0.0f,
    .zGyro = 0.0f,
  },

  // global positioning system data
  .gps = {
    .latitide = 0.0f,
    .longitude = 0.0f,
    .altitude = 0.0f,
    .speed = 0.0f,
    .year = 0,
    .month = 0,
    .day = 0,
  }
};



// RTOS Tasks
TaskHandle_t xHandleIORead = NULL;
TaskHandle_t xHandleReadSerial = NULL;
TaskHandle_t xHandleWriteSerial = NULL;


// Hardware Timers
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


// GPS


// IMU


// Raspberry Pi


/*
===============================================================================================
                                    Function Declarations 
===============================================================================================
*/


// callbacks
void IOReadCallback();
void SerialUpdateCallback();

// tasks
void IOReadTask(void* pvParameters);
void SerialReadTask(void* pvParameters);
void SerialWriteTask(void* pvParameters);


/*
===============================================================================================
                                            Setup 
===============================================================================================
*/


void setup() {
  // set power configuration
  esp_pm_configure(&power_configuration);

  if (debugger.debugEnabled) {
    // delay startup by 3 seconds
    vTaskDelay(3000);
  }

  // ----------------------- initialize serial connection --------------------- //
  Serial.begin(9600);
  Serial.printf("\n\n|--- STARTING SETUP ---|\n\n");

  // setup managment struct
  struct setup
  {
    bool ioActive = false;
    bool gpsActive = false;
    bool imuActive = false;
    bool rpiComActive = false;
    bool lora = false;
  };
  setup setup;
  // -------------------------- initialize GPIO ------------------------------ //
  analogReadResolution(12);
  
  // inputs
  pinMode(FR_SUSPENSION_DAMPER_PIN, INPUT);
  pinMode(FL_SUSPENSION_DAMPER_PIN, INPUT);
  pinMode(BR_SUSPENSION_DAMPER_PIN, INPUT);
  pinMode(BL_SUSPENSION_DAMPER_PIN, INPUT);

  pinMode(FR_TIRE_TEMP_PIN, INPUT);
  pinMode(FL_TIRE_TEMP_PIN, INPUT);
  pinMode(BR_TIRE_TEMP_PIN, INPUT);
  pinMode(BL_TIRE_TEMP_PIN, INPUT);

  pinMode(FR_STRAIN_1_PIN, INPUT);
  pinMode(FL_STRAIN_1_PIN, INPUT);
  pinMode(BR_STRAIN_1_PIN, INPUT);
  pinMode(BL_STRAIN_1_PIN, INPUT);

  pinMode(FR_STRAIN_2_PIN, INPUT);
  pinMode(FL_STRAIN_2_PIN, INPUT);
  pinMode(BR_STRAIN_2_PIN, INPUT);
  pinMode(BL_STRAIN_2_PIN, INPUT);
  
  pinMode(STEERING_DEFLECTION_PIN, INPUT);


  // outputs


  Serial.printf("GPIO INIT [ SUCCESS ]\n");
  setup.ioActive = true;
  // -------------------------------------------------------------------------- //
  

  // -------------------------- initialize LoRa ------------------------------- //

  // -------------------------------------------------------------------------- //


  // -------------------------- initialize GPS -------------------------------- //

  // -------------------------------------------------------------------------- //


  // -------------------------- initialize IMU -------------------------------- //

  // -------------------------------------------------------------------------- /


  // ------------------------------- Scheduler & Task Status --------------------------------- //
  // start tasks
  IOReadCallback();
  SerialUpdateCallback();

  // scheduler status
  if (xTaskGetSchedulerState() == 2) {
    Serial.printf("\nScheduler Status: RUNNING\n");

    // clock frequency
    rtc_cpu_freq_config_t clock_config;
    rtc_clk_cpu_freq_get_config(&clock_config);
    Serial.printf("CPU Frequency: %dMHz\n", clock_config.freq_mhz);
  }
  else {
    Serial.printf("\nScheduler STATUS: FAILED\nHALTING OPERATIONS");
    while (1) {};
  }
  Serial.printf("\n\n|--- END SETUP ---|\n\n");
  // ---------------------------------------------------------------------------------------- //
}


/*
===============================================================================================
                                    Callback Functions
===============================================================================================
*/


/**
 * @brief callback function for queueing I/O read and write tasks
 * 
 * @param args arguments to be passed to the task
 */
void IOReadCallback() 
{
  portENTER_CRITICAL_ISR(&timerMux);

  // inits 
  static uint8_t ucParameterToPass;

  // queue task
  xTaskCreate(IOReadTask, "Read-IO", TASK_STACK_SIZE, &ucParameterToPass, tskIDLE_PRIORITY, &xHandleIORead);

  // kill task if error returned
  if (xHandleIORead != NULL) {
    vTaskDelete(xHandleIORead);
  }
  
  portEXIT_CRITICAL_ISR(&timerMux);

  return;
}


/**
 * @brief callback function for queueing serial read and write tasks
 * 
 * @param args arguments to be passed to the task
 */
void SerialUpdateCallback() {
  portENTER_CRITICAL_ISR(&timerMux);

  // inits
  static uint8_t ucParameterToPassRead;
  static uint8_t ucParameterToPassWrite;

  // queue tasks
  xTaskCreate(SerialReadTask, "Read-Serial", TASK_STACK_SIZE, &ucParameterToPassRead, tskIDLE_PRIORITY, &xHandleReadSerial);
  xTaskCreate(SerialWriteTask, "Write-Serial", TASK_STACK_SIZE, &ucParameterToPassWrite, tskIDLE_PRIORITY, &xHandleWriteSerial);

  // kill tasks if error is returned
  if (xHandleReadSerial != NULL) {
    vTaskDelete(xHandleReadSerial);
  }
  if (xHandleWriteSerial != NULL) {
    vTaskDelete(xHandleWriteSerial);
  }

  portEXIT_CRITICAL_ISR(&timerMux);

  return;
};


/*
===============================================================================================
                                FreeRTOS Task Functions
===============================================================================================
*/


/**
 * @brief reads I/O
 * 
 * @param pvParameters parameters passed to task
 */
void IOReadTask(void* pvParameters)
{
  for (;;) 
  {
    // dampers

    // tire temps

    // strain gauges

    // steering


    // debugging
    if (debugger.debugEnabled) {
      debugger.IO_data = telemetryCoreData;
      debugger.ioReadTaskCount++;
    }
  }
}


/**
 * @brief reads the various serial busses
 * 
 * @param pvParameters parameters passed to task
 */
void SerialReadTask(void* pvParameters) {
  for (;;) 
  {
    // tractive core data


    // GPS


    // IMU
  }
}


/**
 * @brief writes to the various serial busses
 * 
 * @param pvParameters parameters passed to task
 */
void SerialWriteTask(void* pvParameters) {
  for (;;) 
  {  
    // LoRa update


    // HUD update
  }

  // end task
  vTaskDelete(NULL);
}


/*
===============================================================================================
                                    Main Loop
===============================================================================================
*/


/**
 * @brief the main loop of the program
 * 
 */
void loop()
{
  // everything is managed by RTOS, so nothing really happens here!
  vTaskDelay(MAIN_LOOP_DELAY);    // prevent watchdog from getting upset

  // debugging
  if (debugger.debugEnabled) {
    PrintDebug();
  }
}


/* 
===============================================================================================
                                    DEBUG FUNCTIONS
================================================================================================
*/


/**
 * @brief some nice in-depth debugging for I/O
 * 
 */
void PrintIODebug() {
  Serial.printf("\n--- START I/O DEBUG ---\n");

  // INPUTS


  // OUTPUTS


  Serial.printf("\n--- END I/O DEBUG ---\n");
}


/**
 * @brief some nice in-depth debugging for serial
 * 
 */
void PrintSerialDebug() {
  Serial.printf("\n--- START SERIAL DEBUG ---\n");

  // INPUTS


  // OUTPUTS


  Serial.printf("\n--- END SERIAL DEBUG ---\n");
}


/**
 * @brief manages toggle-able debug settings
 * 
 */
void PrintDebug() {
  // serial   
  if (debugger.serial_debugEnabled) {
    PrintSerialDebug();
  }

  // I/O
  if (debugger.IO_debugEnabled) {
    PrintIODebug();
  }

  // scheduler
  if (debugger.scheduler_debugEnable) {
    Serial.printf("read io: %d | read serial: %d | write serial: %d\n", debugger.ioReadTaskCount, debugger.serialReadTaskCount, debugger.serialWriteTaskCount);
  }
}