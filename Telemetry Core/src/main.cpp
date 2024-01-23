/**
 * @file main.cpp
 * @author dominic gasperini
 * @brief telemetry core
 * @version 1.0
 * @date 2024-01-23
 * 
 * @ref https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/libraries.html#apis      (api and hal docs)
 * @ref https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/_images/ESP32-S3_DevKitC-1_pinlayout.jpg  (pinout & overview)
 * @ref https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos_idf.html      (FreeRTOS for ESP32 docs)
 */


/*
===============================================================================================
                                    Includes 
===============================================================================================
*/


#include <Arduino.h>
#include "driver/twai.h"
#include "rtc.h"
#include "rtc_clk_common.h"
#include <vector>

#include <data_types.h>
#include <pin_config.h>


/*
===============================================================================================
                                    Definitions
===============================================================================================
*/


// tasks
#define IO_READ_REFRESH_RATE            9           // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define SERIAL_WRITE_REFRESH_RATE       8           // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define SERIAL_READ_REFRESH_RATE        8           // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define DEBUG_REFRESH_RATE              1000        // measured in ticks (RTOS ticks interrupt at 1 kHz)

#define TWAI_BLOCK_DELAY                1           // time to block to complete function call in FreeRTOS ticks

#define TASK_STACK_SIZE                 20000       // in bytes


// general
#define SERIAL_BAUD_RATE                9600        // baud rate


// debug
#define ENABLE_DEBUG                    true       // master debug message control


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
  .twai_debugEnable = false,
  .scheduler_debugEnable = true,

  // I/O data
  .IO_data = {},

  // scheduler data
  .ioReadTaskCount = 0,
  .serialReadTaskCount = 0,
  .serialWriteTaskCount = 0,
  .twaiReadTaskCount = 0,

  .ioReadTaskPreviousCount = 0,
  .serialReadTaskPreviousCount = 0,
  .serialWriteTaskPreviousCount = 0,
  .twaiReadTaskPreviousCount = 0,
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
    .sensors = {
      .imdFault = true,
      .bmsFault = true,
      .vicoreFault = false,

      .coolingTempIn = 0.0f,
      .coolingTempOut = 0.0f,

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


// Mutex
SemaphoreHandle_t xMutex = NULL;


// RTOS Task Handles
TaskHandle_t xHandleIORead = NULL;
TaskHandle_t xHandleSerialRead = NULL;
TaskHandle_t xHandleSerialWrite = NULL;

TaskHandle_t xHandleDebug = NULL;


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


// tasks
void IOReadTask(void* pvParameters);
void SerialReadTask(void* pvParameters);
void SerialWriteTask(void* pvParameters);
void DebugTask(void* pvParameters);


// helpers
DriveMode NumberToDriveMode(uint8_t value);
PrechargeStates NumberToPrechargeState(uint8_t value);
String TaskStateToString(eTaskState state);


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

  // setup managment struct
  struct Setup
  {
    bool ioActive = false;
    bool gpsActive = false;
    bool imuActive = false;
    bool rpiComActive = false;
    bool loraActive = false;
    bool serialActive = false;
  };
  Setup setup;

  // ----------------------- initialize serial connection --------------------- //
  Serial.begin(9600);
  Serial.printf("\n\n|--- STARTING SETUP ---|\n\n");

  Serial2.begin(SERIAL_BAUD_RATE, SERIAL_8N1, SERIAL_TRACTIVE_RX_PIN, SERIAL_TRACTIVE_TX_PIN);

  Serial.printf("SERIAL INIT [ SUCCESS ]\n");
  setup.serialActive = true;

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
  
  // --------------------- initialize RPi Connection -------------------------- //

  Serial.printf("RPi INIT [ SUCCESS ]\n");
  setup.rpiComActive = true;
  // -------------------------------------------------------------------------- //


  // -------------------------- initialize LoRa ------------------------------- //

  Serial.printf("LORA INIT [ SUCCESS ]\n");
  setup.loraActive = true;
  // -------------------------------------------------------------------------- //

  
  // -------------------------- initialize GPS -------------------------------- //

  Serial.printf("GPS INIT [ SUCCESS ]\n");

  setup.gpsActive = true;
  // -------------------------------------------------------------------------- //


  // -------------------------- initialize IMU -------------------------------- //

  Serial.printf("IMU INIT [ SUCCESS ]\n");
  setup.imuActive = true;
  // -------------------------------------------------------------------------- //


  // ------------------------------- Scheduler & Task Status --------------------------------- //
  // init mutex
  xMutex = xSemaphoreCreateMutex();
  
  // task setup status
  Serial.printf("\nTask Setup Status:\n");
  Serial.printf("I/O TASK SETUP: %s\n", setup.ioActive ? "COMPLETE" : "FAILED");
  Serial.printf("SERIAL TASK SETUP: %s\n", setup.ioActive ? "COMPLETE" : "FAILED");

  if (xMutex != NULL) {
    // start tasks
    if (setup.ioActive) {
      xTaskCreatePinnedToCore(IOReadTask, "IO-Read", TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandleIORead, 0);
    }

    if (setup.rpiComActive && setup.gpsActive && setup.imuActive && setup.loraActive && setup.serialActive) {
      xTaskCreatePinnedToCore(SerialReadTask, "Serial-Read", TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandleSerialRead, 1);
      xTaskCreatePinnedToCore(SerialWriteTask, "Serial-Write", TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandleSerialWrite, 1);
    }  

    if (debugger.debugEnabled == true) {
      xTaskCreate(DebugTask, "Debugger", TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandleDebug);
    }
  }
  else {
    Serial.printf("FAILED TO INIT MUTEX!\nHALTING OPERATIONS!");
    while (1) {};
  }

  // task status
  Serial.printf("\nTask Status:\n");
  if (xHandleIORead != NULL)
    Serial.printf("I/O READ TASK STATUS: %s\n", TaskStateToString(eTaskGetState(xHandleIORead)));
  else 
    Serial.printf("I/O READ TASK STATUS: DISABLED!\n");
  
  if (xHandleSerialRead != NULL) 
    Serial.printf("SERIAL READ TASK STATUS %s\n", TaskStateToString(eTaskGetState(xHandleSerialRead)));
  else
    Serial.printf("SERIAL READ TASK STAUS: DISABLED!\n");

  if (xHandleSerialWrite != NULL) 
    Serial.printf("SERIAL WRITE TASK STATUS %s\n", TaskStateToString(eTaskGetState(xHandleSerialWrite)));
  else
    Serial.printf("SERIAL WRTIE TASK STAUS: DISABLED!\n");


  // scheduler status
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
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
  Serial.printf("\n|--- END SETUP ---|\n\n");
  // ---------------------------------------------------------------------------------------- //
}


/*
===============================================================================================
                                FreeRTOS Task Functions
===============================================================================================
*/


/**
 * @brief reads I/O
 * @param pvParameters parameters passed to task
 */
void IOReadTask(void* pvParameters)
{
  for (;;) 
  {
    if (xSemaphoreTake(xMutex, (TickType_t) 10) == pdTRUE) {
      // dampers

      // tire temps

      // strain gauges

      // steering

      // release mutex!
      xSemaphoreGive(xMutex);
    }

    // debugging
    if (debugger.debugEnabled) {
      debugger.IO_data = telemetryCoreData;
      debugger.ioReadTaskCount++;
    }

    // limit task refresh rate
    vTaskDelay(IO_READ_REFRESH_RATE);
  }
}


/**
 * @brief reads the various serial busses
 * @param pvParameters parameters passed to task
 */
void SerialReadTask(void* pvParameters)
{
  for (;;) 
  {
    if (xSemaphoreTake(xMutex, (TickType_t) 10) == pdTRUE)
    {
      // GPS


      // IMU

      // release mutex!
      xSemaphoreGive(xMutex);
    }

    // debugging
    if (debugger.debugEnabled) {
      debugger.serialReadTaskCount++;
    }

    // limit task refresh rate
    vTaskDelay(SERIAL_READ_REFRESH_RATE);
  }
}


/**
 * @brief writes to the various serial busses
 * @param pvParameters parameters passed to task
 */
void SerialWriteTask(void* pvParameters)
{
  for (;;) 
  {  
    if (xSemaphoreTake(xMutex, (TickType_t) 10) == pdTRUE)
    {
      // LoRa update


      // HUD update

      // release mutex!
      xSemaphoreGive(xMutex);
    }

    // debugging
    if (debugger.debugEnabled) {
      debugger.serialWriteTaskCount++;
    }

    // limit task refresh rate
    vTaskDelay(SERIAL_WRITE_REFRESH_RATE);
  }
}


/**
 * @brief manages toggle-able debug settings & scheduler debugging 
 */
void DebugTask(void* pvParameters) {
  for (;;) 
  {
    // I/O
    if (debugger.IO_debugEnabled) {
      PrintIODebug();
    }

    // Scheduler
    if (debugger.scheduler_debugEnable) {
      PrintSchedulerDebug();
    }

    // limit refresh rate
    vTaskDelay(DEBUG_REFRESH_RATE);
  }
}


/*
===============================================================================================
                                    Main Loop
===============================================================================================
*/


/**
 * @brief the main loop of the program
 */
void loop()
{
  // everything is managed by RTOS, so nothing really happens here!
  vTaskDelay(1);    // prevent watchdog from getting upset
}


/*
===============================================================================================
                                    Helper Functions
===============================================================================================
*/


/**
 * convert a number to drive mode
*/
DriveMode NumberToDriveMode(uint8_t value)
{
  // inits
  DriveMode mode;

  // convert
  if (value == 1) {
    mode = SLOW;
  }
  if (value == 2) {
    mode = ECO;
  }
  if (value == 3) {
    mode = FAST;
  }

  return mode;
}


/**
 * convert a number to precharge state
*/
PrechargeStates NumberToPrechargeState(uint8_t value)
{
  // init 
  PrechargeStates state;

  // convert
  if (value == 1) {
    state = PRECHARGE_OFF;
  }
  if (value == 2) {
    state = PRECHARGE_ON;
  }
  if (value == 3) {
    state = PRECHARGE_DONE;
  }
  if (value == 4) {
    state = PRECHARGE_ERROR;
  }

  return state;
}


/**
 * 
*/
String TaskStateToString(eTaskState state) {
  // init
  String stateStr;

  // get state
  switch (state)
  {
  case eReady:
    stateStr = "RUNNING";        
  break;

  case eBlocked:
    stateStr = "BLOCKED";        
  break;

  case eSuspended:
    stateStr = "SUSPENDED";        
  break;

  case eDeleted:
    stateStr = "DELETED";        
  break;
  
  default:
    stateStr = "ERROR";        
    break;
  }

  return stateStr;
}


/* 
===============================================================================================
                                    DEBUG FUNCTIONS
================================================================================================
*/


/**
 * @brief some nice in-depth debugging for I/O
 */
void PrintIODebug()
{
  Serial.printf("\n--- START I/O DEBUG ---\n");

  // INPUTS


  // OUTPUTS


  Serial.printf("\n--- END I/O DEBUG ---\n");
}


/**
 * @brief some nice in-depth debugging for serial
 */
void PrintSerialDebug()
{
  Serial.printf("\n--- START SERIAL DEBUG ---\n");

  // INPUTS


  // OUTPUTS


  Serial.printf("\n--- END SERIAL DEBUG ---\n");
}


/**
 * @brief scheudler debugging
 */
void PrintSchedulerDebug()
{
  // inits
  std::vector<eTaskState> taskStates;
  std::vector<String> taskStatesStrings;
  std::vector<int> taskRefreshRate;
  int uptime = esp_rtc_get_time_us() / 1000000;

  // gather task information
  if (xHandleIORead != NULL) {
    taskStates.push_back(eTaskGetState(xHandleIORead));
  }
  if (xHandleSerialRead != NULL) {
    taskStates.push_back(eTaskGetState(xHandleSerialRead));
  }
  if (xHandleSerialWrite != NULL) {
    taskStates.push_back(eTaskGetState(xHandleSerialWrite));
  }

  taskRefreshRate.push_back(debugger.ioReadTaskCount - debugger.ioReadTaskPreviousCount);
  taskRefreshRate.push_back(debugger.serialReadTaskCount - debugger.serialReadTaskPreviousCount);
  taskRefreshRate.push_back(debugger.serialWriteTaskCount - debugger.serialWriteTaskPreviousCount);

  // make it usable
  for (int i = 0; i < taskStates.size() - 1; ++i) {
    taskStatesStrings.push_back(TaskStateToString(taskStates.at(i)));        
  }

  // print it
  Serial.printf("uptime: %d | read io:<%d Hz> (%d) | read serial:<%d Hz> (%d) | write serial:<%d Hz> (%d) \r",
    uptime, taskRefreshRate.at(0), debugger.ioReadTaskCount, taskRefreshRate.at(1), debugger.serialReadTaskCount,
    taskRefreshRate.at(2), debugger.serialWriteTaskCount);

  // update counters
  debugger.ioReadTaskPreviousCount = debugger.ioReadTaskCount;
  debugger.serialReadTaskPreviousCount = debugger.serialReadTaskCount;
  debugger.serialWriteTaskPreviousCount = debugger.serialWriteTaskCount;
}