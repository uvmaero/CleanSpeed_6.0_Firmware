/**
 * @file main.cpp
 * @author dominic gasperini
 * @brief telemetry core
 * @version 0.9
 * @date 2024-01-12
 * 
 * @ref https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/libraries.html#apis      (api and hal docs)
 * @ref https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/_images/ESP32-S3_DevKitC-1_pinlayout.jpg  (pinout & overview)
 * @ref https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos_idf.html      (FreeRTOS for ESP32 docs) */


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


// TWAI
#define TWAI_BLOCK_DELAY                5           // time to block to complete function call in FreeRTOS ticks (milliseconds)
#define TELEMETRY_TRACTIVE_1_ADDR       0x001
#define TELEMETRY_TRACTIVE_2_ADDR       0x002
#define TELEMETRY_SENSOR_ADDR           0x003       // telemetry board sensor data send address
#define TELEMETRY_INPUTS_ADDR           0x004
#define TELEMETRY_OUTPUTS_ADDR          0x005

// tasks & timers
#define TASK_STACK_SIZE                 4096        // in bytes
#define TASK_HIGH_PRIORITY              16          // max is 32 but its all relative so we don't need to use 32
#define TASK_MEDIUM_PRIORITY            8           // see above

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


// RTOS Tasks
TaskHandle_t xHandleIORead = NULL;
TaskHandle_t xHandleSerialRead = NULL;
TaskHandle_t xHandleSerialWrite = NULL;
TaskHandle_t xHandleTwaiRead = NULL;


// Hardware Timers
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


// TWAI
static const twai_general_config_t can_general_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TWAI_TX_PIN, (gpio_num_t)TWAI_RX_PIN, TWAI_MODE_NORMAL);
static const twai_timing_config_t can_timing_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t can_filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();


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
void TWAIReadCallback();
void SerialUpdateCallback();

// tasks
void IOReadTask(void* pvParameters);
void TWAIReadTask(void* pvParameters);
void SerialReadTask(void* pvParameters);
void SerialWriteTask(void* pvParameters);


// helpers
DriveMode NumberToDriveMode(uint8_t value);
PrechargeStates NumberToPrechargeState(uint8_t value);


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
  struct Setup
  {
    bool ioActive = false;
    bool gpsActive = false;
    bool imuActive = false;
    bool rpiComActive = false;
    bool loraActive = false;
    bool twaiActive = false;
  };
  Setup setup;
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

  // -------------------------------------------------------------------------- //


  // --------------------- initialize TWAI Controller -------------------------- //
  // install TWAI driver
  if(twai_driver_install(&can_general_config, &can_timing_config, &can_filter_config) == ESP_OK) {
    Serial.printf("TWAI DRIVER INSTALL [ SUCCESS ]\n");

    // start CAN bus
    if (twai_start() == ESP_OK) {
      Serial.printf("TWAI INIT [ SUCCESS ]\n");
      twai_reconfigure_alerts(TWAI_ALERT_ALL, NULL);

      setup.twaiActive = true;
    }

    else {
      Serial.printf("TWAI INIT [ FAILED ]\n");
    }
  }

  else {
    Serial.printf("TWAI DRIVER INSTALL [ FAILED ]\n");
  }
  // --------------------------------------------------------------------------- //


  // ------------------------------- Scheduler & Task Status --------------------------------- //
  // task setup status
  Serial.printf("\nTask Setup Status:\n");
  Serial.printf("I/O TASK SETUP: %s\n", setup.ioActive ? "COMPLETE" : "FAILED");
  Serial.printf("SERIAL TASK SETUP: %s\n", setup.ioActive ? "COMPLETE" : "FAILED");
  Serial.printf("TWAI TASK SETUP: %s\n", setup.twaiActive ? "COMPLETE" : "FAILED");

  // start tasks
  if (setup.ioActive) {
    IOReadCallback();
  }
  if (setup.rpiComActive && setup.gpsActive && setup.imuActive && setup.loraActive) {
    SerialUpdateCallback();
  }
  if (setup.twaiActive) {
    TWAIReadCallback();
  }

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
  Serial.printf("\n|--- END SETUP ---|\n\n");
  // ---------------------------------------------------------------------------------------- //
}


/*
===============================================================================================
                                    Callback Functions
===============================================================================================
*/


/**
 * @brief callback function for queueing I/O read and write tasks
 * @param args arguments to be passed to the task
 */
void IOReadCallback() 
{
  portENTER_CRITICAL_ISR(&timerMux);

  // inits 
  static uint8_t ucParameterToPass;

  // queue task
  xTaskCreate(IOReadTask, "Read-IO", TASK_STACK_SIZE, &ucParameterToPass, tskIDLE_PRIORITY, &xHandleIORead);
  
  portEXIT_CRITICAL_ISR(&timerMux);

  return;
}


/**
 * @brief callback function for queueing serial read and write tasks
 * @param args arguments to be passed to the task
 */
void SerialUpdateCallback() 
{
  portENTER_CRITICAL_ISR(&timerMux);

  // inits
  static uint8_t ucParameterToPassRead;
  static uint8_t ucParameterToPassWrite;

  // queue tasks
  xTaskCreate(SerialReadTask, "Read-Serial", TASK_STACK_SIZE, &ucParameterToPassRead, tskIDLE_PRIORITY, &xHandleSerialRead);
  xTaskCreate(SerialWriteTask, "Write-Serial", TASK_STACK_SIZE, &ucParameterToPassWrite, tskIDLE_PRIORITY, &xHandleSerialWrite);

  portEXIT_CRITICAL_ISR(&timerMux);

  return;
};


/**
 * @brief callback function for queueing twar read tasks
 * @param args arguments to be passed to the task
*/
void TWAIReadCallback() 
{
  portENTER_CRITICAL_ISR(&timerMux);

  // inits
  static uint8_t ucParameterToPassRead;

  // queue tasks
  xTaskCreate(TWAIReadTask, "Read-TWAI", TASK_STACK_SIZE, &ucParameterToPassRead, tskIDLE_PRIORITY, &xHandleTwaiRead);

  portEXIT_CRITICAL_ISR(&timerMux);

  return;
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
void SerialReadTask(void* pvParameters)
{
  for (;;) 
  {
    // GPS


    // IMU


    // debugging
    if (debugger.debugEnabled) {
      debugger.serialReadTaskCount++;
    }
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
    // LoRa update


    // HUD update


    // debugging
    if (debugger.debugEnabled) {
      debugger.serialWriteTaskCount++;
    }
  }
}


/**
 * read twai messages from tractice core
*/
void TWAIReadTask(void* pvParameters) 
{
  for (;;) 
  {
    // inits
    twai_message_t incomingMessage;

    // if rx queue is full clear it (this is bad, implement twai message filtering)
    uint32_t alerts;
    twai_read_alerts(&alerts, pdMS_TO_TICKS(TWAI_BLOCK_DELAY));
    if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
      twai_clear_receive_queue();
    }

    // check for new messages in the CAN buffer
    if (twai_receive(&incomingMessage, pdMS_TO_TICKS(TWAI_BLOCK_DELAY)) == ESP_OK) { // if there are messages to be read
      int id = incomingMessage.identifier;
      
      // parse out data
      switch (id) {
        // tractive 1
        case TELEMETRY_TRACTIVE_1_ADDR:
          telemetryCoreData.tractiveCoreData.tractive.readyToDrive = incomingMessage.data[0];
          telemetryCoreData.tractiveCoreData.tractive.enableInverter = incomingMessage.data[1];
          telemetryCoreData.tractiveCoreData.tractive.rinehartVoltage = incomingMessage.data[2];
          telemetryCoreData.tractiveCoreData.tractive.commandedTorque = incomingMessage.data[3];
          telemetryCoreData.tractiveCoreData.tractive.driveDirection = incomingMessage.data[4];
          telemetryCoreData.tractiveCoreData.tractive.tractionControlEnable = incomingMessage.data[5];
          telemetryCoreData.tractiveCoreData.tractive.tractionControlModifier = incomingMessage.data[6];
          telemetryCoreData.tractiveCoreData.tractive.currentSpeed = incomingMessage.data[7];
        break;

        // tractive 2
        case TELEMETRY_TRACTIVE_2_ADDR:
          telemetryCoreData.tractiveCoreData.tractive.coastRegen = incomingMessage.data[0];
          telemetryCoreData.tractiveCoreData.tractive.brakeRegen = incomingMessage.data[1];
          telemetryCoreData.tractiveCoreData.tractive.prechargeState = NumberToPrechargeState(incomingMessage.data[2]);
          telemetryCoreData.tractiveCoreData.tractive.driveMode = NumberToDriveMode(incomingMessage.data[3]);
        break;

        // sensors
        case TELEMETRY_SENSOR_ADDR:
          telemetryCoreData.tractiveCoreData.sensors.imdFault = incomingMessage.data[0];
          telemetryCoreData.tractiveCoreData.sensors.bmsFault = incomingMessage.data[1];
          telemetryCoreData.tractiveCoreData.sensors.vicoreFault = incomingMessage.data[2];
          telemetryCoreData.tractiveCoreData.sensors.coolingTempIn = incomingMessage.data[3];
          telemetryCoreData.tractiveCoreData.sensors.coolingTempOut = incomingMessage.data[4];
          telemetryCoreData.tractiveCoreData.sensors.frontWheelsSpeed = incomingMessage.data[5];
          telemetryCoreData.tractiveCoreData.sensors.brWheelSpeed = incomingMessage.data[6];
          telemetryCoreData.tractiveCoreData.sensors.blWheelSpeed = incomingMessage.data[7];
        break;

        // inputs
        case TELEMETRY_INPUTS_ADDR:
          telemetryCoreData.tractiveCoreData.inputs.pedal0 = incomingMessage.data[0];
          telemetryCoreData.tractiveCoreData.inputs.pedal1 = incomingMessage.data[1];
          telemetryCoreData.tractiveCoreData.inputs.frontBrake = incomingMessage.data[2];
          telemetryCoreData.tractiveCoreData.inputs.rearBrake = incomingMessage.data[3];
        break;

        // outputs
        case TELEMETRY_OUTPUTS_ADDR:
          telemetryCoreData.tractiveCoreData.outputs.vicoreEnable = incomingMessage.data[0];
          telemetryCoreData.tractiveCoreData.outputs.brakeLightEnable = incomingMessage.data[1];
          telemetryCoreData.tractiveCoreData.outputs.fansEnable = incomingMessage.data[2];
          telemetryCoreData.tractiveCoreData.outputs.buzzerEnable = incomingMessage.data[3];
        break;

        default:
          // do nothing
        break;
      }
    }

    // debugging
    if (debugger.debugEnabled) {
      debugger.twaiReadTaskCount++;
    }
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
  vTaskDelay(MAIN_LOOP_DELAY);    // prevent watchdog from getting upset

  // debugging
  if (debugger.debugEnabled) {
    PrintDebug();
  }
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
 * @brief manages toggle-able debug settings
 */
void PrintDebug()
{
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
    // inits
    std::vector<eTaskState> taskStates;
    std::vector<std::string> taskStatesStrings;
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
    if (xHandleTwaiRead != NULL) {
      taskStates.push_back(eTaskGetState(xHandleTwaiRead));
    }

    taskRefreshRate.push_back(debugger.ioReadTaskCount - debugger.ioReadTaskPreviousCount);
    taskRefreshRate.push_back(debugger.serialReadTaskCount - debugger.serialReadTaskPreviousCount);
    taskRefreshRate.push_back(debugger.serialWriteTaskCount - debugger.serialWriteTaskPreviousCount);
    taskRefreshRate.push_back(debugger.twaiReadTaskCount - debugger.twaiReadTaskPreviousCount);

    // make it usable
    for (int i = 0; i < taskStates.size() - 1; ++i) {
      switch (taskStates.at(i))
      {
      case eReady:
        taskStatesStrings.push_back("RUNNING");        
      break;

      case eBlocked:
        taskStatesStrings.push_back("BLOCKED");        
      break;

      case eSuspended:
        taskStatesStrings.push_back("SUSPENDED");        
      break;

      case eDeleted:
        taskStatesStrings.push_back("DELETED");        
      break;
      
      default:
        taskStatesStrings.push_back("ERROR");        
        break;
      }
    }

    // print it
    Serial.printf("uptime: %d | read io:<%d Hz> (%d) | read serial:<%d Hz> (%d) | write serial:<%d Hz> (%d) | read twai:<%d Hz> (%d) \r",
      uptime, debugger.ioReadTaskCount, taskRefreshRate.at(0), debugger.serialReadTaskCount, taskRefreshRate.at(1),
      debugger.serialWriteTaskCount, taskRefreshRate.at(2), debugger.twaiReadTaskCount, taskRefreshRate.at(3));

    // update counters
    debugger.ioReadTaskPreviousCount = debugger.ioReadTaskCount;
    debugger.twaiReadTaskPreviousCount = debugger.twaiReadTaskCount;
    debugger.serialReadTaskPreviousCount = debugger.serialReadTaskCount;
    debugger.serialWriteTaskPreviousCount = debugger.serialWriteTaskCount;
  }
}