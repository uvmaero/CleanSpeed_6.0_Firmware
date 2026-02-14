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
#include <Wire.h>
#include <SPI.h>
#include "driver/twai.h"
#include "rtc.h"
#include "rtc_clk_common.h"
#include <vector>
#include "LoRa.h"
#include <data_types.h>
#include <pin_config.h>

/*
===============================================================================================
                                    Definitions
===============================================================================================
*/

// task
int IO_READ_REFRESH_RATE = 9;       // measured in ticks (RTOS ticks interrupt at 1 kHz)
int SERIAL_WRITE_REFRESH_RATE = 9;  // measured in ticks (RTOS ticks interrupt at 1 kHz)
int SERIAL_READ_REFRESH_RATE = 9;  // measured in ticks (RTOS ticks interrupt at 1 kHz)
int DEBUG_REFRESH_RATE = 1000;      // measured in ticks (RTOS ticks interrupt at 1 kHz)
int TRACTIVE_READ_REFRESH_RATE = 9; // measured in ticks (RTOS ticks interrupt at 1 kHz)
int TASK_STACK_SIZE = 20000; // in bytes

// general
int SERIAL_BAUD_RATE = 9600;        // baud rate
uint8_t TELEMETRY_CORE_I2C_ADDR = 0x10; // address for i2c in hex
long I2C_FREQUENCY =  100000; // frequency of bus

//pins
gpio_num_t TWAI_TX_PIN = GPIO_NUM_0;
gpio_num_t TWAI_RX_PIN = GPIO_NUM_0;

// port aliases
#define SERIAL_DEBUG Serial
#define HUD Serial1

// debug
boolean ENABLE_DEBUG = true; // master debug message control

/*
===============================================================================================
                                  Global Variables
===============================================================================================
*/

// TWAI
static const twai_general_config_t can_general_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_PIN, TWAI_RX_PIN, TWAI_MODE_NORMAL);
static const twai_timing_config_t can_timing_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t can_filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

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

            .driveDirection = false, // forward is false | reverse is true (we run backwards)
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
        }},

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
TaskHandle_t xHandleDataRead = NULL;
TaskHandle_t xHandleDataWrite = NULL;
TaskHandle_t xHandleTractiveRead = NULL;

TaskHandle_t xHandleDebug = NULL;

// Hardware Timers
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// SPI
SPIClass hspi(HSPI);

// GPS

// IMU

// Raspberry Pi

/*
===============================================================================================
                                    Function Declarations
===============================================================================================
*/

// tasks
void IOReadTask(void *pvParameters);
void DataReadTask(void *pvParameters);
void DataWriteTask(void *pvParameters);
void TractiveReadTask(void *pvParameters);
void DebugTask(void *pvParameters);

// helpers
DriveMode NumberToDriveMode(uint8_t value);
PrechargeStates NumberToPrechargeState(uint8_t value);
String TaskStateToString(eTaskState state);

/*
===============================================================================================
                                            Setup
===============================================================================================
*/

void setup()
{
  // set power configuration
  esp_pm_configure(&power_configuration);

  if (debugger.debugEnabled)
  {
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
    bool i2cActive = false;
  };
  Setup setup;

  // ----------------------- initialize serial connection --------------------- //
  SERIAL_DEBUG.begin(9600);
  SERIAL_DEBUG.printf("\n\n|--- STARTING SETUP ---|\n\n");

  // -------------------------------------------------------------------------- //

  // --------------------- initialize TWAI Controller -------------------------- //
  // install TWAI driver
  if (twai_driver_install(&can_general_config, &can_timing_config, &can_filter_config) == ESP_OK)
  {
    SERIAL_DEBUG.printf("TWAI DRIVER INSTALL [ SUCCESS ]\n");

    // start CAN bus
    if (twai_start() == ESP_OK)
    {
      SERIAL_DEBUG.printf("TWAI INIT [ SUCCESS ]\n");
      twai_reconfigure_alerts(TWAI_ALERT_ALL, NULL);
    }

    else
    {
      SERIAL_DEBUG.printf("TWAI INIT [ FAILED ]\n");
    }
  }

  else
  {
    SERIAL_DEBUG.printf("TWAI DRIVER INSTALL [ FAILED ]\n");
  }
  // --------------------------------------------------------------------------- //


  // ----------------------- initialize i2c connection ----------------------- //

  if (Wire.begin(TELEMETRY_CORE_I2C_ADDR, I2C_RX_PIN, I2C_TX_PIN, 100000) == true)
  {
    Wire.setBufferSize(255); // change the buffer size to fit the data

    SERIAL_DEBUG.printf("TRACTIVE CONNECTION INIT [ SUCCESS ]\n");
    setup.i2cActive = true;
  }
  else
  {
    SERIAL_DEBUG.printf("TRACTIVE CONNECTION INIT [ FAILED ]\n");
  }
  // -------------------------------------------------------------------------- //

  // ----------------------- initialize spi connection ----------------------- //
  // init spi
  hspi.begin(SPI_SCLK, SPI_MISO, SPI_MOSI, SPI_SS);

  // set up slave select pins as outputs as the Arduino API
  // doesn't handle automatically pulling SS low
  pinMode(hspi.pinSS(), OUTPUT); // HSPI SS

  SERIAL_DEBUG.printf("SPI INIT [ SUCCESS ]\n");
  setup.loraActive;
  // -------------------------------------------------------------------------- //

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

  SERIAL_DEBUG.printf("GPIO INIT [ SUCCESS ]\n");
  setup.ioActive = true;
  // -------------------------------------------------------------------------- //

  // --------------------- initialize RPi Connection -------------------------- //
  HUD.begin(9600, 134217756U, RPI_RX_PIN, RPI_TX_PIN);
  setup.rpiComActive = true;
  SERIAL_DEBUG.printf("RPi INIT [ SUCCESS ]\n");
  // -------------------------------------------------------------------------- //

  // -------------------------- initialize LoRa ------------------------------- //
  LoRa.setSPI(hspi);
  LoRa.setPins(SPI_SS);
  if (LoRa.begin(915E6)) // US frequency band
  {
    SERIAL_DEBUG.printf("LORA INIT [ SUCCESS ]\n");
    setup.loraActive = true;
  }
  else
  {
    SERIAL_DEBUG.printf("LORA INIT [ FAILED ]\n");
  }
  // -------------------------------------------------------------------------- //

  // -------------------------- initialize GPS -------------------------------- //

  SERIAL_DEBUG.printf("GPS INIT [ SUCCESS ]\n");

  setup.gpsActive = true;
  // -------------------------------------------------------------------------- //

  // -------------------------- initialize IMU -------------------------------- //

  SERIAL_DEBUG.printf("IMU INIT [ SUCCESS ]\n");
  setup.imuActive = true;
  // -------------------------------------------------------------------------- //

  // ------------------------------- Scheduler & Task Status --------------------------------- //
  // init mutex
  xMutex = xSemaphoreCreateMutex();

  // task setup status
  SERIAL_DEBUG.printf("\nTask Setup Status:\n");
  SERIAL_DEBUG.printf("I/O READ TASK SETUP: %s\n", setup.ioActive ? "COMPLETE" : "FAILED");
  SERIAL_DEBUG.printf("DATA READ TASK SETUP: %s\n", (setup.rpiComActive && setup.loraActive) ? "COMPLETE" : "FAILED");
  SERIAL_DEBUG.printf("DATA WRITE TASK SETUP: %s\n", (setup.gpsActive && setup.imuActive) ? "COMPLETE" : "FAILED");

  if (xMutex != NULL)
  {
    // start tasks
    if (setup.ioActive)
    {
      xTaskCreatePinnedToCore(IOReadTask, "IO-Read", TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandleIORead, 0);
    }

    if (setup.i2cActive)
    {
      xTaskCreatePinnedToCore(TractiveReadTask, "Tractive-Read", TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandleTractiveRead, 0);
    }

    if (setup.rpiComActive && setup.loraActive)
    {
      xTaskCreatePinnedToCore(DataWriteTask, "Data-Write", TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandleDataWrite, 1);
    }

    if (setup.gpsActive && setup.imuActive)
    {
      xTaskCreatePinnedToCore(DataReadTask, "Data-Read", TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandleDataRead, 1);
    }

    if (debugger.debugEnabled == true)
    {
      xTaskCreate(DebugTask, "Debugger", TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandleDebug);
    }
  }
  else
  {
    SERIAL_DEBUG.printf("FAILED TO INIT MUTEX!\nHALTING OPERATIONS!");
    while (1)
    {
    }
  }

  // task status
  SERIAL_DEBUG.printf("\nTask Status:\n");
  if (xHandleIORead != NULL)
    SERIAL_DEBUG.printf("I/O READ TASK STATUS: %s\n", TaskStateToString(eTaskGetState(xHandleIORead)).c_str());
  else
    SERIAL_DEBUG.printf("I/O READ TASK STATUS: DISABLED!\n");

  if (xHandleTractiveRead != NULL)
    SERIAL_DEBUG.printf("TRACTIVE READ TASK STATUS: %s\n", TaskStateToString(eTaskGetState(xHandleTractiveRead)).c_str());
  else
    SERIAL_DEBUG.printf("TRACTIVE READ TASK STATUS: DISABLED!\n");

  if (xHandleDataRead != NULL)
    SERIAL_DEBUG.printf("DATA READ TASK STATUS %s\n", TaskStateToString(eTaskGetState(xHandleDataRead)).c_str());
  else
    SERIAL_DEBUG.printf("DATA READ TASK STAUS: DISABLED!\n");

  if (xHandleDataWrite != NULL)
    SERIAL_DEBUG.printf("DATA WRITE TASK STATUS %s\n", TaskStateToString(eTaskGetState(xHandleDataWrite)));
  else
    SERIAL_DEBUG.printf("DATA WRTIE TASK STAUS: DISABLED!\n");

  // scheduler status
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
  {
    SERIAL_DEBUG.printf("\nScheduler Status: RUNNING\n");

    // clock frequency
    rtc_cpu_freq_config_t clock_config;
    rtc_clk_cpu_freq_get_config(&clock_config);
    SERIAL_DEBUG.printf("CPU Frequency: %dMHz\n", clock_config.freq_mhz);
  }
  else
  {
    SERIAL_DEBUG.printf("\nScheduler STATUS: FAILED\nHALTING OPERATIONS");
    while (1)
    {
    }
  }
  SERIAL_DEBUG.printf("\n|--- END SETUP ---|\n\n");
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
void IOReadTask(void *pvParameters)
{
  for (;;)
  {
    if (xSemaphoreTake(xMutex, (TickType_t)10) == pdTRUE)
    {
      //TODO:
      // dampers

      // tire temps

      // strain gauges

      // steering

      // release mutex!
      xSemaphoreGive(xMutex);
    }

    // debugging
    if (debugger.debugEnabled)
    {
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
void DataReadTask(void *pvParameters)
{
  for (;;)
  {
    if (xSemaphoreTake(xMutex, (TickType_t)10) == pdTRUE)
    {
      //TODO:
      // GPS

      // IMU

      // release mutex!
      xSemaphoreGive(xMutex);
    }

    // debugging
    if (debugger.debugEnabled)
    {
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
void DataWriteTask(void *pvParameters)
{
  for (;;)
  {
    if (xSemaphoreTake(xMutex, (TickType_t)10) == pdTRUE)
    {
      // LoRa update
      LoRa.beginPacket();
      LoRa.write((uint8_t *)&telemetryCoreData, sizeof(telemetryCoreData));
      LoRa.endPacket();

      // HUD update
      HUD.write((uint8_t *)&telemetryCoreData, sizeof(telemetryCoreData));

      //TWAI
      //Dash message
      twai_message_t dash;
      dash.identifier = 0xA777;
      dash.extd = 1;
      dash.data_length_code = 3;

      //assigning dashboard data
      uint64_t hexSpeed, hexTemp, hexMode;

      //copying data to make raw hexa data
      memcpy(&hexSpeed, &telemetryCoreData.tractiveCoreData.tractive.currentSpeed, sizeof(float));
      memcpy(&hexTemp, &telemetryCoreData.tractiveCoreData.sensors.coolingTempOut, sizeof(float));
      memcpy(&hexMode, &telemetryCoreData.tractiveCoreData.tractive.driveMode, sizeof(int));

      dash.data[0] = hexSpeed;
      dash.data[1] = hexTemp;
      dash.data[2] = hexMode;


      //Queue message for transmission
      if (twai_transmit(&dash, pdMS_TO_TICKS(1000)) == ESP_OK) {
        SERIAL_DEBUG.printf("Message queued for transmission\n");
      } else {
        SERIAL_DEBUG.printf("Failed to queue message for transmission\n");
      }


      // release mutex!
      xSemaphoreGive(xMutex);
    }

    // debugging
    if (debugger.debugEnabled)
    {
      debugger.serialWriteTaskCount++;
    }

    // limit task refresh rate
    vTaskDelay(SERIAL_WRITE_REFRESH_RATE);
  }
}

/**
 * @brief reads the i2c bus from tractive core
 * @param pvParameters parameters passed to task
 */
void TractiveReadTask(void *pvParameters)
{
  for (;;)
  {
    if (xSemaphoreTake(xMutex, (TickType_t)10) == pdTRUE)
    {
      // inits
      TractiveCoreData tmpData;
      long tmp;

      // read the bus
      while (Wire.available())
      {
        Wire.readBytes((uint8_t *)&tmpData, sizeof(tmpData));

        // copy data
        telemetryCoreData.tractiveCoreData = tmpData;
      }

      // release mutex!
      xSemaphoreGive(xMutex);
    }

    // debugging
    if (debugger.debugEnabled)
    {
      debugger.tractiveReadTaskCount++;
    }

    // limit task refresh rate
    vTaskDelay(TRACTIVE_READ_REFRESH_RATE);
  }
}

/**
 * @brief manages toggle-able debug settings & scheduler debugging
 */
void DebugTask(void *pvParameters)
{
  for (;;)
  {
    // I/O
    if (debugger.IO_debugEnabled)
    {
      PrintIODebug();
    }

    // Scheduler
    if (debugger.scheduler_debugEnable)
    {
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
  vTaskDelay(1); // prevent watchdog from getting upset
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
  if (value == 1)
  {
    mode = SLOW;
  }
  if (value == 2)
  {
    mode = ECO;
  }
  if (value == 3)
  {
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
  if (value == 1)
  {
    state = PRECHARGE_OFF;
  }
  if (value == 2)
  {
    state = PRECHARGE_ON;
  }
  if (value == 3)
  {
    state = PRECHARGE_DONE;
  }
  if (value == 4)
  {
    state = PRECHARGE_ERROR;
  }

  return state;
}

/**
 *
 */
String TaskStateToString(eTaskState state)
{
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
  SERIAL_DEBUG.printf("\n--- START I/O DEBUG ---\n");

  // INPUTS

  // OUTPUTS

  SERIAL_DEBUG.printf("\n--- END I/O DEBUG ---\n");
}

/**
 * @brief some nice in-depth debugging for serial
 */
void PrintSerialDebug()
{
  SERIAL_DEBUG.printf("\n--- START SERIAL DEBUG ---\n");

  // INPUTS

  // OUTPUTS

  SERIAL_DEBUG.printf("\n--- END SERIAL DEBUG ---\n");
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
  if (xHandleIORead != NULL)
  {
    taskStates.push_back(eTaskGetState(xHandleIORead));
  }
  if (xHandleDataRead != NULL)
  {
    taskStates.push_back(eTaskGetState(xHandleDataRead));
  }
  if (xHandleDataWrite != NULL)
  {
    taskStates.push_back(eTaskGetState(xHandleDataWrite));
  }
  if (xHandleTractiveRead != NULL)
  {
    taskStates.push_back(eTaskGetState(xHandleTractiveRead));
  }

  taskRefreshRate.push_back(debugger.ioReadTaskCount - debugger.ioReadTaskPreviousCount);
  taskRefreshRate.push_back(debugger.serialReadTaskCount - debugger.serialReadTaskPreviousCount);
  taskRefreshRate.push_back(debugger.serialWriteTaskCount - debugger.serialWriteTaskPreviousCount);
  taskRefreshRate.push_back(debugger.tractiveReadTaskCount - debugger.tractiveReadTaskPreviousCount);

  // make it usable
  for (int i = 0; i < taskStates.size() - 1; ++i)
  {
    taskStatesStrings.push_back(TaskStateToString(taskStates.at(i)));
  }

  // print it
  SERIAL_DEBUG.printf("uptime: %d | read io:<%d Hz> (%d) | read data:<%d Hz> (%d) | write data:<%d Hz> (%d) | tractive update: <%d Hz> (%d) \r",
                uptime, taskRefreshRate.at(0), debugger.ioReadTaskCount, taskRefreshRate.at(1), debugger.serialReadTaskCount,
                taskRefreshRate.at(2), debugger.serialWriteTaskCount, taskRefreshRate.at(3), debugger.tractiveReadTaskCount);

  // update counters
  debugger.ioReadTaskPreviousCount = debugger.ioReadTaskCount;
  debugger.serialReadTaskPreviousCount = debugger.serialReadTaskCount;
  debugger.serialWriteTaskPreviousCount = debugger.serialWriteTaskCount;
  debugger.tractiveReadTaskPreviousCount = debugger.tractiveReadTaskCount;
}