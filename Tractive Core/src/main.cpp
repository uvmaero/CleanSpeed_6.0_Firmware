/**
 * @file main.cpp
 * @author dom gasperini
 * @brief tractive core
 * @version 1.7.0
 * @date 2024-03-05
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
#include "driver/twai.h"
#include "rtc.h"
#include "rtc_clk_common.h"
#include "vector"

#include <data_types.h>
#include <pin_config.h>

/*
===============================================================================================
                                    Definitions
===============================================================================================
*/

// definitions
#define TRAC_CON_ENABLE_BIAS 0.1     // the activation threshold of traction control expressed as a percentage difference between front and rear wheel speeds
#define TRAC_CON_CORNER_ENABLE 0.7   // percent difference between rear wheel speeds needed to enable cornering traction control
#define TRAC_CON_MAX_MOD 0.75        // maximum power reduction percentage that traction control can apply
#define TRAC_CON_MOD_STEP 0.01       // the step in which each iteration of traction control modified the throttle response
#define TRAC_CON_STEP_INCREASE_MOD 3 // the decrease in traction control management as a mutliple of the increasing step

#define TIRE_DIAMETER 20.0         // diameter of the vehicle's tires in inches
#define WHEEL_RPM_CALC_THRESHOLD 5 // the number of times the hall effect sensor is tripped before calculating vehicle speed

#define BRAKE_LIGHT_THRESHOLD 50 // the threshold that must be crossed for the brake to be considered active

#define PEDAL_MIN 0       // minimum value the pedals can read as
#define PEDAL_MAX 255     // maximum value a pedal can read as
#define PEDAL_DEADBAND 15 // ~5% of PEDAL_MAX
#define PEDAL_DELTA 0.15  // difference between pedal value for cutoff threshold computed by percent difference

#define MAX_TORQUE 225        // MAX TORQUE RINEHART CAN ACCEPT, DO NOT EXCEED 230!!!
#define MAX_REVERSE_TORQUE 25 // for limiting speed while in reverse

#define MIN_BUS_VOLTAGE 150 // min bus voltage

#define COOLING_ENABLE_THRESHOLD 35  // in degrees C
#define COOLING_DISABLE_THRESHOLD 30 // in degrees C

#define PRECHARGE_FLOOR 0.8  // precentage of bus voltage rinehart should be at
#define BUZZER_DURATION 2000 // in milliseconds

#define TELEMETRY_CORE_I2C_ADDR 0x10 // address in hex
#define SERIAL_BAUD_RATE 9600        // baud rate
#define I2C_FREQUENCY 100000         // frequency of bus

// TWAI
#define RINE_MOTOR_INFO_ADDR 0x0A5    // get motor information from Rinehart
#define RINE_VOLT_INFO_ADDR 0x0A7     // get rinehart electrical information
#define RINE_BUS_INFO_ADDR 0x0AA      // get rinehart relay information
#define RINE_MOTOR_CONTROL_ADDR 0x0C0 // motor command address
#define RINE_BUS_CONTROL_ADDR 0x0C1   // control rinehart relay states

#define BMS_GEN_DATA_ADDR 0x6B0  // important BMS data
#define BMS_CELL_DATA_ADDR 0x6B2 // cell data

// tasks
#define IO_WRITE_REFRESH_RATE 9         // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define IO_READ_REFRESH_RATE 2          // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define TWAI_WRITE_REFRESH_RATE 8       // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define TWAI_READ_REFRESH_RATE 8        // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define PRECHARGE_REFRESH_RATE 225      // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define TELEMETRY_UPDATE_REFRESH_RATE 9 // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define DEBUG_REFRESH_RATE 1000         // measured in ticks (RTOS ticks interrupt at 1 kHz)

#define TWAI_BLOCK_DELAY 1 // time to block to complete function call in FreeRTOS ticks

#define TASK_STACK_SIZE 20000 // in bytes

// port aliases
#define SERIAL_DEBUG Serial
#define I2C_CONN Wire

// debug
#define ENABLE_DEBUG true // master debug message control

/*
===============================================================================================
                                  Global Variables
===============================================================================================
*/

/**
 * @brief debugger structure used for organizing debug information
 */
Debugger debugger = {
    // debug toggle
    .debugEnabled = ENABLE_DEBUG,
    .TWAI_debugEnabled = false,
    .IO_debugEnabled = false,
    .scheduler_debugEnable = true,

    // TWAI data
    .TWAI_rinehartCtrlResult = ESP_OK,
    .TWAI_prechargeCtrlResult = ESP_OK,
    .telemetryTractive1MessageResult = ESP_OK,
    .telemetryTractive2MessageResult = ESP_OK,
    .telemetrySensorMessageResult = ESP_OK,
    .telemetryInputsMessageMessageResult = ESP_OK,
    .telemetryOutputsMessageMessageResult = ESP_OK,

    .TWAI_rinehartCtrlMessage = {},
    .TWAI_prechargeCtrlMessage = {},

    // I/O data
    .IO_data = {},

    // precharge data
    .prechargeState = PRECHARGE_OFF,

    // scheduler data
    .ioReadTaskCount = 0,
    .ioWriteTaskCount = 0,
    .twaiReadTaskCount = 0,
    .twaiWriteTaskCount = 0,
    .prechargeTaskCount = 0,
    .telemetryUpdateTaskCount = 0,

    .ioReadTaskPreviousCount = 0,
    .ioWriteTaskPreviousCount = 0,
    .twaiReadTaskPreviousCount = 0,
    .twaiWriteTaskPreviousCount = 0,
    .prechargeTaskPreviousCount = 0,
    .telemetryUpdateTaskPreviousCount = 0,
};

/**
 * @brief the dataframe that describes the entire state of the car
 */
TractiveCoreData tractiveCoreData = {
    // tractive data
    .tractive = {
        .readyToDrive = false,
        .enableInverter = false,

        .prechargeState = PRECHARGE_OFF,

        .rinehartVoltage = 0.0f,
        .commandedTorque = 0,

        .driveDirection = false, // forward = false | reverse = true (we run backwards)
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
        .vicoreEnable = false,

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
    },
};

// Mutex
SemaphoreHandle_t xMutex = NULL;

// Hardware Timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// RTOS Task Handles
TaskHandle_t xHandleIORead = NULL;
TaskHandle_t xHandleIOWrite = NULL;

TaskHandle_t xHandleTWAIRead = NULL;
TaskHandle_t xHandleTWAIWrite = NULL;

TaskHandle_t xHandlePrecharge = NULL;
TaskHandle_t xHandleTelemetryUpdate = NULL;

TaskHandle_t xHandleFrontWheelSpeed = NULL;
TaskHandle_t xHandleRearRightWheelSpeed = NULL;
TaskHandle_t xHandleRearLeftWheelSpeed = NULL;

TaskHandle_t xHandleDebug = NULL;

// TWAI
static const twai_general_config_t can_general_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TWAI_TX_PIN, (gpio_num_t)TWAI_RX_PIN, TWAI_MODE_NORMAL);
static const twai_timing_config_t can_timing_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t can_filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

/*
===============================================================================================
                                    Function Declarations
===============================================================================================
*/

// tasks
void IOReadTask(void *pvParameters);
void IOWriteTask(void *pvParameters);
void TWAIReadTask(void *pvParameters);
void TWAIWriteTask(void *pvParameters);
void PrechargeTask(void *pvParameters);
void TelemetryUpdateTask(void *pvParameters);
void DebugTask(void *pvParameters);

// helpers
void GetCommandedTorque();
uint16_t CalculateThrottleResponse(uint16_t value);
uint16_t TractionControl(uint16_t commandedTorque);

void FrontWheelSpeedCalculator();
void RearRightWheelSpeedCalculator();
void RearLeftWheelSpeedCalculator();

short DriveModeToNumber();
short PrechargeStateToNumber();
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

  // setup management struct
  struct setup
  {
    bool ioActive = false;
    bool twaiActive = false;
    bool i2cActive = false;
  };
  setup setup;

  // -------------------------------------------------------------------------- //

  // ----------------------- initialize serial connection --------------------- //
  SERIAL_DEBUG.begin(SERIAL_BAUD_RATE);
  SERIAL_DEBUG.printf("\n\n|--- STARTING SETUP ---|\n\n");
  // -------------------------------------------------------------------------- //

  // ----------------------- initialize I2C connection --------------------- //
  if (I2C_CONN.begin(I2C_RX_PIN, I2C_TX_PIN, I2C_FREQUENCY) == true)
  {
    I2C_CONN.setBufferSize(255);

    // test for telemetry connection
    I2C_CONN.beginTransmission(TELEMETRY_CORE_I2C_ADDR);
    if (I2C_CONN.endTransmission() == 0)
    {
      SERIAL_DEBUG.printf("TELEMETRY CONNECTION INIT [ SUCCESS ]\n");
      setup.i2cActive = true;
    }
    else
    {
      SERIAL_DEBUG.printf("TELEMETRY CONNECTION INIT [ FAILED ]\n");
    }
  }
  else
  {
    SERIAL_DEBUG.printf("TELEMETRY INIT [ FAILED ]\n");
  }
  // -------------------------------------------------------------------------- //

  // -------------------------- initialize GPIO ------------------------------ //
  analogReadResolution(12);

  // inputs
  pinMode(PEDAL_0_PIN, INPUT);
  pinMode(PEDAL_1_PIN, INPUT);
  pinMode(FRONT_BRAKE_PIN, INPUT);
  pinMode(REAR_BRAKE_PIN, INPUT);

  pinMode(COAST_REGEN_PIN, INPUT);
  pinMode(BRAKE_REGEN_PIN, INPUT);

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(DRIVE_MODE_BUTTON_PIN, INPUT_PULLUP);

  pinMode(IMD_FAULT_PIN, INPUT);
  pinMode(BMS_FAULT_PIN, INPUT);
  pinMode(VICORE_FAULT_PIN, INPUT);

  pinMode(TRACTION_CONTROL_SWITCH_PIN, INPUT);

  pinMode(FR_HALL_EFFECT_PIN, INPUT);
  pinMode(FL_HALL_EFFECT_PIN, INPUT);
  pinMode(BR_HALL_EFFECT_PIN, INPUT);
  pinMode(BL_HALL_EFFECT_PIN, INPUT);

  // outputs
  pinMode(VICORE_ENABLE_PIN, OUTPUT);
  pinMode(RTD_LED_PIN, OUTPUT);
  pinMode(DRIVE_MODE_LED_PIN, OUTPUT);
  pinMode(BMS_FAULT_LED_PIN, OUTPUT);
  pinMode(IMD_FAULT_LED_PIN, OUTPUT);

  pinMode(FAN_ENABLE_PIN, OUTPUT);

  pinMode(BRAKE_LIGHT_PIN, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);

  gpio_set_drive_capability((gpio_num_t)BRAKE_LIGHT_PIN, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability((gpio_num_t)FAN_ENABLE_PIN, GPIO_DRIVE_CAP_3);

  SERIAL_DEBUG.printf("GPIO INIT [ SUCCESS ]\n");
  setup.ioActive = true;
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

      setup.twaiActive = true;
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

  // ------------------------------- Scheduler & Task Status --------------------------------- //
  // init mutex
  xMutex = xSemaphoreCreateMutex();

  // task setup status
  SERIAL_DEBUG.printf("\nTask Setup Status:\n");
  SERIAL_DEBUG.printf("I/O TASK SETUP: %s\n", setup.ioActive ? "COMPLETE" : "FAILED");
  SERIAL_DEBUG.printf("TWAI TASK SETUP: %s\n", setup.twaiActive ? "COMPLETE" : "FAILED");

  // start tasks
  if (xMutex != NULL)
  {
    if (setup.ioActive)
    {
      xTaskCreatePinnedToCore(IOReadTask, "Read-IO", TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandleIORead, 0);
      xTaskCreatePinnedToCore(IOWriteTask, "Write-IO", TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandleIOWrite, 0);
    }

    if (setup.twaiActive)
    {
      xTaskCreatePinnedToCore(TWAIReadTask, "Read-TWAI", TASK_STACK_SIZE, NULL, 1, &xHandleTWAIRead, 1);
      xTaskCreatePinnedToCore(TWAIWriteTask, "Write-TWAI", TASK_STACK_SIZE, NULL, 1, &xHandleTWAIWrite, 1);
    }

    xTaskCreate(PrechargeTask, "Precharge-Update", TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandlePrecharge);

    if (setup.i2cActive)
    {
      xTaskCreate(TelemetryUpdateTask, "Telemetry-Update", TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandleTelemetryUpdate);
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
    SERIAL_DEBUG.printf("I/O READ TASK STATUS: %s\n", TaskStateToString(eTaskGetState(xHandleIORead)));
  else
    SERIAL_DEBUG.printf("I/O READ TASK STATUS: DISABLED!\n");

  if (xHandleIOWrite != NULL)
    SERIAL_DEBUG.printf("I/O WRITE TASK STATUS: %s\n", TaskStateToString(eTaskGetState(xHandleIOWrite)));
  else
    SERIAL_DEBUG.printf("I/O WRITE TASK STATUS: DISABLED!\n");

  if (xHandleTWAIRead != NULL)
    SERIAL_DEBUG.printf("TWAI READ TASK STATUS: %s\n", TaskStateToString(eTaskGetState(xHandleTWAIRead)));
  else
    SERIAL_DEBUG.printf("TWAI READ TASK STATUS: DISABLED!\n");

  if (xHandleTWAIWrite != NULL)
    SERIAL_DEBUG.printf("TWAI WRITE TASK STATUS: %s\n", TaskStateToString(eTaskGetState(xHandleTWAIWrite)));
  else
    SERIAL_DEBUG.printf("TWAI WRITE TASK STATUS: DISABLED!\n");

  if (xHandlePrecharge != NULL)
    SERIAL_DEBUG.printf("PRECHARGE TASK STATUS: %s\n", TaskStateToString(eTaskGetState(xHandlePrecharge)));
  else
    SERIAL_DEBUG.printf("PRECHARGE TASK STATUS: DISABLED!\n");

  if (xHandleTelemetryUpdate != NULL)
    SERIAL_DEBUG.printf("TELEMETRY UPDATE TASK STATUS: %s\n", TaskStateToString(eTaskGetState(xHandleTelemetryUpdate)));
  else
    SERIAL_DEBUG.printf("TELEMETRY UPDATE TASK STAUS: DISABLED!\n");

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
  SERIAL_DEBUG.printf("\n\n|--- END SETUP ---|\n\n");
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
    // check for mutex availability
    if (xSemaphoreTake(xMutex, (TickType_t)10) == pdTRUE)
    {

      // read pedals
      uint16_t tmpPedal1 = analogReadMilliVolts(PEDAL_1_PIN);
      tmpPedal1 = map(tmpPedal1, 290, 1425, PEDAL_MIN, PEDAL_MAX); // (0.29V - 1.379V) | values found via testing
      tractiveCoreData.inputs.pedal1 = CalculateThrottleResponse(tmpPedal1);

      uint16_t tmpPedal0 = analogReadMilliVolts(PEDAL_0_PIN); //  (0.59V - 2.75V) | values found via testing
      tmpPedal0 = map(tmpPedal0, 575, 2810, PEDAL_MIN, PEDAL_MAX);
      tractiveCoreData.inputs.pedal0 = CalculateThrottleResponse(tmpPedal0);

      // calculate commanded torque
      GetCommandedTorque();

      // calculate current speed in mph
      float tmpAverageWheelRpm = (tractiveCoreData.sensors.frontWheelsSpeed + tractiveCoreData.sensors.brWheelSpeed + tractiveCoreData.sensors.blWheelSpeed) / 3;
      tractiveCoreData.tractive.currentSpeed = tmpAverageWheelRpm * TIRE_DIAMETER * 3.14 * (60 / 63360); // 63360 inches in a mile

      // brake pressure / pedal
      float tmpFrontBrake = analogReadMilliVolts(FRONT_BRAKE_PIN);
      tractiveCoreData.inputs.frontBrake = map(tmpFrontBrake, 265, 855, PEDAL_MIN, PEDAL_MAX); // (0.26V - 0.855V) | values found via testing

      float tmpRearBrake = analogReadMilliVolts(REAR_BRAKE_PIN);
      tractiveCoreData.inputs.rearBrake = map(tmpRearBrake, 265, 855, PEDAL_MIN, PEDAL_MAX); // (0.26V - 0.855V) | values found via testing

      uint16_t brakeAverage = (tractiveCoreData.inputs.frontBrake + tractiveCoreData.inputs.rearBrake) / 2;
      if (brakeAverage >= BRAKE_LIGHT_THRESHOLD)
      {
        tractiveCoreData.outputs.brakeLightEnable = true;
      }
      else
      {
        tractiveCoreData.outputs.brakeLightEnable = false;
      }

      // wheel speed
      FrontWheelSpeedCalculator();
      RearRightWheelSpeedCalculator();
      RearLeftWheelSpeedCalculator();

      // traction control
      if (digitalRead(TRACTION_CONTROL_SWITCH_PIN == HIGH))
      {
        tractiveCoreData.tractive.tractionControlEnable = true;
      }
      else
      {
        tractiveCoreData.tractive.tractionControlEnable = false;
      }

      // start button
      if ((digitalRead(START_BUTTON_PIN) == LOW) && tractiveCoreData.tractive.readyToDrive && !tractiveCoreData.tractive.enableInverter)
      {
        // only activate the buzzer is the inverter is not enabled, we don't need to repeat actions
        tractiveCoreData.outputs.buzzerEnable = true;
      }

      // drive mode button
      if (digitalRead(DRIVE_MODE_BUTTON_PIN) == LOW)
      {
        //Cycle the drive mode from Slow -> Eco -> Fast -> Slow -> etc.
        switch (tractiveCoreData.tractive.driveMode)
        {
        case SLOW:
          tractiveCoreData.tractive.driveMode = ECO;
          break;

        case ECO:
          tractiveCoreData.tractive.driveMode = FAST;
          break;

        case FAST:
          tractiveCoreData.tractive.driveMode = SLOW;
          break;

        default:
          tractiveCoreData.tractive.driveMode = ECO;
          break;
        }
      }

      // faults
      if (digitalRead(BMS_FAULT_PIN) == HIGH)
      {
        tractiveCoreData.sensors.bmsFault = false;
      }
      else
      {
        tractiveCoreData.sensors.bmsFault = true;
      }

      if (digitalRead(IMD_FAULT_PIN) == HIGH)
      {
        tractiveCoreData.sensors.imdFault = false;
      }
      else
      {
        tractiveCoreData.sensors.imdFault = true;
      }

      if (digitalRead(VICORE_FAULT_PIN) == LOW)
      {
        tractiveCoreData.sensors.vicoreFault = false;
      }
      else
      {
        tractiveCoreData.sensors.vicoreFault = true;
      }

      // cooling
      int tmpCoolingIn = analogReadMilliVolts(COOLING_IN_TEMP_PIN);
      tractiveCoreData.sensors.coolingTempIn = map(tmpCoolingIn, 0, 2500, 0, 100); // find thermistor values via testing

      int tmpCoolingOut = analogReadMilliVolts(COOLING_OUT_TEMP_PIN);
      tractiveCoreData.sensors.coolingTempOut = map(tmpCoolingOut, 0, 2500, 0, 100); // find thermistor values via testing

      if (tractiveCoreData.sensors.coolingTempIn >= COOLING_ENABLE_THRESHOLD)
      {
        tractiveCoreData.outputs.fansEnable = true;
      }
      if (tractiveCoreData.sensors.coolingTempIn <= COOLING_DISABLE_THRESHOLD)
      {
        tractiveCoreData.outputs.fansEnable = false;
      }

      // debugging
      if (debugger.debugEnabled)
      {
        debugger.IO_data = tractiveCoreData;
        debugger.ioReadTaskCount++;
      }

      // release mutex!
      xSemaphoreGive(xMutex);
    }

    // limit task refresh rate
    vTaskDelay(IO_READ_REFRESH_RATE);
  }
}

/**
 * @brief writes I/O
 * @param pvParameters parameters passed to task
 */
void IOWriteTask(void *pvParameters)
{
  for (;;)
  {
    // check for mutex availability
    if (xSemaphoreTake(xMutex, (TickType_t)10) == pdTRUE)
    {

      // brake light
      if (tractiveCoreData.outputs.brakeLightEnable)
      {
        digitalWrite(BRAKE_LIGHT_PIN, HIGH);
      }
      else
      {
        digitalWrite(BRAKE_LIGHT_PIN, LOW);
      }

      // fans
      if (tractiveCoreData.outputs.fansEnable)
      {
        digitalWrite(FAN_ENABLE_PIN, HIGH);
      }
      else
      {
        digitalWrite(FAN_ENABLE_PIN, LOW);
      }

      // buzzer
      if (tractiveCoreData.outputs.buzzerEnable)
      {
        // buzz buzzer
        digitalWrite(BUZZER_PIN, HIGH);
        vTaskDelay(BUZZER_DURATION);
        digitalWrite(BUZZER_PIN, LOW);

        tractiveCoreData.outputs.buzzerEnable = false;
        tractiveCoreData.tractive.enableInverter = true; // enable the inverter
      }

      // fault leds
      if (tractiveCoreData.sensors.bmsFault)
      {
        digitalWrite(BMS_FAULT_LED_PIN, HIGH);
      }
      else
      {
        digitalWrite(BMS_FAULT_LED_PIN, LOW);
      }

      if (tractiveCoreData.sensors.imdFault)
      {
        digitalWrite(IMD_FAULT_LED_PIN, HIGH);
      }
      else
      {
        digitalWrite(IMD_FAULT_LED_PIN, LOW);
      }

      // drive mode led
      // TODO: implement this doing some rgb led stuff

      // ready to drive LED
      if (tractiveCoreData.tractive.readyToDrive)
      {
        digitalWrite(RTD_LED_PIN, HIGH);
      }
      else
      {
        digitalWrite(RTD_LED_PIN, LOW);
      }

      // debugging
      if (debugger.debugEnabled)
      {
        debugger.IO_data = tractiveCoreData;
        debugger.ioWriteTaskCount++;
      }

      // release mutex!
      xSemaphoreGive(xMutex);
    }

    // limit task refresh rate
    vTaskDelay(IO_WRITE_REFRESH_RATE);
  }
}

/**
 * @brief reads TWAI bus
 * @param pvParameters parameters passed to task
 */
void TWAIReadTask(void *pvParameters)
{
  for (;;)
  {
    // check for mutex availability
    if (xSemaphoreTake(xMutex, (TickType_t)10) == pdTRUE)
    {
      // inits
      twai_message_t incomingMessage;
      uint8_t tmp1, tmp2;

      // if rx queue is full clear it (this is bad, TODO: implement twai message filtering)
      uint32_t alerts;
      twai_read_alerts(&alerts, pdMS_TO_TICKS(TWAI_BLOCK_DELAY));
      if (alerts & TWAI_ALERT_RX_QUEUE_FULL)
      {
        twai_clear_receive_queue();
      }

      // check for new messages in the CAN buffer
      if (twai_receive(&incomingMessage, pdMS_TO_TICKS(TWAI_BLOCK_DELAY)) == ESP_OK)
      { // if there are messages to be read
        int id = incomingMessage.identifier;

        // parse out data
        switch (id)
        {
        // Rinehart: voltage information
        case RINE_VOLT_INFO_ADDR:
          // Rinehart voltage is spread across the first 2 bytes
          tmp1 = incomingMessage.data[0];
          tmp2 = incomingMessage.data[1];

          // combine the first two bytes and assign that to the rinehart voltage
          tractiveCoreData.tractive.rinehartVoltage = (tmp2 << 8) | tmp1; // little endian combination: value = (byte2 << 8) | byte1;
          break;

        // BMS: general pack data
        case BMS_GEN_DATA_ADDR:
          // pack current
          tmp1 = incomingMessage.data[0];
          tmp2 = incomingMessage.data[1];
          tractiveCoreData.orion.packCurrent = (tmp1 << 8) | tmp2; // big endian combination: value = (byte1 << 8) | byte2;

          // pack voltage
          tmp1 = incomingMessage.data[2];
          tmp2 = incomingMessage.data[3];
          tractiveCoreData.orion.busVoltage = ((tmp1 << 8) | tmp2) / 10; // big endian combination: value = (byte1 << 8) | byte2;

          // state of charge
          tractiveCoreData.orion.batteryChargeState = incomingMessage.data[4];
          break;

        // BMS: cell data
        case BMS_CELL_DATA_ADDR:
          tractiveCoreData.orion.minCellVoltage = incomingMessage.data[0];
          tractiveCoreData.orion.maxCellVoltage = incomingMessage.data[1];
          break;

        default:
          // do nothing
          break;
        }
      }

      // debugging
      if (debugger.debugEnabled)
      {
        // scheduler counter update
        debugger.twaiReadTaskCount++;
      }

      // release mutex!
      xSemaphoreGive(xMutex);
    }

    // limit refresh rate
    vTaskDelay(TWAI_READ_REFRESH_RATE);
  }
}

/**
 * @brief writes to TWAI bus
 * @param pvParameters parameters passed to task
 */
void TWAIWriteTask(void *pvParameters)
{
  for (;;)
  {
    // check for mutex availability
    if (xSemaphoreTake(xMutex, (TickType_t)10) == pdTRUE)
    {
      // tractive system control message
      twai_message_t rinehartMessage;
      rinehartMessage.identifier = RINE_MOTOR_CONTROL_ADDR;
      rinehartMessage.flags = TWAI_MSG_FLAG_NONE;
      rinehartMessage.data_length_code = 8;

      // build message
      rinehartMessage.data[0] = tractiveCoreData.tractive.commandedTorque & 0xFF; // commanded torque is sent across two bytes
      rinehartMessage.data[1] = tractiveCoreData.tractive.commandedTorque >> 8;
      rinehartMessage.data[2] = 0x00;                                                // speed command NOT USING
      rinehartMessage.data[3] = 0x00;                                                // speed command NOT USING
      rinehartMessage.data[4] = (uint8_t)(tractiveCoreData.tractive.driveDirection); // 1: forward | 0: reverse (we run in reverse!)
      rinehartMessage.data[5] = (uint8_t)(tractiveCoreData.tractive.enableInverter); // enable inverter command
      rinehartMessage.data[6] = (MAX_TORQUE * 10) & 0xFF;                            // this is the max torque value that rinehart will push
      rinehartMessage.data[7] = (MAX_TORQUE * 10) >> 8;                              // rinehart expects 10x value spread across 2 bytes

      // --- precharge messages --- //
      twai_message_t prechargeCtrlMessage;
      prechargeCtrlMessage.identifier = RINE_BUS_CONTROL_ADDR;
      prechargeCtrlMessage.flags = TWAI_MSG_FLAG_NONE;
      prechargeCtrlMessage.data_length_code = 8;

      // build rinehart message based on precharge state
      switch (tractiveCoreData.tractive.prechargeState)
      {
      case PRECHARGE_OFF:
        // message is sent to rinehart to turn everything off
        prechargeCtrlMessage.data[0] = 0x01; // parameter address. LSB
        prechargeCtrlMessage.data[1] = 0x00; // parameter address. MSB
        prechargeCtrlMessage.data[2] = 0x01; // Read / Write Mode (0 = read | 1 = write)
        prechargeCtrlMessage.data[3] = 0x00; // N/A
        prechargeCtrlMessage.data[4] = 0x00; // Data: ( 0: all off | 1: relay 1 on | 2: relay 2 on | 3: relay 1 & 2 on )
        prechargeCtrlMessage.data[5] = 0x55; // 0x55 means external relay control
        prechargeCtrlMessage.data[6] = 0x00; // N/A
        prechargeCtrlMessage.data[7] = 0x00; // N/A
        break;

      // do precharge
      case PRECHARGE_ON:
        // message is sent to rinehart to turn on precharge relay
        // precharge relay is on relay 1 in Rinehart
        prechargeCtrlMessage.data[0] = 0x01; // parameter address. LSB
        prechargeCtrlMessage.data[1] = 0x00; // parameter address. MSB
        prechargeCtrlMessage.data[2] = 0x01; // Read / Write Mode (0 = read | 1 = write)
        prechargeCtrlMessage.data[3] = 0x00; // N/A
        prechargeCtrlMessage.data[4] = 0x01; // Data: ( 0: all off | 1: relay 1 on | 2: relay 2 on | 3: relay 1 & 2 on )
        prechargeCtrlMessage.data[5] = 0x55; // 0x55 means external relay control
        prechargeCtrlMessage.data[6] = 0x00; // N/A
        prechargeCtrlMessage.data[7] = 0x00; // N/A
        break;

      // precharge complete!
      case PRECHARGE_DONE:
        // message is sent to rinehart to turn everything on
        // Keep precharge relay on and turn on main contactor
        prechargeCtrlMessage.data[0] = 0x01; // parameter address. LSB
        prechargeCtrlMessage.data[1] = 0x00; // parameter address. MSB
        prechargeCtrlMessage.data[2] = 0x01; // Read / Write Mode (0 = read | 1 = write)
        prechargeCtrlMessage.data[3] = 0x00; // N/A
        prechargeCtrlMessage.data[4] = 0x03; // Data: ( 0: all off | 1: relay 1 on | 2: relay 2 on | 3: relay 1 & 2 on )
        prechargeCtrlMessage.data[5] = 0x55; // 0x55 means external relay control
        prechargeCtrlMessage.data[6] = 0x00; // N/A
        prechargeCtrlMessage.data[7] = 0x00; // N/A
        break;

      // error state
      case PRECHARGE_ERROR:
        // message is sent to rinehart to turn everything off
        prechargeCtrlMessage.data[0] = 0x01; // parameter address. LSB
        prechargeCtrlMessage.data[1] = 0x00; // parameter address. MSB
        prechargeCtrlMessage.data[2] = 0x01; // Read / Write Mode (0 = read | 1 = write)
        prechargeCtrlMessage.data[3] = 0x00; // N/A
        prechargeCtrlMessage.data[4] = 0x00; // Data: ( 0: all off | 1: relay 1 on | 2: relay 2 on | 3: relay 1 & 2 on )
        prechargeCtrlMessage.data[5] = 0x55; // 0x55 means external relay control
        prechargeCtrlMessage.data[6] = 0x00; // N/A
        prechargeCtrlMessage.data[7] = 0x00; // N/A
        break;
      }

      // queue all messages for transmission
      esp_err_t rinehartCtrlResult = twai_transmit(&rinehartMessage, pdMS_TO_TICKS(TWAI_BLOCK_DELAY));
      esp_err_t prechargeCtrlMessageResult = twai_transmit(&prechargeCtrlMessage, pdMS_TO_TICKS(TWAI_BLOCK_DELAY));

      // debugging
      if (debugger.debugEnabled)
      {
        debugger.TWAI_rinehartCtrlResult = rinehartCtrlResult;
        // copy message
        for (int i = 0; i < 8; ++i)
        {
          debugger.TWAI_rinehartCtrlMessage[i] = rinehartMessage.data[i];
        }

        debugger.TWAI_prechargeCtrlResult = prechargeCtrlMessageResult;
        // copy message
        for (int i = 0; i < 8; ++i)
        {
          debugger.TWAI_prechargeCtrlMessage[i] = prechargeCtrlMessage.data[i];
        }

        // scheduler counter update
        debugger.twaiWriteTaskCount++;
      }

      // release mutex!
      xSemaphoreGive(xMutex);
    }

    // limit task refresh rate
    vTaskDelay(TWAI_WRITE_REFRESH_RATE);
  }
}

/**
 * @brief run precharge
 * @param pvParameters
 */
void PrechargeTask(void *pvParameters)
{
  for (;;)
  {
    // check for mutex availability
    if (xSemaphoreTake(xMutex, (TickType_t)10) == pdTRUE)
    {
      // precharge state machine
      switch (tractiveCoreData.tractive.prechargeState)
      {

      // prepare for and start precharge
      case PRECHARGE_OFF:

        // set ready to drive state
        tractiveCoreData.tractive.readyToDrive = false;

        // disable vicore
        tractiveCoreData.outputs.vicoreEnable = false;

        if (tractiveCoreData.sensors.imdFault == false && tractiveCoreData.sensors.bmsFault == false)
        {
          tractiveCoreData.tractive.prechargeState = PRECHARGE_ON;
        }

        break;

      // do precharge
      case PRECHARGE_ON:

        // set ready to drive state
        tractiveCoreData.tractive.readyToDrive = false;

        // ensure voltages are above correct values
        if ((tractiveCoreData.tractive.rinehartVoltage >= (tractiveCoreData.orion.busVoltage * PRECHARGE_FLOOR)) &&
            (tractiveCoreData.orion.busVoltage > MIN_BUS_VOLTAGE))
        {
          tractiveCoreData.tractive.prechargeState = PRECHARGE_DONE;
        }

        break;

      // precharge complete!
      case PRECHARGE_DONE:

        // set ready to drive state
        tractiveCoreData.tractive.readyToDrive = true;

        // enable vicore
        tractiveCoreData.outputs.vicoreEnable = true;

        // if rinehart voltage drops below battery, something's wrong,
        if (tractiveCoreData.tractive.rinehartVoltage < MIN_BUS_VOLTAGE)
        {
          tractiveCoreData.tractive.prechargeState = PRECHARGE_ERROR;
        }

        break;

      // error state
      case PRECHARGE_ERROR:

        // ensure car cannot drive
        tractiveCoreData.tractive.enableInverter = false;
        tractiveCoreData.tractive.commandedTorque = 0;

        // reset precharge cycle
        tractiveCoreData.tractive.prechargeState = PRECHARGE_OFF;

        break;

      // handle undefined behavior
      default:

        // if we've entered an undefined state, go to error mode
        tractiveCoreData.tractive.prechargeState = PRECHARGE_ERROR;

        break;
      }

      // debugging
      if (debugger.debugEnabled)
      {
        debugger.prechargeState = tractiveCoreData.tractive.prechargeState;
        debugger.prechargeTaskCount++;
      }

      // release mutex!
      xSemaphoreGive(xMutex);
    }

    // limit task refresh rate
    vTaskDelay(PRECHARGE_REFRESH_RATE);
  }
}

/**
 * @brief writes tractive core data to telemetry core
 */
void TelemetryUpdateTask(void *pvParameters)
{
  for (;;)
  {
    // check for mutex availability
    if (xSemaphoreTake(xMutex, (TickType_t)10) == pdTRUE)
    {
      // write to i2c bus
      I2C_CONN.beginTransmission(TELEMETRY_CORE_I2C_ADDR);
      I2C_CONN.write((uint8_t *)&tractiveCoreData, sizeof(tractiveCoreData));
      I2C_CONN.endTransmission();

      // debugging
      if (debugger.debugEnabled)
      {
        debugger.telemetryUpdateTaskCount++;
      }

      // release mutex!
      xSemaphoreGive(xMutex);
    }

    // limit task refresh rate
    vTaskDelay(TELEMETRY_UPDATE_REFRESH_RATE);
  }
}

/**
 * @brief manages toggle-able debug settings & scheduler debugging
 */
void DebugTask(void *pvParameters)
{
  for (;;)
  {
    // CAN (TWAI)
    if (debugger.TWAI_debugEnabled)
    {
      PrintTWAIDebug();
    }

    // I/O
    if (debugger.IO_debugEnabled)
    {
      PrintIODebug();
    }

    // Scheduler
    if (debugger.scheduler_debugEnable)
    {
      PrintScheduler();
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
 * @brief main loop!
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
 * @brief Get the Commanded Torque from pedal values
 */
void GetCommandedTorque()
{
  // inits
  uint16_t commandedTorque;

  // get the pedal average
  uint16_t pedalAverage = (tractiveCoreData.inputs.pedal0 + tractiveCoreData.inputs.pedal1) / 2;

  // drive mode logic (values are 10x because that is the format for Rinehart)
  switch (tractiveCoreData.tractive.driveMode)
  {
  case SLOW: // runs at 50% power
    commandedTorque = map(pedalAverage, PEDAL_MIN, PEDAL_MAX, 0, (MAX_TORQUE * 10) * 0.50);
    break;

  case ECO: // runs at 75% power
    commandedTorque = map(pedalAverage, PEDAL_MIN, PEDAL_MAX, 0, (MAX_TORQUE * 10) * 0.75);
    break;

  case FAST: // runs at 100% power
    commandedTorque = map(pedalAverage, PEDAL_MIN, PEDAL_MAX, 0, (MAX_TORQUE * 10));
    break;

  // error state, set the mode to ECO
  default:
    // set the state to ECO for next time
    tractiveCoreData.tractive.driveMode = ECO;

    // we don't want to send a torque command if we were in an undefined state
    commandedTorque = 0;
    break;
  }

  // --- traction control --- //
  if (tractiveCoreData.tractive.tractionControlEnable)
  {
    commandedTorque = TractionControl(commandedTorque);
  }

  // --- safety checks --- //

  // while driving in reverse
  if (tractiveCoreData.tractive.driveDirection == true)
  { // forward = false | reverse = true
    if (commandedTorque >= MAX_REVERSE_TORQUE)
    {
      commandedTorque = MAX_REVERSE_TORQUE;
    }
  }

  // rinehart voltage check
  if (tractiveCoreData.tractive.rinehartVoltage < MIN_BUS_VOLTAGE)
  {
    tractiveCoreData.tractive.enableInverter = false;
  }

  // pedal difference
  int pedalDifference = tractiveCoreData.inputs.pedal0 - tractiveCoreData.inputs.pedal1;
  if (_abs(pedalDifference) > (PEDAL_MAX * PEDAL_DELTA))
  {
    commandedTorque = 0;
  }

  // buffer overflow / too much torque somehow
  if ((commandedTorque > (MAX_TORQUE * 10)) || (commandedTorque < 0))
  {
    commandedTorque = 0;
  }

  // if brake is engaged
  if (tractiveCoreData.outputs.brakeLightEnable)
  {
    commandedTorque = 0;
  }

  // check if ready to drive
  if (!tractiveCoreData.tractive.readyToDrive)
  {
    commandedTorque = 0; // if not ready to drive then block all torque
  }

  // --- finalize commanded torque --- //
  tractiveCoreData.tractive.commandedTorque = commandedTorque;
}

/**
 * @brief calculate throttle response of pedal
 * @param value the raw pedal value
 * @return uint16_t the commanded torque value
 */
uint16_t CalculateThrottleResponse(uint16_t value)
{
  // inits
  float calculatedResponse = 0;
  float exponent = 0;

  // check for buffer overflow
  if ((value > PEDAL_MAX) || (value < PEDAL_MIN))
  {
    return 0;
  }

  // account for deadband
  if (value < PEDAL_DEADBAND)
  {
    return 0;
  }

  // determine response curve based on drive mode
  switch (tractiveCoreData.tractive.driveMode)
  {
  case SLOW:
    exponent = 4.0;
    calculatedResponse = (pow(value, exponent)) / (pow(PEDAL_MAX, exponent) / PEDAL_MAX);
    break;

  case ECO:
    exponent = 2.0;
    calculatedResponse = (pow(value, exponent)) / (pow(PEDAL_MAX, exponent) / PEDAL_MAX);
    break;

  case FAST:
    exponent = 0.75;
    calculatedResponse = (pow(value, exponent)) / (pow(PEDAL_MAX, exponent) / PEDAL_MAX);
    break;

  // if we are in an undefined state, pedals should do nothing
  default:
    return 0;
    break;
  }

  // cast final calculated response to an uint16_t
  return (uint16_t)calculatedResponse;
}

/**
 * @brief traction control
 * @param commanded torque
 * @return a traction controlled commanded torque
 */
uint16_t TractionControl(uint16_t commandedTorque)
{
  // inits
  float rearWheelSpeedRatio = 0.0f;
  float averageRearWheelSpeed = 0.0f;
  bool corneringTractionControlEnable;
  bool spinTractionControlEnable;

  // compare rear wheels
  averageRearWheelSpeed = (tractiveCoreData.sensors.brWheelSpeed + tractiveCoreData.sensors.blWheelSpeed) / 2;
  if (tractiveCoreData.sensors.brWheelSpeed > tractiveCoreData.sensors.blWheelSpeed)
  {
    rearWheelSpeedRatio = tractiveCoreData.sensors.blWheelSpeed / tractiveCoreData.sensors.brWheelSpeed;
  }
  else
  {
    rearWheelSpeedRatio = tractiveCoreData.sensors.brWheelSpeed / tractiveCoreData.sensors.blWheelSpeed;
  }

  // cornering scenario
  if (rearWheelSpeedRatio > TRAC_CON_CORNER_ENABLE)
  {
    corneringTractionControlEnable = true;
  }
  else
  {
    corneringTractionControlEnable = false;
  }

  // spinning rear wheels scenario
  if (averageRearWheelSpeed > (tractiveCoreData.sensors.frontWheelsSpeed * (1 + TRAC_CON_ENABLE_BIAS)))
  {
    spinTractionControlEnable = true;
  }
  else
  {
    spinTractionControlEnable = false;
  }

  // apply traction control
  if (corneringTractionControlEnable || spinTractionControlEnable)
  {
    tractiveCoreData.tractive.tractionControlModifier -= TRAC_CON_MOD_STEP;
  }
  else
  {
    tractiveCoreData.tractive.tractionControlModifier += (TRAC_CON_MOD_STEP * TRAC_CON_STEP_INCREASE_MOD);
  }

  // error check result
  if (tractiveCoreData.tractive.tractionControlModifier <= TRAC_CON_MAX_MOD)
  {
    tractiveCoreData.tractive.tractionControlModifier = TRAC_CON_MAX_MOD;
  }

  if (tractiveCoreData.tractive.tractionControlModifier >= 1)
  {
    tractiveCoreData.tractive.tractionControlModifier = 1;
  }

  // calculate an adjusted commanded torque
  return (uint16_t)(commandedTorque * tractiveCoreData.tractive.tractionControlModifier);
}

/**
 * @brief get the speed of the front wheels
 * @param pvParameters parameters passed to task
 */
void FrontWheelSpeedCalculator()
{
  // get time difference
  float timeDiff = (float)esp_timer_get_time() - (float)tractiveCoreData.sensors.frontWheelSpeedTime;

  // calculate rpm
  tractiveCoreData.sensors.frontWheelsSpeed = (timeDiff / 1000000.0) * 60.0;

  // update time keeping
  tractiveCoreData.sensors.frontWheelSpeedTime = esp_timer_get_time();

  return;
}

/**
 * @brief get the speed of the rear right wheels
 * @param pvParameters parameters passed to task
 */
void RearRightWheelSpeedCalculator()
{
  // get time difference
  float timeDiff = (float)esp_timer_get_time() - (float)tractiveCoreData.sensors.brWheelSpeedTime;

  // calculate rpm
  tractiveCoreData.sensors.brWheelSpeed = (timeDiff / 1000000.0) * 60.0;

  // update time keeping
  tractiveCoreData.sensors.brWheelSpeedTime = esp_timer_get_time();

  return;
}

/**
 * @brief get the speed of the rear left wheels
 * @param pvParameters parameters passed to task
 */
void RearLeftWheelSpeedCalculator()
{
  // get time difference
  float timeDiff = (float)esp_timer_get_time() - (float)tractiveCoreData.sensors.blWheelSpeedTime;

  // calculate rpm
  tractiveCoreData.sensors.blWheelSpeed = (timeDiff / 1000000.0) * 60.0;

  // update time keeping
  tractiveCoreData.sensors.blWheelSpeedTime = esp_timer_get_time();

  return;
}

/**
 *
 */
short DriveModeToNumber()
{
  // inits
  short number;

  // convert
  if (tractiveCoreData.tractive.driveMode == SLOW)
  {
    number = 1;
  }
  if (tractiveCoreData.tractive.driveMode == ECO)
  {
    number = 2;
  }
  if (tractiveCoreData.tractive.driveMode == FAST)
  {
    number = 3;
  }

  return number;
}

/**
 *
 */
short PrechargeStateToNumber()
{
  // inits
  short number;

  // convert
  if (tractiveCoreData.tractive.prechargeState == PRECHARGE_OFF)
  {
    number = 1;
  }
  if (tractiveCoreData.tractive.prechargeState == PRECHARGE_ON)
  {
    number = 2;
  }
  if (tractiveCoreData.tractive.prechargeState == PRECHARGE_DONE)
  {
    number = 3;
  }
  if (tractiveCoreData.tractive.prechargeState == PRECHARGE_ERROR)
  {
    number = 4;
  }

  return number;
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
 * @brief some nice in-depth debugging for TWAI
 */
void PrintTWAIDebug()
{
  SERIAL_DEBUG.printf("\n--- START TWAI DEBUG ---\n\n");
  // bus alerts
  SERIAL_DEBUG.printf("TWAI BUS Alerts:\n");
  uint32_t alerts;
  twai_read_alerts(&alerts, pdMS_TO_TICKS(100));
  if (alerts & TWAI_ALERT_TX_SUCCESS)
  {
    SERIAL_DEBUG.printf("TWAI ALERT: TX Success\n");
  }
  if (alerts & TWAI_ALERT_TX_FAILED)
  {
    SERIAL_DEBUG.printf("TWAI ALERT: TX Failed\n");
  }
  if (alerts & TWAI_ALERT_RX_QUEUE_FULL)
  {
    SERIAL_DEBUG.printf("TWAI ALERT: RX Queue Full\n");
  }
  if (alerts & TWAI_ALERT_ABOVE_ERR_WARN)
  {
    SERIAL_DEBUG.printf("TWAI ALERT: Surpassed Error Warning Limit\n");
  }
  if (alerts & TWAI_ALERT_ERR_PASS)
  {
    SERIAL_DEBUG.printf("TWAI ALERT: Entered Error Passive state\n");
  }
  if (alerts & TWAI_ALERT_BUS_OFF)
  {
    SERIAL_DEBUG.printf("TWAI ALERT: Bus Off\n");
  }

  SERIAL_DEBUG.printf("\n");

  // --- incoming messages --- //

  // --- outgoing messages --- //

  // sent status
  SERIAL_DEBUG.printf("Rine Ctrl Send Status: 0x%X\n", debugger.TWAI_rinehartCtrlResult);
  SERIAL_DEBUG.printf("Precharge Ctrl Send Status: 0x%X\n", debugger.TWAI_prechargeCtrlResult);

  // messages
  SERIAL_DEBUG.printf("\n");
  SERIAL_DEBUG.printf("Rinehart Ctrl Outgoing Message:\n");
  for (int i = 0; i < 8; ++i)
  {
    SERIAL_DEBUG.printf("Byte %d: %02X\t", i, debugger.TWAI_rinehartCtrlMessage[i]);
  }

  SERIAL_DEBUG.printf("\n");

  SERIAL_DEBUG.printf("Precharge Ctrl Outgoing Message:\n");
  for (int i = 0; i < 8; ++i)
  {
    SERIAL_DEBUG.printf("Byte %d: %02X\t", i, debugger.TWAI_prechargeCtrlMessage[i]);
  }

  SERIAL_DEBUG.printf("\n\n--- END TWAI DEBUG ---\n");
}

/**
 * @brief some nice in-depth debugging for I/O
 */
void PrintIODebug()
{
  SERIAL_DEBUG.printf("\n--- START I/O DEBUG ---\n");

  // // INPUTS
  // // pedal 0 & 1
  SERIAL_DEBUG.printf("Pedal 0: %d\tPedal 1: %d\n", debugger.IO_data.inputs.pedal0, debugger.IO_data.inputs.pedal1);

  // // brake 0 & 1
  SERIAL_DEBUG.printf("Brake Front: %d\tBrake Rear: %d\n", debugger.IO_data.inputs.frontBrake, debugger.IO_data.inputs.rearBrake);

  // // brake regen
  SERIAL_DEBUG.printf("Brake Regen: %d\n", debugger.IO_data.tractive.brakeRegen);

  // // coast regen
  SERIAL_DEBUG.printf("Coast Regen: %d\n", debugger.IO_data.tractive.coastRegen);

  // // faults
  SERIAL_DEBUG.printf("Faults: IMD: %d | BMS: %d\n", tractiveCoreData.sensors.imdFault, tractiveCoreData.sensors.bmsFault);

  // // rtd
  SERIAL_DEBUG.printf("Ready to Drive: %s\n", tractiveCoreData.tractive.readyToDrive ? "READY" : "DEACTIVATED");

  // // inverter
  SERIAL_DEBUG.printf("Inverter Enable: %s\n", tractiveCoreData.tractive.enableInverter ? "ENABLED" : "DISABLED");

  // // OUTPUTS
  SERIAL_DEBUG.printf("Buzzer Status: %s\n", debugger.IO_data.outputs.buzzerEnable ? "On" : "Off");

  SERIAL_DEBUG.printf("Commanded Torque: %d\n", tractiveCoreData.tractive.commandedTorque);

  SERIAL_DEBUG.printf("Drive Mode: %d\n", (int)tractiveCoreData.tractive.driveMode);

  SERIAL_DEBUG.printf("Brake Light: %d\n", (int)tractiveCoreData.outputs.brakeLightEnable);
  SERIAL_DEBUG.printf("Fan Enable: %d\n", (int)tractiveCoreData.outputs.fansEnable);

  SERIAL_DEBUG.printf("\n--- END I/O DEBUG ---\n");
}

/**
 * @brief scheduler debugging
 */
void PrintScheduler()
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
  if (xHandleIOWrite != NULL)
  {
    taskStates.push_back(eTaskGetState(xHandleIOWrite));
  }
  if (xHandleTWAIRead != NULL)
  {
    taskStates.push_back(eTaskGetState(xHandleTWAIRead));
  }
  if (xHandleTWAIWrite != NULL)
  {
    taskStates.push_back(eTaskGetState(xHandleTWAIWrite));
  }
  if (xHandlePrecharge != NULL)
  {
    taskStates.push_back(eTaskGetState(xHandlePrecharge));
  }
  if (xHandleTelemetryUpdate != NULL)
  {
    taskStates.push_back(eTaskGetState(xHandleTelemetryUpdate));
  }

  taskRefreshRate.push_back(debugger.ioReadTaskCount - debugger.ioReadTaskPreviousCount);
  taskRefreshRate.push_back(debugger.ioWriteTaskCount - debugger.ioWriteTaskPreviousCount);
  taskRefreshRate.push_back(debugger.twaiReadTaskCount - debugger.twaiReadTaskPreviousCount);
  taskRefreshRate.push_back(debugger.twaiWriteTaskCount - debugger.twaiWriteTaskPreviousCount);
  taskRefreshRate.push_back(debugger.prechargeTaskCount - debugger.prechargeTaskPreviousCount);
  taskRefreshRate.push_back(debugger.telemetryUpdateTaskCount - debugger.telemetryUpdateTaskPreviousCount);

  // make it usable
  for (int i = 0; i < taskStates.size() - 1; ++i)
  {
    taskStatesStrings.push_back(TaskStateToString(taskStates.at(i)));
  }

  // print
  // SERIAL_DEBUG.printf("read io:[%s](%d)<%d Hz> | write io:[%s](%d)<%d Hz> | read twai:[%s](%d)<%d Hz> | write twai:[%s](%d)<%d Hz> | precharge:[%s](%d)<%d Hz> \r",
  //   taskStatesStrings.at(0), debugger.ioReadTaskCount, taskRefreshRate.at(0), taskStatesStrings.at(1), debugger.ioWriteTaskCount, taskRefreshRate.at(1), taskStatesStrings.at(2),
  //   debugger.twaiReadTaskCount, taskRefreshRate.at(2), taskStatesStrings.at(3), debugger.twaiWriteTaskCount, taskRefreshRate.at(3), taskStatesStrings.at(4), debugger.prechargeTaskCount, taskRefreshRate.at(4));

  SERIAL_DEBUG.printf("uptime: %d | read io:<%d Hz> (%d) | write io:<%d Hz> (%d) | read twai:<%d Hz> (%d) | write twai:<%d Hz> (%d) | precharge:<%d Hz> (%d) | telemetry update:<%d Hz> (%d) \r",
                uptime, taskRefreshRate.at(0), debugger.ioReadTaskCount, taskRefreshRate.at(1), debugger.ioWriteTaskCount,
                taskRefreshRate.at(2), debugger.twaiReadTaskCount, taskRefreshRate.at(3), debugger.twaiWriteTaskCount, taskRefreshRate.at(4), debugger.prechargeTaskCount,
                taskRefreshRate.at(5), debugger.telemetryUpdateTaskCount);

  // update counters
  debugger.ioReadTaskPreviousCount = debugger.ioReadTaskCount;
  debugger.ioWriteTaskPreviousCount = debugger.ioWriteTaskCount;
  debugger.twaiReadTaskPreviousCount = debugger.twaiReadTaskCount;
  debugger.twaiWriteTaskPreviousCount = debugger.twaiWriteTaskCount;
  debugger.prechargeTaskPreviousCount = debugger.prechargeTaskCount;
  debugger.telemetryUpdateTaskPreviousCount = debugger.telemetryUpdateTaskCount;
}