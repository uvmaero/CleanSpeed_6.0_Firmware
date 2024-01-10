/**
 * @file main.cpp
 * @author dom gasperini
 * @brief tractive core
 * @version 1.1
 * @date 2024-01-10
 * 
 * @ref https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/libraries.html#apis (api and hal docs)
 * @ref https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/_images/ESP32-S3_DevKitC-1_pinlayout.jpg (pinout & overview)
 * @ref https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos_idf.html (FreeRTOS for ESP32 docs)
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
#include "vector"

#include <data_types.h>
#include <pin_config.h>


/*
===============================================================================================
                                    Definitions
===============================================================================================
*/


// definitions
#define TRAC_CON_ENABLE_BIAS            0.1         // the activation threshold of traction control expressed as a percentage difference between front and rear wheel speeds
#define TRAC_CON_CORNER_ENABLE          0.7         // percent difference between rear wheel speeds needed to enable cornering traction control
#define TRAC_CON_MAX_MOD                0.75        // maximum power reduction percentage that traction control can apply
#define TRAC_CON_MOD_STEP               0.02        // the step in which each iteration of traction control modified the throttle response
#define TRAC_CON_STEP_INCREASE_MOD      3           // the decrease in traction control management as a mutliple of the increasing step          
#define TIRE_DIAMETER                   20.0        // diameter of the vehicle's tires in inches
#define WHEEL_RPM_CALC_THRESHOLD        20          // the number of times the hall effect sensor is tripped before calculating vehicle speed
#define BRAKE_LIGHT_THRESHOLD           50          // the threshold that must be crossed for the brake to be considered active
#define PEDAL_MIN                       0           // minimum value the pedals can read as
#define PEDAL_MAX                       255         // maximum value a pedal can read as
#define PEDAL_DEADBAND                  15          // ~5% of PEDAL_MAX
#define MAX_TORQUE                      225         // MAX TORQUE RINEHART CAN ACCEPT, DO NOT EXCEED 230!!!
#define MIN_BUS_VOLTAGE                 150         // min bus voltage
#define COOLING_ENABLE_THRESHOLD        30          // in degrees C 
#define COOLING_DISABLE_THRESHOLD       25          // in degrees C
#define PRECHARGE_FLOOR                 0.8         // precentage of bus voltage rinehart should be at
#define BUZZER_DURATION                 2000        // in milliseconds

// TWAI
#define RINE_MOTOR_INFO_ADDR            0x0A5       // get motor information from Rinehart 
#define RINE_VOLT_INFO_ADDR             0x0A7       // get rinehart electrical information
#define RINE_BUS_INFO_ADDR              0x0AA       // get rinehart relay information 
#define RINE_MOTOR_CONTROL_ADDR         0x0C0       // motor command address 
#define RINE_BUS_CONTROL_ADDR           0x0C1       // control rinehart relay states
#define BMS_GEN_DATA_ADDR               0x6B0       // important BMS data
#define BMS_CELL_DATA_ADDR              0x6B2       // cell data

// tasks & timers
#define TASK_STACK_SIZE                 4096        // in bytes
#define TWAI_BLOCK_DELAY                10          // time to block to complete function call in FreeRTOS ticks (milliseconds)

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

  .ioReadTaskPreviousCount = 0,
  .ioWriteTaskPreviousCount = 0,
  .twaiReadTaskPreviousCount = 0,
  .twaiWriteTaskPreviousCount = 0,
  .prechargeTaskPreviousCount = 0,
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
  },
};


// Hardware Timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


// RTOS Task Handles
TaskHandle_t xHandleIORead = NULL;
TaskHandle_t xHandleIOWrite = NULL;

TaskHandle_t xHandleTWAIRead = NULL;
TaskHandle_t xHandleTWAIWrite = NULL;

TaskHandle_t xHandlePrecharge = NULL;
TaskHandle_t xHandleSerial = NULL;


// TWAI
static const twai_general_config_t can_general_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TWAI_TX_PIN, (gpio_num_t)TWAI_RX_PIN, TWAI_MODE_NORMAL);
static const twai_timing_config_t can_timing_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t can_filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();


/*
===============================================================================================
                                    Function Declarations 
===============================================================================================
*/


// callbacks
void IOUpdateCallback();
void TWAIUpdateCallback();
void PrechargeCallback();
void SerialCallback();

// tasks
void IOReadTask(void* pvParameters);
void IOWriteTask(void* pvParameters);
void TWAIReadTask(void* pvParameters);
void TWAIWriteTask(void* pvParameters);
void PrechargeTask(void* pvParameters);
void SerialTask(void* pvParameters);

// helpers
void GetCommandedTorque();
uint16_t CalculateThrottleResponse(uint16_t value);
uint16_t TractionControl();

// ISRs
void FrontWheelSpeedISR();
void BRWheelSpeedISR();
void BLWheelSpeedISR();


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
    bool twaiActive = false;
  };
  setup setup;
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

  pinMode(TRACTION_CONTROL_SWITCH_PIN, INPUT);

  pinMode(FR_HALL_EFFECT_PIN, INPUT);
  pinMode(FL_HALL_EFFECT_PIN, INPUT);
  pinMode(BR_HALL_EFFECT_PIN, INPUT);
  pinMode(BL_HALL_EFFECT_PIN, INPUT);

  // pin interrupts
  attachInterrupt(FR_HALL_EFFECT_PIN, FrontWheelSpeedISR, RISING);
  attachInterrupt(BR_HALL_EFFECT_PIN, BRWheelSpeedISR, RISING);
  attachInterrupt(BL_HALL_EFFECT_PIN, BLWheelSpeedISR, RISING);

  // outputs
  pinMode(RTD_LED_PIN, OUTPUT);
  pinMode(DRIVE_MODE_LED_PIN, OUTPUT);
  pinMode(BMS_FAULT_LED_PIN, OUTPUT);
  pinMode(IMD_FAULT_LED_PIN, OUTPUT);
  pinMode(FANS_ACTIVE_LED_PIN, OUTPUT);
  pinMode(PUMP_ACTIVE_LED_PIN, OUTPUT);

  pinMode(FAN_ENABLE_PIN, OUTPUT);

  pinMode(BRAKE_LIGHT_PIN, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);

  Serial.printf("GPIO INIT [ SUCCESS ]\n");
  setup.ioActive = true;
  // -------------------------------------------------------------------------- //


  // --------------------- initialize CAN Controller -------------------------- //
  // install CAN driver
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
  // start tasks
  IOUpdateCallback();
  TWAIUpdateCallback();
  PrechargeCallback();
  SerialCallback();

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
 * @param args arguments to be passed to the task
 */
void IOUpdateCallback() 
{
  portENTER_CRITICAL_ISR(&timerMux);

  // inits
  static uint8_t ucParameterToPassRead;
  static uint8_t ucParameterToPassWrite;

  // queue tasks 
  xTaskCreatePinnedToCore(IOReadTask, "Read-IO", TASK_STACK_SIZE, &ucParameterToPassRead, tskIDLE_PRIORITY, &xHandleIORead, 0);     // pinned to core 0
  xTaskCreatePinnedToCore(IOWriteTask, "Write-IO", TASK_STACK_SIZE, &ucParameterToPassWrite, tskIDLE_PRIORITY, &xHandleIOWrite, 0); // pinned to core 0

  // kill task if it returns an error
  if (xHandleIORead != NULL) {
    vTaskDelete(xHandleIORead);
  }
  
  if (xHandleIOWrite != NULL) {
    vTaskDelete(xHandleIOWrite);
  }

  portEXIT_CRITICAL_ISR(&timerMux);

  return;
}


/**
 * @brief callback function for queueing TWAI read and write tasks
 * @param args arguments to be passed to the task
 */
void TWAIUpdateCallback() 
{
  portENTER_CRITICAL_ISR(&timerMux);

  // inits
  static uint8_t ucParameterToPassRead;
  static uint8_t ucParameterToPassWrite;

  // queue tasks 
  xTaskCreatePinnedToCore(TWAIReadTask, "Read-TWAI", TASK_STACK_SIZE, &ucParameterToPassRead, tskIDLE_PRIORITY, &xHandleTWAIRead, 1);     // pinned to core 1
  xTaskCreatePinnedToCore(TWAIWriteTask, "Write-TWAI", TASK_STACK_SIZE, &ucParameterToPassWrite, tskIDLE_PRIORITY, &xHandleTWAIWrite, 1); // pinned to core 1

  // kill task if it returns an error
  if (xHandleTWAIRead != NULL) {
    vTaskDelete(xHandleTWAIRead);
  }
  
  if (xHandleTWAIWrite != NULL) {
    vTaskDelete(xHandleTWAIWrite);
  }

  portEXIT_CRITICAL_ISR(&timerMux);
  
  return;
}


/**
 * @brief callback function to create a new Precharge task 
 */
void PrechargeCallback() 
{
  portENTER_CRITICAL_ISR(&timerMux);
  
  // inits
  static uint8_t ucParameterToPass;

  // queue task
  xTaskCreate(PrechargeTask, "Precharge-Update", TASK_STACK_SIZE, &ucParameterToPass, tskIDLE_PRIORITY, &xHandlePrecharge);

  // kill task if it returns an error
  if (xHandlePrecharge != NULL) {
    vTaskDelete(xHandlePrecharge);
  }

  portEXIT_CRITICAL_ISR(&timerMux);

  return;
}


/**
 * @brief callback function to create a new Serial task
*/
void SerialCallback()
{
  portENTER_CRITICAL_ISR(&timerMux);

  // inits
  static uint8_t ucParameterToPass;

  // queue task
  xTaskCreate(SerialTask, "Serial-Task", TASK_STACK_SIZE, &ucParameterToPass, tskIDLE_PRIORITY, &xHandleSerial);

  // kill task if it returns an error
  if (xHandleSerial != NULL) {
    vTaskDelete(xHandleSerial);
  }

  portEXIT_CRITICAL_ISR(&timerMux);

  return;
}


/*
===============================================================================================
                                        ISRs
===============================================================================================
*/


/**
 * ISR for caluclating front wheel speeds 
*/
void FrontWheelSpeedISR() 
{
  portENTER_CRITICAL_ISR(&timerMux);

  // increment pass counter
  tractiveCoreData.sensors.frontWheelSpeedCount++;

  // calculate wheel rpm
  if (tractiveCoreData.sensors.frontWheelSpeedCount > WHEEL_RPM_CALC_THRESHOLD) {
    // get time difference
    float timeDiff = (float)esp_timer_get_time() - (float)tractiveCoreData.sensors.frontWheelSpeedTime;

    // calculate rpm
    tractiveCoreData.sensors.frontWheelsSpeed = ((float)tractiveCoreData.sensors.frontWheelSpeedCount / (timeDiff / 1000000.0)) * 60.0;

    // update time keeping
    tractiveCoreData.sensors.frontWheelSpeedTime = esp_timer_get_time();

    // reset counter
    tractiveCoreData.sensors.frontWheelSpeedCount = 0;
  }

  portEXIT_CRITICAL_ISR(&timerMux);

  return;
}


/**
 * ISR for caluclating rear right wheel speeds 
*/
void BRWheelSpeedISR() 
{
  portENTER_CRITICAL_ISR(&timerMux);

  // increment pass counter
  tractiveCoreData.sensors.brWheelSpeedCount++;

  // calculate wheel rpm
  if (tractiveCoreData.sensors.brWheelSpeedCount > WHEEL_RPM_CALC_THRESHOLD) {
    // get time difference
    float timeDiff = (float)esp_timer_get_time() - (float)tractiveCoreData.sensors.brWheelSpeedTime;

    // calculate rpm
    tractiveCoreData.sensors.brWheelSpeed = ((float)tractiveCoreData.sensors.brWheelSpeedCount / (timeDiff / 1000000.0)) * 60.0;

    // update time keeping
    tractiveCoreData.sensors.brWheelSpeedTime = esp_timer_get_time();

    // reset counter
    tractiveCoreData.sensors.brWheelSpeedCount = 0;
  }

  portEXIT_CRITICAL_ISR(&timerMux);

  return;
}


/**
 * ISR for caluclating rear left wheel speeds 
*/
void BLWheelSpeedISR() 
{
  portENTER_CRITICAL_ISR(&timerMux);

  // increment pass counter
  tractiveCoreData.sensors.blWheelSpeedCount++;

  // calculate wheel rpm
  if (tractiveCoreData.sensors.blWheelSpeedCount > WHEEL_RPM_CALC_THRESHOLD) {
    // get time difference
    float timeDiff = (float)esp_timer_get_time() - (float)tractiveCoreData.sensors.blWheelSpeedTime;

    // calculate rpm
    tractiveCoreData.sensors.blWheelSpeed = ((float)tractiveCoreData.sensors.blWheelSpeedCount / (timeDiff / 1000000.0)) * 60.0;

    // update time keeping
    tractiveCoreData.sensors.blWheelSpeedTime = esp_timer_get_time();

    // reset counter
    tractiveCoreData.sensors.blWheelSpeedCount = 0;
  }

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
    // read pedals
    uint16_t tmpPedal1 = analogReadMilliVolts(PEDAL_1_PIN);
    tmpPedal1 = map(tmpPedal1, 290, 1425, PEDAL_MIN, PEDAL_MAX);                // (0.29V - 1.379V) | values found via testing
    tractiveCoreData.inputs.pedal1 = CalculateThrottleResponse(tmpPedal1);

    uint16_t tmpPedal0 = analogReadMilliVolts(PEDAL_0_PIN);                     //  (0.59V - 2.75V) | values found via testing
    tmpPedal0 = map(tmpPedal0, 575, 2810, PEDAL_MIN, PEDAL_MAX);
    tractiveCoreData.inputs.pedal0 = CalculateThrottleResponse(tmpPedal0);

    // calculate commanded torque
    GetCommandedTorque();

    // calculate current speed in mph
    float tmpAverageWheelRpm = (tractiveCoreData.sensors.frontWheelsSpeed + tractiveCoreData.sensors.brWheelSpeed + tractiveCoreData.sensors.blWheelSpeed) / 3;
    tractiveCoreData.tractive.currentSpeed = tmpAverageWheelRpm * TIRE_DIAMETER * 3.14 * (60 / 63360);   // 63360 inches in a mile

    // brake pressure / pedal
    float tmpFrontBrake = analogReadMilliVolts(FRONT_BRAKE_PIN);
    tractiveCoreData.inputs.frontBrake = map(tmpFrontBrake, 265, 855, PEDAL_MIN, PEDAL_MAX);  // (0.26V - 0.855V) | values found via testing

    float tmpRearBrake = analogReadMilliVolts(REAR_BRAKE_PIN);
    tractiveCoreData.inputs.rearBrake = map(tmpRearBrake, 265, 855, PEDAL_MIN, PEDAL_MAX);    // (0.26V - 0.855V) | values found via testing

    uint16_t brakeAverage = (tractiveCoreData.inputs.frontBrake + tractiveCoreData.inputs.rearBrake) / 2;
    if (brakeAverage >= BRAKE_LIGHT_THRESHOLD) {
      tractiveCoreData.outputs.brakeLightEnable = true;
    }
    else {
      tractiveCoreData.outputs.brakeLightEnable = false;
    }

    // traction control
    if (digitalRead(TRACTION_CONTROL_SWITCH_PIN == HIGH)) {
      tractiveCoreData.tractive.tractionControlEnable = true;
    }
    else {
      tractiveCoreData.tractive.tractionControlEnable = false;
    }

    // start button 
    if ((digitalRead(START_BUTTON_PIN) == LOW) && tractiveCoreData.tractive.readyToDrive && !tractiveCoreData.tractive.enableInverter) {
      // only activate the buzzer is the inverter is not enabled, we don't need to repeat actions
      tractiveCoreData.outputs.buzzerEnable = true;
    }

    // drive mode button
    if (digitalRead(DRIVE_MODE_BUTTON_PIN) == LOW) {
      switch (tractiveCoreData.tractive.driveMode) {
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
    if (digitalRead(BMS_FAULT_PIN) == LOW) {
      tractiveCoreData.sensors.bmsFault = false;
    }
    else {
      tractiveCoreData.sensors.bmsFault = true;
    }

    if (digitalRead(IMD_FAULT_PIN) == LOW) {
      tractiveCoreData.sensors.imdFault = false;
    }
    else {
      tractiveCoreData.sensors.imdFault = true;
    }

    // cooling 
    int tmpCoolingIn = analogReadMilliVolts(COOLING_IN_TEMP_PIN);
    tractiveCoreData.sensors.coolingTempIn = map(tmpCoolingIn, 0, 2500, 0, 100);      // find thermistor values via testing 

    int tmpCoolingOut = analogReadMilliVolts(COOLING_OUT_TEMP_PIN);
    tractiveCoreData.sensors.coolingTempOut = map(tmpCoolingOut, 0, 2500, 0, 100);    // find thermistor values via testing 

    if (tractiveCoreData.sensors.coolingTempIn >= COOLING_ENABLE_THRESHOLD) {
      tractiveCoreData.outputs.fansEnable = true;
    }
    if (tractiveCoreData.sensors.coolingTempIn <= COOLING_DISABLE_THRESHOLD) {
      tractiveCoreData.outputs.fansEnable = false;
    }

    // debugging
    if (debugger.debugEnabled) {
      debugger.IO_data = tractiveCoreData;
      debugger.ioReadTaskCount++;
    }
  }
}


/**
 * @brief writes I/O
 * @param pvParameters parameters passed to task
 */
void IOWriteTask(void* pvParameters)
{
  for (;;) 
  {
    // brake light 
    if (tractiveCoreData.outputs.brakeLightEnable) {
      digitalWrite(BRAKE_LIGHT_PIN, HIGH);
    }
    else {
      digitalWrite(BRAKE_LIGHT_PIN, LOW);
    }

    // cooling
    if (tractiveCoreData.outputs.fansEnable) {
      digitalWrite(FAN_ENABLE_PIN, HIGH);
    }
    else {
      digitalWrite(FAN_ENABLE_PIN, LOW);
    }

    // buzzer
    if (tractiveCoreData.outputs.buzzerEnable) {
      // buzz buzzer
      digitalWrite(BUZZER_PIN, HIGH);
      vTaskDelay(BUZZER_DURATION);
      digitalWrite(BUZZER_PIN, LOW);

      tractiveCoreData.outputs.buzzerEnable = false;
      tractiveCoreData.tractive.enableInverter = true;      // enable the inverter
    }

    // fault leds
    if (tractiveCoreData.sensors.bmsFault) {
      digitalWrite(BMS_FAULT_LED_PIN, HIGH);
    }
    else {
      digitalWrite(BMS_FAULT_LED_PIN, LOW);
    }

    if (tractiveCoreData.sensors.imdFault) {
      digitalWrite(IMD_FAULT_LED_PIN, HIGH);
    }
    else {
      digitalWrite(IMD_FAULT_LED_PIN, LOW);
    }

    // drive mode led
    // TODO: implement this doing some rgb led stuff

    // cooling led
    if (tractiveCoreData.outputs.fansEnable) {
      digitalWrite(FANS_ACTIVE_LED_PIN, HIGH);
    }
    else {
      digitalWrite(FANS_ACTIVE_LED_PIN, LOW);
    }

    // ready to drive LED
    if (tractiveCoreData.tractive.readyToDrive) {
      digitalWrite(RTD_LED_PIN, HIGH);
    }
    else {
      digitalWrite(RTD_LED_PIN, LOW);
    }

    // debugging
    if (debugger.debugEnabled) {
      debugger.IO_data = tractiveCoreData;
      debugger.ioWriteTaskCount++;
    }
  }
}


/**
 * @brief reads TWAI bus
 * @param pvParameters parameters passed to task
 */
void TWAIReadTask(void* pvParameters)
{
  for (;;) {
    // inits
    twai_message_t incomingMessage;
    uint8_t tmp1, tmp2;
    int id;

    // if rx queue is full clear it (this is bad, implement twai message filtering)
    uint32_t alerts;
    twai_read_alerts(&alerts, pdMS_TO_TICKS(TWAI_BLOCK_DELAY));
    if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
      twai_clear_receive_queue();
    }

    // check for new messages in the CAN buffer
    if (twai_receive(&incomingMessage, pdMS_TO_TICKS(TWAI_BLOCK_DELAY)) == ESP_OK) { // if there are messages to be read
      id = incomingMessage.identifier;
      
      // parse out data
      switch (id) {
        // Rinehart: voltage information
        case RINE_VOLT_INFO_ADDR:
          // rinehart voltage is spread across the first 2 bytes
          tmp1 = incomingMessage.data[0];
          tmp2 = incomingMessage.data[1];

          // combine the first two bytes and assign that to the rinehart voltage
          tractiveCoreData.tractive.rinehartVoltage = (tmp2 << 8) | tmp1;   // little endian combination: value = (byte2 << 8) | byte1;
        break;

        // BMS: general pack data
        case BMS_GEN_DATA_ADDR:
          // pack current
          tmp1 = incomingMessage.data[0]; 
          tmp2 = incomingMessage.data[1];
          tractiveCoreData.orion.packCurrent = (tmp1 << 8) | tmp2;   // big endian combination: value = (byte1 << 8) | byte2;

          // pack voltage
          tmp1 = incomingMessage.data[2];
          tmp2 = incomingMessage.data[3];
          tractiveCoreData.orion.busVoltage = ((tmp1 << 8) | tmp2) / 10;    // big endian combination: value = (byte1 << 8) | byte2;

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
  }
}


/**
 * @brief writes to TWAI bus
 * @param pvParameters parameters passed to task
 */
void TWAIWriteTask(void* pvParameters)
{
  for (;;)
  {
    // inits
    bool sentStatus = false;

    // tractive system control message
    twai_message_t rinehartMessage;
    rinehartMessage.identifier = RINE_MOTOR_CONTROL_ADDR;
    rinehartMessage.flags = TWAI_MSG_FLAG_NONE;
    rinehartMessage.data_length_code = 8;

    // build message
    rinehartMessage.data[0] = tractiveCoreData.tractive.commandedTorque & 0xFF;     // commanded torque is sent across two bytes
    rinehartMessage.data[1] = tractiveCoreData.tractive.commandedTorque >> 8;
    rinehartMessage.data[2] = 0x00;                                                 // speed command NOT USING
    rinehartMessage.data[3] = 0x00;                                                 // speed command NOT USING
    rinehartMessage.data[4] = (uint8_t)(tractiveCoreData.tractive.driveDirection);  // 1: forward | 0: reverse (we run in reverse!)
    rinehartMessage.data[5] = (uint8_t)(tractiveCoreData.tractive.enableInverter);  // enable inverter command
    rinehartMessage.data[6] = (MAX_TORQUE * 10) & 0xFF;                             // this is the max torque value that rinehart will push
    rinehartMessage.data[7] = (MAX_TORQUE * 10) >> 8;                               // rinehart expects 10x value spread across 2 bytes

    // queue message for transmission
    esp_err_t rinehartCtrlResult = twai_transmit(&rinehartMessage, pdMS_TO_TICKS(TWAI_BLOCK_DELAY));


    // --- precharge messages --- // 
    twai_message_t prechargeCtrlMessage;
    prechargeCtrlMessage.identifier = RINE_BUS_CONTROL_ADDR;
    prechargeCtrlMessage.flags = TWAI_MSG_FLAG_NONE;
    prechargeCtrlMessage.data_length_code = 8;

    // build rinehart message based on precharge state
    switch (tractiveCoreData.tractive.prechargeState) {
      case PRECHARGE_OFF:
        // message is sent to rinehart to turn everything off
        prechargeCtrlMessage.data[0] = 0x01;          // parameter address. LSB
        prechargeCtrlMessage.data[1] = 0x00;          // parameter address. MSB
        prechargeCtrlMessage.data[2] = 0x01;          // Read / Write Mode (0 = read | 1 = write)
        prechargeCtrlMessage.data[3] = 0x00;          // N/A
        prechargeCtrlMessage.data[4] = 0x00;          // Data: ( 0: all off | 1: relay 1 on | 2: relay 2 on | 3: relay 1 & 2 on )
        prechargeCtrlMessage.data[5] = 0x55;          // 0x55 means external relay control
        prechargeCtrlMessage.data[6] = 0x00;          // N/A
        prechargeCtrlMessage.data[7] = 0x00;          // N/A
      break;

      // do precharge
      case PRECHARGE_ON:
        // message is sent to rinehart to turn on precharge relay
        // precharge relay is on relay 1 in Rinehart
        prechargeCtrlMessage.data[0] = 0x01;          // parameter address. LSB
        prechargeCtrlMessage.data[1] = 0x00;          // parameter address. MSB
        prechargeCtrlMessage.data[2] = 0x01;          // Read / Write Mode (0 = read | 1 = write)
        prechargeCtrlMessage.data[3] = 0x00;          // N/A
        prechargeCtrlMessage.data[4] = 0x01;          // Data: ( 0: all off | 1: relay 1 on | 2: relay 2 on | 3: relay 1 & 2 on )
        prechargeCtrlMessage.data[5] = 0x55;          // 0x55 means external relay control
        prechargeCtrlMessage.data[6] = 0x00;          // N/A
        prechargeCtrlMessage.data[7] = 0x00;          // N/A
      break;


      // precharge complete!
      case PRECHARGE_DONE:
        // message is sent to rinehart to turn everything on
        // Keep precharge relay on and turn on main contactor
        prechargeCtrlMessage.data[0] = 0x01;          // parameter address. LSB
        prechargeCtrlMessage.data[1] = 0x00;          // parameter address. MSB
        prechargeCtrlMessage.data[2] = 0x01;          // Read / Write Mode (0 = read | 1 = write)
        prechargeCtrlMessage.data[3] = 0x00;          // N/A
        prechargeCtrlMessage.data[4] = 0x03;          // Data: ( 0: all off | 1: relay 1 on | 2: relay 2 on | 3: relay 1 & 2 on )
        prechargeCtrlMessage.data[5] = 0x55;          // 0x55 means external relay control
        prechargeCtrlMessage.data[6] = 0x00;          // N/A
        prechargeCtrlMessage.data[7] = 0x00;          // N/A
      break;


      // error state
      case PRECHARGE_ERROR:
        // message is sent to rinehart to turn everything off
        prechargeCtrlMessage.data[0] = 0x01;          // parameter address. LSB
        prechargeCtrlMessage.data[1] = 0x00;          // parameter address. MSB
        prechargeCtrlMessage.data[2] = 0x01;          // Read / Write Mode (0 = read | 1 = write)
        prechargeCtrlMessage.data[3] = 0x00;          // N/A
        prechargeCtrlMessage.data[4] = 0x00;          // Data: ( 0: all off | 1: relay 1 on | 2: relay 2 on | 3: relay 1 & 2 on )
        prechargeCtrlMessage.data[5] = 0x55;          // 0x55 means external relay control
        prechargeCtrlMessage.data[6] = 0x00;          // N/A
        prechargeCtrlMessage.data[7] = 0x00;          // N/A
      break;
    }

    // queue rinehart message for transmission
    esp_err_t prechargeCtrlMessageResult = twai_transmit(&prechargeCtrlMessage, pdMS_TO_TICKS(TWAI_BLOCK_DELAY));

    // debugging
    if (debugger.debugEnabled) {
      debugger.TWAI_rinehartCtrlResult = rinehartCtrlResult;
      // copy message
      for (int i = 0; i < 8; ++i) {
        debugger.TWAI_rinehartCtrlMessage[i] = rinehartMessage.data[i];
      }

      debugger.TWAI_prechargeCtrlResult = prechargeCtrlMessageResult;
      // copy message 
      for (int i = 0; i < 8; ++i) {
        debugger.TWAI_prechargeCtrlMessage[i] = prechargeCtrlMessage.data[i];
      }

      // scheduler counter update
      debugger.twaiWriteTaskCount++;
    }
  }
}


/**
 * @brief run precharge
 * @param pvParameters 
 */
void PrechargeTask(void* pvParameters)
{
  for (;;)
  {
    // inits
    twai_message_t outgoingMessage;
    int result;

    // precharge state machine
    switch (tractiveCoreData.tractive.prechargeState) {

      // prepare for and start precharge
      case PRECHARGE_OFF:

        // set ready to drive state
        tractiveCoreData.tractive.readyToDrive = false;

        if (tractiveCoreData.sensors.imdFault == false && tractiveCoreData.sensors.bmsFault == false) {
          tractiveCoreData.tractive.prechargeState = PRECHARGE_ON;
        }

      break;

      // do precharge
      case PRECHARGE_ON:

        // set ready to drive state
        tractiveCoreData.tractive.readyToDrive = false;

        // ensure voltages are above correct values
        if ((tractiveCoreData.tractive.rinehartVoltage >= (tractiveCoreData.orion.busVoltage * PRECHARGE_FLOOR)) &&
        (tractiveCoreData.orion.busVoltage > MIN_BUS_VOLTAGE)) {
          tractiveCoreData.tractive.prechargeState = PRECHARGE_DONE;
        }

      break;


      // precharge complete!
      case PRECHARGE_DONE:

        // set ready to drive state
        tractiveCoreData.tractive.readyToDrive = true;

        // if rinehart voltage drops below battery, something's wrong, 
        if (tractiveCoreData.tractive.rinehartVoltage < MIN_BUS_VOLTAGE) {
          tractiveCoreData.tractive.prechargeState = PRECHARGE_ERROR;
        }

      break;


      // error state
      case PRECHARGE_ERROR:

        // ensure car cannot drive
        tractiveCoreData.tractive.readyToDrive = false;
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
    if (debugger.debugEnabled) {
      debugger.prechargeState = tractiveCoreData.tractive.prechargeState;
      debugger.prechargeTaskCount++;
    }
  }
}


/**
 * send messages to telemetry core 
*/
void SerialTask(void* pvParameters) 
{
  for (;;) 
  {
    // write tractive core data to serial bus
    Serial.write((uint8_t *) &tractiveCoreData, sizeof(tractiveCoreData));
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
  vTaskDelay(MAIN_LOOP_DELAY);    // prevent watchdog from getting upset

  // debugging
  if (debugger.debugEnabled) {
    PrintDebug();
  }
}


/**
 * @brief Get the Commanded Torque from pedal values
 */
void GetCommandedTorque()
{
  // get the pedal average
  int pedalAverage = (tractiveCoreData.inputs.pedal0 + tractiveCoreData.inputs.pedal1) / 2;

  // drive mode logic (values are 10x because that is the format for Rinehart)
  switch (tractiveCoreData.tractive.driveMode)
  {
    case SLOW:  // runs at 50% power
      tractiveCoreData.tractive.commandedTorque = map(pedalAverage, PEDAL_MIN, PEDAL_MAX, 0, (MAX_TORQUE * 10) * 0.50);
    break;

    case ECO:   // runs at 75% power
      tractiveCoreData.tractive.commandedTorque = map(pedalAverage, PEDAL_MIN, PEDAL_MAX, 0, (MAX_TORQUE * 10) * 0.75);
    break;

    case FAST:  // runs at 100% power
      tractiveCoreData.tractive.commandedTorque = map(pedalAverage, PEDAL_MIN, PEDAL_MAX, 0, (MAX_TORQUE * 10));
    break;
    
    // error state, set the mode to ECO
    default:
      // set the state to ECO for next time
      tractiveCoreData.tractive.driveMode = ECO;

      // we don't want to send a torque command if we were in an undefined state
      tractiveCoreData.tractive.commandedTorque = 0;
    break;
  }


  // --- traction control --- //
  if (tractiveCoreData.tractive.tractionControlEnable) {
    tractiveCoreData.tractive.commandedTorque = TractionControl();
  }


  // --- safety checks --- //

  // rinehart voltage check
  if (tractiveCoreData.tractive.rinehartVoltage < MIN_BUS_VOLTAGE) {
    tractiveCoreData.tractive.enableInverter = false;
  }

  // pedal difference 
  int pedalDifference = tractiveCoreData.inputs.pedal0 - tractiveCoreData.inputs.pedal1;
  if (_abs(pedalDifference) > (PEDAL_MAX * 0.15)) {
    tractiveCoreData.tractive.commandedTorque = 0;
  }
  
  // buffer overflow / too much torque somehow
  if ((tractiveCoreData.tractive.commandedTorque > (MAX_TORQUE * 10)) || (tractiveCoreData.tractive.commandedTorque < 0)) {
    tractiveCoreData.tractive.commandedTorque = 0;
  }

  // if brake is engaged
  if (tractiveCoreData.outputs.brakeLightEnable) {
    tractiveCoreData.tractive.commandedTorque = 0;
  }

  // check if ready to drive
  if (!tractiveCoreData.tractive.readyToDrive) {
    tractiveCoreData.tractive.commandedTorque = 0;      // if not ready to drive then block all torque
  }
} 


/**
 * @brief calculate throttle response of pedal
 * 
 * @param value the raw pedal value
 * @return uint16_t the commanded torque value
 */
uint16_t CalculateThrottleResponse(uint16_t value) 
{
  // inits
  float calculatedResponse = 0;
  float exponent = 0;

  // check for buffer overflow
  if ((value > PEDAL_MAX) || (value < PEDAL_MIN)) {
    return 0;
  }

  // account for deadband
  if (value < PEDAL_DEADBAND) {
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
*/
uint16_t TractionControl() {
  // inits
  float rearWheelSpeedRatio = 0.0f;
  float averageRearWheelSpeed = 0.0f;
  bool corneringTractionControlEnable;
  bool spinTractionControlEnable;

  // compare rear wheels
  averageRearWheelSpeed = (tractiveCoreData.sensors.brWheelSpeed + tractiveCoreData.sensors.blWheelSpeed) / 2;
  if (tractiveCoreData.sensors.brWheelSpeed > tractiveCoreData.sensors.blWheelSpeed) {
    rearWheelSpeedRatio = tractiveCoreData.sensors.blWheelSpeed / tractiveCoreData.sensors.brWheelSpeed;
  }
  else{
    rearWheelSpeedRatio = tractiveCoreData.sensors.brWheelSpeed / tractiveCoreData.sensors.blWheelSpeed;
  }

  // cornering scenario
  if (rearWheelSpeedRatio > TRAC_CON_CORNER_ENABLE) {
    corneringTractionControlEnable = true;
  }
  else {
    corneringTractionControlEnable = false;
  }

  // spinning rear wheels scenario
  if (averageRearWheelSpeed > (tractiveCoreData.sensors.frontWheelsSpeed * (1 - TRAC_CON_ENABLE_BIAS))) {
    spinTractionControlEnable = true;
  }
  else {
    spinTractionControlEnable = false;
  }

  // apply traction control
  if (corneringTractionControlEnable || spinTractionControlEnable) {
    tractiveCoreData.tractive.tractionControlModifier -= TRAC_CON_MOD_STEP;
  }
  else {
    tractiveCoreData.tractive.tractionControlModifier += (TRAC_CON_MOD_STEP * TRAC_CON_STEP_INCREASE_MOD);
  }

  // error check result
  if (tractiveCoreData.tractive.tractionControlModifier <= TRAC_CON_MAX_MOD) {
    tractiveCoreData.tractive.tractionControlModifier = TRAC_CON_MAX_MOD;
  }

  if (tractiveCoreData.tractive.tractionControlModifier >= 1) {
    tractiveCoreData.tractive.tractionControlModifier = 1;
  }

  // calculate an adjusted commanded torque
  return (uint16_t)(tractiveCoreData.tractive.commandedTorque * tractiveCoreData.tractive.tractionControlModifier);
}


/* 
===============================================================================================
                                    DEBUG FUNCTIONS
================================================================================================
*/


/**
 * @brief some nice in-depth debugging for TWAI
 */
void PrintTWAIDebug() {
  Serial.printf("\n--- START TWAI DEBUG ---\n\n");
  // bus alerts            
  Serial.printf("TWAI BUS Alerts:\n");
  uint32_t alerts;
  twai_read_alerts(&alerts, pdMS_TO_TICKS(100));
  if (alerts & TWAI_ALERT_TX_SUCCESS) {
    Serial.printf("TWAI ALERT: TX Success\n");
  }
  if (alerts & TWAI_ALERT_TX_FAILED) {
    Serial.printf("TWAI ALERT: TX Failed\n");
  }
  if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
    Serial.printf("TWAI ALERT: RX Queue Full\n");
  }
  if (alerts & TWAI_ALERT_ABOVE_ERR_WARN) {
    Serial.printf("TWAI ALERT: Surpassed Error Warning Limit\n");
  }
  if (alerts & TWAI_ALERT_ERR_PASS) {
    Serial.printf("TWAI ALERT: Entered Error Passive state\n");
  }
  if (alerts & TWAI_ALERT_BUS_OFF) {
    Serial.printf("TWAI ALERT: Bus Off\n");
  }

  Serial.printf("\n");

  // --- incoming messages --- // 


  // --- outgoing messages --- // 

  // sent status
  Serial.printf("Rine Ctrl Send Status: 0x%X\n", debugger.TWAI_rinehartCtrlResult);
  Serial.printf("Precharge Ctrl Send Status: 0x%X\n", debugger.TWAI_prechargeCtrlResult);

  // messages
  Serial.printf("\n");
  Serial.printf("Rinehart Ctrl Outgoing Message:\n");
  for (int i = 0; i < 8; ++i) {
    Serial.printf("Byte %d: %02X\t", i, debugger.TWAI_rinehartCtrlMessage[i]);
  }

  Serial.printf("\n");

  Serial.printf("Precharge Ctrl Outgoing Message:\n");
  for (int i = 0; i < 8; ++i) {
    Serial.printf("Byte %d: %02X\t", i, debugger.TWAI_prechargeCtrlMessage[i]);
  }

  Serial.printf("\n\n--- END TWAI DEBUG ---\n");
}


/**
 * @brief some nice in-depth debugging for I/O
 */
void PrintIODebug() {
  Serial.printf("\n--- START I/O DEBUG ---\n");

  // // INPUTS
  // // pedal 0 & 1
  // Serial.printf("Pedal 0: %d\tPedal 1: %d\n", debugger.IO_data.inputs.pedal0, debugger.IO_data.inputs.pedal1);	

  // // brake 0 & 1
  // Serial.printf("Brake Front: %d\tBrake Rear: %d\n", debugger.IO_data.inputs.brakeFront, debugger.IO_data.inputs.brakeRear);

  // // brake regen
  // Serial.printf("Brake Regen: %d\n", debugger.IO_data.inputs.brakeRegen);

  // // coast regen
  // Serial.printf("Coast Regen: %d\n", debugger.IO_data.inputs.coastRegen);

  // // faults
  // Serial.printf("Faults: IMD: %d | BMS: %d\n", tractiveCoreData.tractive.imdFault, tractiveCoreData.tractive.bmsFault);

  // // rtd
  // Serial.printf("Ready to Drive: %s\n", tractiveCoreData.tractive.readyToDrive ? "READY" : "DEACTIVATED");

  // // inverter
  // Serial.printf("Inverter Enable: %s\n", tractiveCoreData.tractive.enableInverter ? "ENABLED" : "DISABLED");

  // // OUTPUTS
  // Serial.printf("Buzzer Status: %s, Buzzer Counter: %d\n", debugger.IO_data.outputs.buzzerActive ? "On" : "Off", debugger.IO_data.outputs.buzzerCounter);

  // Serial.printf("Commanded Torque: %d\n", tractiveCoreData.tractive.commandedTorque);
  
  // Serial.printf("Drive Mode: %d\n", (int)tractiveCoreData.tractive.driveMode);

  Serial.printf("\n--- END I/O DEBUG ---\n");
}


/**
 * @brief manages toggle-able debug settings & scheduler debugging 
 */
void PrintDebug() {
  // CAN
  if (debugger.TWAI_debugEnabled) {
      PrintTWAIDebug();
  }

  // I/O
  if (debugger.IO_debugEnabled) {
    PrintIODebug();
  }

  // Scheduler
  if (debugger.scheduler_debugEnable) {
    // inits
    std::vector<eTaskState> taskStates;
    std::vector<std::string> taskStatesStrings;
    std::vector<float> taskRefreshRate;

    // gather task information
    taskStates.push_back(eTaskGetState(xHandleIORead));
    taskStates.push_back(eTaskGetState(xHandleIOWrite));
    taskStates.push_back(eTaskGetState(xHandleTWAIRead));
    taskStates.push_back(eTaskGetState(xHandleTWAIWrite));
    taskStates.push_back(eTaskGetState(xHandlePrecharge));

    taskRefreshRate.push_back((debugger.ioReadTaskCount - debugger.ioReadTaskPreviousCount) / MAIN_LOOP_DELAY);
    taskRefreshRate.push_back((debugger.ioWriteTaskCount - debugger.ioWriteTaskPreviousCount) / MAIN_LOOP_DELAY);
    taskRefreshRate.push_back((debugger.twaiReadTaskCount - debugger.twaiReadTaskPreviousCount) / MAIN_LOOP_DELAY);
    taskRefreshRate.push_back((debugger.twaiWriteTaskCount - debugger.twaiWriteTaskPreviousCount) / MAIN_LOOP_DELAY);
    taskRefreshRate.push_back((debugger.prechargeTaskCount - debugger.prechargeTaskPreviousCount) / MAIN_LOOP_DELAY);

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

    // print
    Serial.printf("read io:[%s](%d)<%f.1Hz> | write io:[%s](%d)<%f.1Hz> | read twai:[%s](%d)<%f.1Hz> | write twai:[%s](%d)<%f.1Hz> | precharge:[%s](%d)<%f.1Hz> \r", 
      taskStatesStrings.at(0), debugger.ioReadTaskCount, taskRefreshRate.at(0), taskStatesStrings.at(1), debugger.ioWriteTaskCount, taskRefreshRate.at(1), taskStatesStrings.at(2), 
      debugger.twaiReadTaskCount, taskRefreshRate.at(2), taskStatesStrings.at(3), debugger.twaiWriteTaskCount, taskRefreshRate.at(3), taskStatesStrings.at(4), debugger.prechargeTaskCount, taskRefreshRate.at(4));

    // update counters
    debugger.ioReadTaskPreviousCount = debugger.ioReadTaskCount;
    debugger.ioWriteTaskPreviousCount = debugger.ioWriteTaskCount;
    debugger.twaiReadTaskPreviousCount = debugger.twaiReadTaskCount;
    debugger.twaiWriteTaskPreviousCount = debugger.twaiWriteTaskCount;
    debugger.prechargeTaskPreviousCount = debugger.prechargeTaskCount;
  }
}