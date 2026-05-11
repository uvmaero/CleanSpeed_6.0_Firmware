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

//system
#include <Wire.h>
#include "functions.h"

// port aliases
#define SERIAL_DEBUG Serial
#define I2C_CONN Wire

/*
===============================================================================================
                                            Setup
===============================================================================================
*/

/**
 * @breif set up function
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
  pinMode(BMS_FAULT_IND_PIN, OUTPUT);
  pinMode(IMD_FAULT_IND_PIN, OUTPUT);
  pinMode(TSSI_FAULT_IND_PIN, OUTPUT);

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
    SERIAL_DEBUG.printf("I/O READ TASK STATUS: %s\n", TaskStateToString(eTaskGetState(xHandleIORead)).c_str());
  else
    SERIAL_DEBUG.printf("I/O READ TASK STATUS: DISABLED!\n");

  if (xHandleIOWrite != NULL)
    SERIAL_DEBUG.printf("I/O WRITE TASK STATUS: %s\n", TaskStateToString(eTaskGetState(xHandleIOWrite)).c_str());
  else
    SERIAL_DEBUG.printf("I/O WRITE TASK STATUS: DISABLED!\n");

  if (xHandleTWAIRead != NULL)
    SERIAL_DEBUG.printf("TWAI READ TASK STATUS: %s\n", TaskStateToString(eTaskGetState(xHandleTWAIRead)).c_str());
  else
    SERIAL_DEBUG.printf("TWAI READ TASK STATUS: DISABLED!\n");

  if (xHandleTWAIWrite != NULL)
    SERIAL_DEBUG.printf("TWAI WRITE TASK STATUS: %s\n", TaskStateToString(eTaskGetState(xHandleTWAIWrite)).c_str());
  else
    SERIAL_DEBUG.printf("TWAI WRITE TASK STATUS: DISABLED!\n");

  if (xHandlePrecharge != NULL)
    SERIAL_DEBUG.printf("PRECHARGE TASK STATUS: %s\n", TaskStateToString(eTaskGetState(xHandlePrecharge)).c_str());
  else
    SERIAL_DEBUG.printf("PRECHARGE TASK STATUS: DISABLED!\n");

  if (xHandleTelemetryUpdate != NULL)
    SERIAL_DEBUG.printf("TELEMETRY UPDATE TASK STATUS: %s\n", TaskStateToString(eTaskGetState(xHandleTelemetryUpdate)).c_str());
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

/**
 * @brief main loop!
 */
void loop()
{
  // everything is managed by RTOS, so nothing really happens here!
  vTaskDelay(1); // prevent watchdog from getting upset
}