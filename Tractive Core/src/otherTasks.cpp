/**
* Created by ethan on 4/9/26.
* Precharge, Telemetry, Debug, Tasks
*/

/*
===========================================================
                        Includes
===========================================================
*/

#include <Wire.h>

#include "data_types.h"
#include "functions.h"
// port aliases
#define SERIAL_DEBUG Serial
#define I2C_CONN Wire

/*
===========================================================
                    Implementations
===========================================================
*/

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