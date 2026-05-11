/**
* Created by ethan on 4/9/26.
 * TWAI read and write tasks
*/

/*
===========================================================
                        Includes
===========================================================
*/

#include <driver/twai.h>

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

        SERIAL_DEBUG.println(String(reinterpret_cast<char*>(incomingMessage.data)));

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
