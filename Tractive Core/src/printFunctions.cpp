/**
* Created by ethan on 4/9/26.
* Print tasks
*/

/*
===========================================================
                        Includes
===========================================================
*/

#include <driver/twai.h>
#include <rtc.h>

#include <vector>

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