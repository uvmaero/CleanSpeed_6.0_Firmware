/**
* Created by ethan on 4/9/26.
* Helper functions
*/

/*
===========================================================
                        Includes
===========================================================
*/

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
 * @brief Get the Commanded Torque from pedal values
 */
void GetCommandedTorque()
{
  // vars
  uint16_t commandedTorque;
  long pedal0map;
  long pedal1map;

  // drive mode logic (values are 10x because that is the format for Rinehart)
  switch (tractiveCoreData.tractive.driveMode)
  {
  case SLOW: // runs at 50% power
    pedal0map = map(tractiveCoreData.inputs.pedal0, PEDAL0_MIN, PEDAL0_MAX, 0, MAX_TORQUE * 10 * 0.5);
    pedal1map = map(tractiveCoreData.inputs.pedal1, PEDAL1_MIN, PEDAL1_MAX, 0, MAX_TORQUE * 10 * 0.5);
    commandedTorque = (pedal0map + pedal1map) / 2;
    break;

  case ECO: // runs at 75% power
    pedal0map = map(tractiveCoreData.inputs.pedal0, PEDAL0_MIN, PEDAL0_MAX, 0, MAX_TORQUE * 10 * 0.75);
    pedal1map = map(tractiveCoreData.inputs.pedal1, PEDAL1_MIN, PEDAL1_MAX, 0, MAX_TORQUE * 10 * 0.75);
    commandedTorque = (pedal0map + pedal1map) / 2;
    break;

  case FAST: // runs at 100% power
    pedal0map = map(tractiveCoreData.inputs.pedal0, PEDAL0_MIN, PEDAL0_MAX, 0, MAX_TORQUE * 10);
    pedal1map = map(tractiveCoreData.inputs.pedal1, PEDAL1_MIN, PEDAL1_MAX, 0, MAX_TORQUE * 10);
    commandedTorque = (pedal0map + pedal1map) / 2;
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

