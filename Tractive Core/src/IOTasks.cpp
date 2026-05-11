/**
 * Created by ethan on 4/9/26.
 * IO read and write tasks
*/

/*
===========================================================
                        Includes
===========================================================
*/

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
 * @brief reads I/O
 * @param pvParameters parameters passed to task
 */
void IOReadTask(void *pvParameters)
{
  for (;;)
  {
    // check for mutex availability
    if (xSemaphoreTake(xMutex, 10) == pdTRUE)
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
      if (digitalRead(TRACTION_CONTROL_SWITCH_PIN) == HIGH)
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
      if (digitalRead(BMS_FAULT_PIN) != LOW)
      {
        tractiveCoreData.sensors.bmsFault = true;
      }
      else
      {
        tractiveCoreData.sensors.bmsFault = false;
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
      int tmpCoolingIn = analogReadMilliVolts(PUMP_TEMP_IN_PIN);
      tractiveCoreData.sensors.coolingTempIn = map(tmpCoolingIn, 0, 2500, 0, 100); // find thermistor values via testing

      int tmpCoolingOut = analogReadMilliVolts(PUMP_TEMP_OUT_PIN);
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
        digitalWrite(BMS_FAULT_IND_PIN, HIGH);
        digitalWrite(TSSI_FAULT_IND_PIN, HIGH);
      } else if (tractiveCoreData.sensors.imdFault)
      {
        digitalWrite(IMD_FAULT_IND_PIN, HIGH);
        digitalWrite(TSSI_FAULT_IND_PIN, HIGH);
      } else
      {
        digitalWrite(BMS_FAULT_IND_PIN, LOW);
        digitalWrite(IMD_FAULT_IND_PIN, LOW);
        digitalWrite(TSSI_FAULT_IND_PIN, LOW);
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


