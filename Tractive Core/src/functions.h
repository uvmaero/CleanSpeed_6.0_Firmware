/**
 * Created by ethan on 4/9/26.
 * Central Header for all the
 * essential car functions
*/

#ifndef TRACTIVE_CORE_FUNCTIONS_H
#define TRACTIVE_CORE_FUNCTIONS_H

/*
===========================================================
                        Includes
===========================================================
*/

#pragma once
#include "pin_config.h"

/*
===========================================================
                    Declarations
===========================================================
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

// debug functions
void PrintTWAIDebug();
void PrintIODebug();
void PrintScheduler();

#endif //TRACTIVE_CORE_FUNCTIONS_H
