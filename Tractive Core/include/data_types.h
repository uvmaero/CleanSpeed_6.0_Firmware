/**
 * @file dataTypes.h
 * @author Dominic Gasperini
 * @brief all of the unique data types used to manage the state of the car
 * @version 1.6
 * @date 2024-01-22
 */


/*
===============================================================================================
                                    Includes 
===============================================================================================
*/


#include <esp_err.h>


/*
===============================================================================================
                                    Data Types
===============================================================================================
*/


/**
 * @brief the different throttle modes for to modify driving behavior
 * 
 */
enum DriveMode
{
    SLOW = 1,
    ECO = 2,
    FAST = 3,
};


/**
 * @brief each state that the precharge state machine can be in
 * 
 */
enum PrechargeStates
{
    PRECHARGE_OFF = 0,
    PRECHARGE_ON = 1,
    PRECHARGE_DONE = 2,
    PRECHARGE_ERROR = 3,
};


/**
 * @brief tractive state of the core
 * 
 */
struct TractiveCoreData
{
    struct Tractive 
    {
        bool readyToDrive = false;
        bool enableInverter = false;

        PrechargeStates prechargeState = PRECHARGE_OFF;

        float rinehartVoltage = 0.0f;

        uint16_t commandedTorque = 0;

        bool driveDirection = false;             // true = forward | false = reverse
        DriveMode driveMode = ECO;

        float currentSpeed = 0.0f;

        bool tractionControlEnable = true;
        float tractionControlModifier = 1.00f;

        uint16_t coastRegen = 0;
        uint16_t brakeRegen = 0;
    } tractive;

    struct Sensors
    {
        bool imdFault = true;
        bool bmsFault = true;
        bool vicoreFault = false;

        float coolingTempIn = 0.0f;
        float coolingTempOut = 0.0f;

        float frontWheelsSpeed = 0.0f;
        int16_t frontWheelSpeedCount = 0;
        int16_t frontWheelSpeedTime = 0;

        float brWheelSpeed = 0.0f;
        int16_t brWheelSpeedCount = 0;
        int16_t brWheelSpeedTime = 0;

        float blWheelSpeed = 0.0f;
        int16_t blWheelSpeedCount = 0;
        int16_t blWheelSpeedTime = 0;
    } sensors;
    
    struct Inputs
    {
        uint16_t pedal0 = 0;
        uint16_t pedal1 = 0;

        uint16_t frontBrake = 0;
        uint16_t rearBrake = 0;
    } inputs;

    struct Outputs
    {
        bool vicoreEnable = false;

        bool brakeLightEnable = false;

        bool fansEnable = false;

        bool buzzerEnable = false;
    } outputs;

    struct Orion
    {        
        float batteryChargeState = 0.0f;

        float busVoltage = 0.0f;

        float packCurrent = 0.0f;

        float minCellVoltage = 0.0f;
        float maxCellVoltage = 0.0f;
        float minCellTemp = 0.0f;
        float maxCellTemp = 0.0f;
    } orion;

};



/**
 * @brief Debugger Structure
 * 
 */
struct Debugger
{
    // debug toggle
    bool debugEnabled = false;
    bool TWAI_debugEnabled = false;
    bool IO_debugEnabled = false;
    bool scheduler_debugEnable = true;

    // TWAI data
    esp_err_t TWAI_rinehartCtrlResult = ESP_OK;
    esp_err_t TWAI_prechargeCtrlResult = ESP_OK;
    esp_err_t telemetryTractive1MessageResult = ESP_OK;
    esp_err_t telemetryTractive2MessageResult = ESP_OK;
    esp_err_t telemetrySensorMessageResult = ESP_OK;
    esp_err_t telemetryInputsMessageMessageResult = ESP_OK;
    esp_err_t telemetryOutputsMessageMessageResult = ESP_OK;

    uint8_t TWAI_rinehartCtrlMessage[8] = {};
    uint8_t TWAI_prechargeCtrlMessage[8] = {};

    // I/O data
    TractiveCoreData IO_data = {};

    // precharge data
    PrechargeStates prechargeState = PRECHARGE_OFF;

    // scheduler data
    int ioReadTaskCount = 0;
    int ioWriteTaskCount = 0;
    int twaiReadTaskCount = 0;
    int twaiWriteTaskCount = 0;
    int prechargeTaskCount = 0;
    int telemetryUpdateTaskCount = 0;

    int ioReadTaskPreviousCount = 0;
    int ioWriteTaskPreviousCount = 0;
    int twaiReadTaskPreviousCount = 0;
    int twaiWriteTaskPreviousCount = 0;
    int prechargeTaskPreviousCount = 0;
    int telemetryUpdateTaskPreviousCount = 0;
};


// debug functions
void PrintTWAIDebug();
void PrintIODebug();
void PrintScheduler();