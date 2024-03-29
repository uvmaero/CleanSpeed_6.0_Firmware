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
typedef enum DriveMode
{
    SLOW = 1,
    ECO = 2,
    FAST = 3,
} DriveMode;


/**
 * @brief each state that the precharge state machine can be in
 * 
 */
typedef enum PrechargeStates
{
    PRECHARGE_OFF = 0,
    PRECHARGE_ON = 1,
    PRECHARGE_DONE = 2,
    PRECHARGE_ERROR = 3,
} PrechargeStates;


/**
 * @brief tractive state of the core
 * 
 */
typedef struct TractiveCoreData
{
    struct Tractive 
    {
        bool readyToDrive;
        bool enableInverter;

        PrechargeStates prechargeState;

        float rinehartVoltage;

        uint16_t commandedTorque;

        bool driveDirection;             // true = forward | false = reverse
        DriveMode driveMode;

        float currentSpeed;

        bool tractionControlEnable;
        float tractionControlModifier;

        uint16_t coastRegen;
        uint16_t brakeRegen;
    } tractive;

    struct Sensors
    {
        bool imdFault;
        bool bmsFault;
        bool vicoreFault;

        float coolingTempIn;
        float coolingTempOut;

        float frontWheelsSpeed;
        int16_t frontWheelSpeedCount;
        int16_t frontWheelSpeedTime;

        float brWheelSpeed;
        int16_t brWheelSpeedCount;
        int16_t brWheelSpeedTime;

        float blWheelSpeed;
        int16_t blWheelSpeedCount;
        int16_t blWheelSpeedTime;
    } sensors;
    
    struct Inputs
    {
        uint16_t pedal0;
        uint16_t pedal1;

        uint16_t frontBrake;
        uint16_t rearBrake;
    } inputs;

    struct Outputs
    {
        bool vicoreEnable;

        bool brakeLightEnable;

        bool fansEnable;

        bool buzzerEnable;
    } outputs;

    struct Orion
    {        
        float batteryChargeState;

        float busVoltage;

        float packCurrent;

        float minCellVoltage;
        float maxCellVoltage;
        float minCellTemp;
        float maxCellTemp;
    } orion;

} TractiveCoreData;



/**
 * @brief Debugger Structure
 * 
 */
typedef struct Debugger
{
    // debug toggle
    bool debugEnabled;
    bool TWAI_debugEnabled;
    bool IO_debugEnabled;
    bool scheduler_debugEnable;

    // TWAI data
    esp_err_t TWAI_rinehartCtrlResult;
    esp_err_t TWAI_prechargeCtrlResult;
    esp_err_t telemetryTractive1MessageResult;
    esp_err_t telemetryTractive2MessageResult;
    esp_err_t telemetrySensorMessageResult;
    esp_err_t telemetryInputsMessageMessageResult;
    esp_err_t telemetryOutputsMessageMessageResult;
    uint8_t TWAI_rinehartCtrlMessage[8];
    uint8_t TWAI_prechargeCtrlMessage[8];

    // I/O data
    TractiveCoreData IO_data;

    // precharge data
    PrechargeStates prechargeState;

    // scheduler data
    int ioReadTaskCount;
    int ioWriteTaskCount;
    int twaiReadTaskCount;
    int twaiWriteTaskCount;
    int prechargeTaskCount;
    int telemetryUpdateTaskCount;

    int ioReadTaskPreviousCount;
    int ioWriteTaskPreviousCount;
    int twaiReadTaskPreviousCount;
    int twaiWriteTaskPreviousCount;
    int prechargeTaskPreviousCount;
    int telemetryUpdateTaskPreviousCount;
} Debugger;


// debug functions
void PrintTWAIDebug();
void PrintIODebug();
void PrintScheduler();