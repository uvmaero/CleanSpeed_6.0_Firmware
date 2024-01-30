/**
 * @file dataTypes.h
 * @author dominic gasperini
 * @brief all of the unique data types used to manage the state of the car
 * @version 1.0
 * @date 2024-01-23
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
 * @brief telemetry state of the core
 * 
 */
typedef struct TelemetryCoreData
{
    TractiveCoreData tractiveCoreData;

    struct Dampers 
    {
        uint8_t frSuspensionDamper;
        uint8_t flSuspensionDamper;
        uint8_t brSuspensionDamper;
        uint8_t blSuspensionDamper;
    } dampers;

    struct TireTemp
    {
        uint8_t frTireTemp;
        uint8_t flTireTemp;
        uint8_t brTireTemp;
        uint8_t blTireTemp;
    } tireTemp;

    struct Strain
    {
        uint8_t frStrain1;
        uint8_t flStrain1;
        uint8_t brStrain1;
        uint8_t blStrain1;

        uint8_t frStrain2;
        uint8_t flStrain2;
        uint8_t brStrain2;
        uint8_t blStrain2;
    } strain;

    struct Steering
    {
        uint16_t steeringWheelDeflection;
    } steering;

    struct IMU
    {
        float xAcceleration;
        float yAcceleration;
        float zAcceleration;
        float xGyro;
        float yGyro;
        float zGyro;
    } imu;

    struct GPS
    {
        float latitide;
        float longitude;
        float altitude;
        float speed;
        uint8_t year;
        uint8_t month;
        uint8_t day;
    } gps;
} TelemetryCoreData;


/**
 * @brief Debugger Structure
 * 
 */
typedef struct Debugger
{
    // debug toggle
    bool debugEnabled;
    bool serial_debugEnabled;
    bool IO_debugEnabled;
    bool twai_debugEnable;
    bool scheduler_debugEnable;

    // I/O data
    TelemetryCoreData IO_data;

    // scheduler data
    int ioReadTaskCount;
    int serialReadTaskCount;
    int serialWriteTaskCount;
    int tractiveReadTaskCount;
    int twaiReadTaskCount;

    int ioReadTaskPreviousCount;
    int serialReadTaskPreviousCount;
    int serialWriteTaskPreviousCount;
    int tractiveReadTaskPreviousCount;
    int twaiReadTaskPreviousCount;
} Debugger;


// debug functions
void PrintIODebug();
void PrintSerialDebug();
void PrintSchedulerDebug();