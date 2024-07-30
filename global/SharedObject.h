#pragma once

#include <cstring>
#include <iostream>
#include <stdint.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>

constexpr int NUM_JOINTS = 6; // Change this to the desired number of joints



enum class OperationModeState
{
    POSITION_MODE = 8,
    VELOCITY_MODE = 9,
    TORQUE_MODE = 10,
};


enum class StatusWordValues : uint16_t
{
    SW_NOT_READY_TO_SWITCH_ON = 0,
    SW_SWITCH_ON_DISABLED = 64,
    SW_READY_TO_SWITCH_ON = 33,
    SW_SWITCHED_ON = 35,
    SW_OPERATION_ENABLED = 39,
    SW_QUICK_STOP_ACTIVE = 7,
    SW_FAULT_REACTION_ACTIVE = 15,
    SW_FAULT = 8
    // Add more status word values as needed
};

enum class ControlWordValues : uint16_t
{
    CW_SHUTDOWN = 6,
    CW_READY_TO_SWITCH_ON=6,
    CW_SWITCH_ON = 7,
    CW_DISABLE_OPERATION=7,
    CW_ENABLE_OPERATION = 15,
    CW_DISABLE_VOLTAGE = 0,
    CW_QUICK_STOP = 2,
    CW_RESET = 128,
    // Add more control word values as needed
};

enum class LogicLimitCheck
{
    OK,
    NOT_OK,
    NOT_EVAL,
};

enum class FieldbusState
{
    INIT,
    NOT_READY,
    READY,
    OPERATIONAL,
    ERROR,
};

enum class UserInputState 
{
    INIT,
    START,
    SWITCH_ON,
    IDLE,
    HANDCONTROLLER,
    HOMING,
    OPERATIONAL
};
enum class LogicState 
{
    INIT,
    START,
    IDLE,
    HANDCONTROLLER,
    HOMING,
    OPERATIONAL
};

struct ServoDrives
{
    bool enableDrive;
    int32_t actual_position;
    int32_t actual_velocity;
    int16_t actual_torque;
    StatusWordValues statusword;

    int32_t target_position;
    int32_t target_velocity;
    int16_t target_torque;
    OperationModeState mode_of_operation;
    ControlWordValues controlword;

    // Add error code
    uint32_t vendor_id;
    uint32_t product_code;
    uint16_t alias;
    uint16_t position;
    
    //motor_info
    double gear_ratio;
    double motor_rated_torque;
    double encoder_resolution;
    double axis_direction;  
    double torque_axis_direction; 


};

struct EthercatStateData
{
    FieldbusState state;
    bool allDrivesOpEnabled;
    OperationModeState operationMode;
};

struct LogicStateData
{
    LogicLimitCheck state;
    bool allDriveReady;
    UserInputState userInput;
    LogicState prcoessState;
    int cycles;
    //UserInputState prevUserInput;
};

//for actuator control : processing encoder values
// struct JointParameters
// {
//     double gear_ratio;
//     double motor_rated_torque;
//     double encoder_resolution;
//     double axis_direction;   
// };
//for storinf processed data from encoder values and test safety check or calculate taget data from actual data
struct JointOutputData
{
    double actual_position;
    double actual_velocity;
    double actual_torque;

    double target_position;
    double target_velocity;
    double target_torque;
};
struct handcontroller
{
    
};
// struct Cycles
// {
//     int logic;
//     int logic_drive;
//     int ethercat;
// };
struct JointFrictionCoff
{
    double pos1;
    double pos2;
    double pos3;
    double pos4;

    double neg1;
    double neg2;
    double neg3;
    double neg4;
};
