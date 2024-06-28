#pragma once

#include <cstring>
#include <iostream>

#include <fcntl.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>

constexpr int NUM_JOINTS = 4; // Change this to the desired number of joints

enum class OperationModeState
{
    POSITION_MODE = 8,
    VELOCITY_MODE = 9,
    TORQUE_MODE = 10,
};

enum class FieldbusState
{
    INIT,
    NOT_READY,
    READY,
    OPERATIONAL,
    ERROR,
};


enum class StatusWordValues : uint16_t
{
    SW_NOT_READY_TO_SWITCH_ON = 0x0000,
    SW_SWITCH_ON_DISABLED = 0x0040,
    SW_READY_TO_SWITCH_ON = 0x0021,
    SW_SWITCHED_ON = 0x0023,
    SW_OPERATION_ENABLED = 0x0027,
    SW_QUICK_STOP_ACTIVE = 0x0007,
    SW_FAULT_REACTION_ACTIVE = 0x000F,
    SW_FAULT = 0x0008
    // Add more status word values as needed
};

enum class ControlWordValues : uint16_t
{
    CW_SHUTDOWN = 0x06,
    CW_SWITCH_ON = 0x07,
    CW_ENABLE_OPERATION = 0x0F,
    CW_DISABLE_VOLTAGE = 0x00,
    CW_QUICK_STOP = 0x02,
    CW_RESET = 0x80,
    // Add more control word values as needed
};

struct ServoDrives
{
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
};

struct JointParameters
{
    double gear_ratio;
    double motor_rated_torque;
    double encoder_resolution;
    double axis_direction;   
};

struct EthercatStateData
{
    FieldbusState state;
    bool allDrivesEnabledOp;
    OperationModeState operationMode;
};

struct JointOutputData
{
    double actual_position;
    double actual_velocity;
    double actual_torque;

    double target_position;
    double target_velocity;
    double target_torque;
};
