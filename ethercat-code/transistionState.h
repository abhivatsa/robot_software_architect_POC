#pragma once
#include "master.h"

void EthercatMaster::transitionToState(ControlWordValues value, int jnt_ctr)
{
    EC_WRITE_U16(domainPd + driveOffset[jnt_ctr].controlword, value);
}

StatusWordValues EthercatMaster::readDriveState(int joint_num)
{
    uint16_t drive_status = EC_READ_U16(domainPd + driveOffset[joint_num].statusword);

    // cout << "update_state" << endl;
    if ((drive_status & 79) == (int)StatusWordValues::SW_NOT_READY_TO_SWITCH_ON)
    {
        return StatusWordValues::SW_NOT_READY_TO_SWITCH_ON;
    }
    else if ((drive_status & 79 ) == (int)StatusWordValues::SW_SWITCH_ON_DISABLED)
    {
        return StatusWordValues::SW_SWITCH_ON_DISABLED;
    }
    else if ((drive_status & 111 ) == (int)StatusWordValues::SW_READY_TO_SWITCH_ON)
    {
        return StatusWordValues::SW_READY_TO_SWITCH_ON;
    }
    else if ((drive_status & 111 ) == (int)StatusWordValues::SW_SWITCHED_ON)
    {
        return StatusWordValues::SW_SWITCHED_ON;
    }
    else if ((drive_status & 111 ) == (int)StatusWordValues::SW_OPERATION_ENABLED)
    {
        return StatusWordValues::SW_OPERATION_ENABLED;
    }
    else if ((drive_status & 79 ) == (int)StatusWordValues::SW_FAULT_REACTION_ACTIVE)
    {
        // Fault Reaction Active
        return StatusWordValues::SW_FAULT_REACTION_ACTIVE;
    }
    else if ((drive_status & 79) == (int)StatusWordValues::SW_FAULT)
    {
        // Fault
        return StatusWordValues::SW_FAULT;
    }
    else{
        return StatusWordValues::SW_NOT_READY_TO_SWITCH_ON;
    }
}