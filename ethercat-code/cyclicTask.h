#pragma once

#include "transistionState.h"

using namespace std;

void EthercatMaster::inc_period(struct period_info *pinfo)
{

    pinfo->next_period.tv_nsec += pinfo->period_ns;

    while (pinfo->next_period.tv_nsec >= 1000000000)
    {
        /* timespec nsec overflow */
        pinfo->next_period.tv_sec++;
        pinfo->next_period.tv_nsec -= 1000000000;
    }
}

void EthercatMaster::periodic_task_init(struct period_info *pinfo)
{
    /* for simplicity, hardcoding a 1ms period */
    pinfo->period_ns = 1000000;

    clock_gettime(CLOCK_MONOTONIC, &(pinfo->next_period));
}

void EthercatMaster::wait_rest_of_period(struct period_info *pinfo)
{
    inc_period(pinfo);

    /* for simplicity, ignoring possibilities of signal wakes */
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, NULL);
}

void EthercatMaster::cyclicTask()
{
    struct period_info pinfo;

    periodic_task_init(&pinfo);

    std::cout<<"Line 42 "<<std::endl;

    fieldbusSharedDataPtr->state = FieldbusState::NOT_READY;

    while (true)
    // while (!exitFlag)
    {
        ecrt_master_application_time(master, ((uint64_t)pinfo.next_period.tv_sec * 1000000000 + pinfo.next_period.tv_nsec));
        ecrt_master_receive(master);
        ecrt_domain_process(domain);
        checkDomainState();
        checkMasterState();
        do_rt_task();
        ecrt_domain_queue(domain);
        ecrt_master_send(master);
        wait_rest_of_period(&pinfo);
    }

    return;
}

void EthercatMaster::do_rt_task()
{
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        driveObjectPtr[i]->statusword = readDriveState(i);
    }

    switch (fieldbusSharedDataPtr->state)
    {
    case FieldbusState::NOT_READY:
        handleNotReadyState();
        break;
    case FieldbusState::READY:
        handleReadyState();
        break;
    case FieldbusState::OPERATIONAL:
        handleOperationalState();
        break;
    case FieldbusState::ERROR:
        handleErrorState();
        break;
    default:
        break;
    }
}

void EthercatMaster::handleNotReadyState()
{
    // checking if all the statusword are in SWITCH_ON_DISABLED

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        if (readDriveState(i) != StatusWordValues::SW_SWITCH_ON_DISABLED)
        {
            std::cout<<"readDriveState(i)  : "<<(readDriveState(i) == StatusWordValues::SW_NOT_READY_TO_SWITCH_ON) <<std::endl;
            fieldbusSharedDataPtr->state = FieldbusState::ERROR;
            return;
        }
    }

    std::cout<<"All Drives are in Switched on"<<std::endl;
}

void EthercatMaster::handleReadyState()
{
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        switch (readDriveState(i))
        {
        case StatusWordValues::SW_SWITCH_ON_DISABLED:
            transitionToState(ControlWordValues::CW_READY_TO_SWITCH_ON, i);
            break;
        case StatusWordValues::SW_READY_TO_SWITCH_ON:
            transitionToState(ControlWordValues::CW_SWITCH_ON, i);
            break;
        case StatusWordValues::SW_SWITCHED_ON:
            for (int i = 0; i < NUM_JOINTS; i++)
            {
                driveObjectPtr[i]->actual_position = EC_READ_S32(domainPd + driveOffset[i].position_actual_value);
                driveObjectPtr[i]->actual_velocity = EC_READ_S32(domainPd + driveOffset[i].velocity_actual_value);
                driveObjectPtr[i]->actual_torque = EC_READ_S16(domainPd + driveOffset[i].torque_actual_value);
            }
            break;
        default:
            fieldbusSharedDataPtr->state = FieldbusState::ERROR;
            break;
        }
    }
}

void EthercatMaster::handleOperationalState()
{
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        switch (readDriveState(i))
        {
        case StatusWordValues::SW_SWITCHED_ON:
            transitionToState(ControlWordValues::CW_ENABLE_OPERATION, i);
            for (int i = 0; i < NUM_JOINTS; i++)
            {
                driveObjectPtr[i]->actual_position = EC_READ_S32(domainPd + driveOffset[i].position_actual_value);
                driveObjectPtr[i]->actual_velocity = EC_READ_S32(domainPd + driveOffset[i].velocity_actual_value);
                driveObjectPtr[i]->actual_torque = EC_READ_S16(domainPd + driveOffset[i].torque_actual_value);
            }
            break;
        case StatusWordValues::SW_OPERATION_ENABLED:
            for (int i = 0; i < NUM_JOINTS; i++)
            {
                driveObjectPtr[i]->actual_position = EC_READ_S32(domainPd + driveOffset[i].position_actual_value);
                driveObjectPtr[i]->actual_velocity = EC_READ_S32(domainPd + driveOffset[i].velocity_actual_value);
                driveObjectPtr[i]->actual_torque = EC_READ_S16(domainPd + driveOffset[i].torque_actual_value);
            }
            if (fieldbusSharedDataPtr->allDrivesOpEnabled)
            {
                std::cout<<"mode of operation : "<<(int)(fieldbusSharedDataPtr->operationMode)<<std::endl;
                std::cout<<"target position : "<<driveObjectPtr[0]->target_position<<std::endl;
                // switch (fieldbusSharedDataPtr->operationMode)
                // {
                // case OperationModeState::POSITION_MODE:
                //     for (int i = 0; i < NUM_JOINTS; i++)
                //     {
                //         EC_WRITE_U16(domainPd + driveOffset[i].modes_of_operation, (int)OperationModeState::POSITION_MODE);
                //         EC_WRITE_S32(domainPd + driveOffset[i].target_position, driveObjectPtr[i]->target_position);
                //     }
                //     break;

                // case OperationModeState::VELOCITY_MODE:
                //     for (int i = 0; i < NUM_JOINTS; i++)
                //     {
                //         EC_WRITE_U16(domainPd + driveOffset[i].modes_of_operation, (int)OperationModeState::VELOCITY_MODE);
                //         EC_WRITE_S32(domainPd + driveOffset[i].target_velocity, driveObjectPtr[i]->target_velocity);
                //     }
                //     break;
                
                // case OperationModeState::TORQUE_MODE:
                //     for (int i = 0; i < NUM_JOINTS; i++)
                //     {
                //         EC_WRITE_U16(domainPd + driveOffset[i].modes_of_operation, (int)OperationModeState::TORQUE_MODE);
                //         EC_WRITE_S16(domainPd + driveOffset[i].target_torque, driveObjectPtr[i]->target_torque);
                //     }
                //     break;
                // default:
                //     break;
                // }
            }

            break;
        default:
            fieldbusSharedDataPtr->state = FieldbusState::ERROR;
            break;
        }
    }

}

void EthercatMaster::handleErrorState()
{
    // user needs to approve the fault , added later
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        if (readDriveState(i) == StatusWordValues::SW_NOT_READY_TO_SWITCH_ON){
            std::cout<<"Drive Not Ready To Switch ON"<<std::endl;
            return;
        }
        else if (readDriveState(i) == StatusWordValues::SW_FAULT)
        {
            transitionToState(ControlWordValues::CW_RESET, i);
        }
        else if (readDriveState(i) != StatusWordValues::SW_SWITCH_ON_DISABLED)
        {
            transitionToState(ControlWordValues::CW_DISABLE_VOLTAGE, i);
        }
    }
}
