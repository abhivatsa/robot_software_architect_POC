#include "driveLogicSharedMemory.h"

DriveLogic::DriveLogic(/* args */)
{
    configureSharedMemory();
}

DriveLogic::~DriveLogic()
{
}

void DriveLogic::run()
{

    switch (fieldbusSharedDataPtr->state)
    {
    case FieldbusState::INIT:
        /* code */
        break;
    case FieldbusState::NOT_READY:
        for (int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
        {
            if (driveObjectPtr[jnt_ctr]->statusword != StatusWordValues::SW_SWITCH_ON_DISABLED)
            {
                fieldbusSharedDataPtr->state = FieldbusState::ERROR;
                return;
            }
        }
        fieldbusSharedDataPtr->state = FieldbusState::READY;
        break;
    case FieldbusState::READY:
        for (int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
        {
            if (driveObjectPtr[jnt_ctr]->statusword != StatusWordValues::SW_SWITCHED_ON)
            {
                return;
            }
        }

        logicStateDataPtr->allDriveReady = true;

        switch (logicStateDataPtr->state)
        {
        case LogicState::OK:
            fieldbusSharedDataPtr->state = FieldbusState::OPERATIONAL;
            break;
        case LogicState::NOT_OK:
            fieldbusSharedDataPtr->state = FieldbusState::ERROR;
            break;
        default:
            break;
        }

        break;
    case FieldbusState::OPERATIONAL:
        if (logicStateDataPtr->state == LogicState::NOT_OK)
        {
            fieldbusSharedDataPtr->state = FieldbusState::ERROR;
        }
        break;
    case FieldbusState::ERROR:
        /* code */
        logicStateDataPtr->allDriveReady = false;
        logicStateDataPtr->state = LogicState::NOT_EVAL;
        break;

    default:
        break;
    }

}