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
        std::cout<<"FieldbusState::INIT "<<std::endl;
        /* code */
        break;
    case FieldbusState::NOT_READY:
        std::cout<<"FieldbusState::NOT_READY "<<std::endl;
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
        std::cout<<"FieldbusState::READY "<<std::endl;
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
        std::cout<<"FieldbusState::OPERATIONAL "<<std::endl;
        if (logicStateDataPtr->state == LogicState::NOT_OK)
        {
            fieldbusSharedDataPtr->state = FieldbusState::ERROR;
        }
        break;
    case FieldbusState::ERROR:
    {
        std::cout<<"FieldbusState::ERROR "<<std::endl;
        /* code */
        logicStateDataPtr->allDriveReady = false;
        logicStateDataPtr->state = LogicState::NOT_EVAL;

        int chk_ctr = 0;

        for (int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
        {
            std::cout<<"Inside Switch on Discaled : "<<jnt_ctr<<" : "<<(driveObjectPtr[jnt_ctr]->statusword == StatusWordValues::SW_SWITCH_ON_DISABLED)<<std::endl;
            if (driveObjectPtr[jnt_ctr]->statusword == StatusWordValues::SW_SWITCH_ON_DISABLED)
            {
                chk_ctr++;
                std::cout<<"chk_ctr : "<<chk_ctr<<std::endl;
            }
        }

        if (chk_ctr == NUM_JOINTS)
        {
            std::cout << "All Joints Switch On Disabled" << std::endl;
            fieldbusSharedDataPtr->state = FieldbusState::NOT_READY;
        }

        break;
    }
    default:
        break;
    }
}