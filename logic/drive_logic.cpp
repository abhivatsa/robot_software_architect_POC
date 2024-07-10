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
    {
         // std::cout<<"FieldbusState::NOT_READY "<<std::endl;
        // for (int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
        // {
        //     if (driveObjectPtr[jnt_ctr]->statusword != StatusWordValues::SW_SWITCH_ON_DISABLED)
        //     {
        //         fieldbusSharedDataPtr->state = FieldbusState::ERROR;
        //         return;
        //     }
        // }
        // fieldbusSharedDataPtr->state = FieldbusState::READY;
        ////------------------------------------

        // if(logicStateDataPtr->cycles>5000)
        // {
        //     fieldbusSharedDataPtr->state = FieldbusState::ERROR;
        //     break;
        // }

        int count=0;
        std::cout<<"FieldbusState::NOT_READY "<<std::endl;
        for (int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
        {
            if(driveObjectPtr[jnt_ctr]->statusword == StatusWordValues::SW_SWITCH_ON_DISABLED)
            {
                
                count++;
            }
        }
        std::cout<<"in the stuff count : "<<count<<std::endl;
        std::cout<<"NUM_JOINTS : "<<NUM_JOINTS<<std::endl;
        if(count==NUM_JOINTS)
        {
            std::cout<<"changing NOT_READY to READY "<<std::endl;
            fieldbusSharedDataPtr->state = FieldbusState::READY;
            logicStateDataPtr->cycles=0;
        }

        //logicStateDataPtr->cycles++;
        break;

    }
       
    case FieldbusState::READY:
    {
        std::cout<<"FieldbusState::READY "<<std::endl;

        // if(logicStateDataPtr->cycles>5000)
        // {
        //     fieldbusSharedDataPtr->state = FieldbusState::ERROR;
        // }

        int count=0;
        for (int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
        {
            if (driveObjectPtr[jnt_ctr]->statusword == StatusWordValues::SW_SWITCHED_ON)
            {
                count++;
            }
            else{
                //logicStateDataPtr->cycles++;
            }
        }

        if(count==NUM_JOINTS)
        {
            logicStateDataPtr->allDriveReady = true;
            logicStateDataPtr->cycles=0;

            //testing
            std::cout<<"mode of operation : "<<(int)(fieldbusSharedDataPtr->operationMode)<<std::endl;
                std::cout<<"target position : "<<driveObjectPtr[0]->target_position<<std::endl;
            //
            
            if(logicStateDataPtr->state==LogicLimitCheck::NOT_OK)
            {
                fieldbusSharedDataPtr->state = FieldbusState::ERROR;
            }

            if(logicStateDataPtr->userInput!=UserInputState::IDLE)
            {
                if(logicStateDataPtr->state==LogicLimitCheck::OK)
                {
                    fieldbusSharedDataPtr->state = FieldbusState::OPERATIONAL;
                }
            }
        }
        break;
    }
        
    case FieldbusState::OPERATIONAL:
        std::cout<<"FieldbusState::OPERATIONAL "<<std::endl;
        if (logicStateDataPtr->state == LogicLimitCheck::NOT_OK)
        {
            fieldbusSharedDataPtr->state = FieldbusState::ERROR;
        }
        else if(logicStateDataPtr->userInput==UserInputState::IDLE)
        {
            fieldbusSharedDataPtr->state = FieldbusState::READY;
            logicStateDataPtr->state==LogicLimitCheck::NOT_EVAL;
        }
        break;
    case FieldbusState::ERROR:
    {
        std::cout<<"FieldbusState::ERROR "<<std::endl;
        /* code */
        logicStateDataPtr->allDriveReady = false;
        logicStateDataPtr->state = LogicLimitCheck::NOT_EVAL;

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