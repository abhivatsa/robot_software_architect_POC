#include "logicSharedMemory.h"

Logic::Logic(/* args */)
{
    configureSharedMemory();


    logicStateDataPtr->state = LogicLimitCheck::NOT_EVAL;
    logicStateDataPtr->allDriveReady = false;
    logicStateDataPtr->userInput=UserInputState::IDLE;
    //logicStateDataPtr->prevUserInput=UserInputState::START;
    logicStateDataPtr->cycles=0;
    fieldbusSharedDataPtr->allDrivesOpEnabled = false;
    fieldbusSharedDataPtr->operationMode = OperationModeState::TORQUE_MODE;
    fieldbusSharedDataPtr->state = FieldbusState::INIT;

    double gear_ratio[6] = {161, 161, 161, 161, 101, 101};
    double motor_rated_torque[6] = {2.56, 2.56, 1.5, 1.5, 0.4, 0.4};
    double encoder_resolution[6] = {524288, 524288, 524288, 524288, 262144, 262144};
    double axis_direction[6] = {-1, 1, -1, -1, -1, -1};
    double torque_axis_direction[6] = {-1, 1, -1, -1, -1, -1};
    bool enable_drive[6]={true,true,true,true,true,true};


    for(int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        driveObjectPtr[jnt_ctr]->enableDrive = enable_drive[jnt_ctr];
        driveObjectPtr[jnt_ctr]->actual_position = 0;
        driveObjectPtr[jnt_ctr]->actual_velocity = 0;
        driveObjectPtr[jnt_ctr]->actual_torque = 0;
        driveObjectPtr[jnt_ctr]->target_position = 0;
        driveObjectPtr[jnt_ctr]->target_velocity = 0;
        driveObjectPtr[jnt_ctr]->target_torque = 0;
        driveObjectPtr[jnt_ctr]->controlword = ControlWordValues::CW_DISABLE_VOLTAGE;
        driveObjectPtr[jnt_ctr]->statusword = StatusWordValues::SW_NOT_READY_TO_SWITCH_ON;
        driveObjectPtr[jnt_ctr]->mode_of_operation = OperationModeState::TORQUE_MODE;
        driveObjectPtr[jnt_ctr]->alias = 0;
        driveObjectPtr[jnt_ctr]->position = jnt_ctr;
        driveObjectPtr[jnt_ctr]->vendor_id = 0x000022d2;
        driveObjectPtr[jnt_ctr]->product_code = 0x00000301;
        jointDataPtr[jnt_ctr]->actual_position = 0;
        jointDataPtr[jnt_ctr]->actual_velocity = 0;
        jointDataPtr[jnt_ctr]->actual_torque = 0;
        jointDataPtr[jnt_ctr]->target_position = 0;
        jointDataPtr[jnt_ctr]->target_velocity = 0;
        jointDataPtr[jnt_ctr]->target_torque = 0;
        //enter motor info
        driveObjectPtr[jnt_ctr]->gear_ratio=gear_ratio[jnt_ctr];
        driveObjectPtr[jnt_ctr]->motor_rated_torque=motor_rated_torque[jnt_ctr];
        driveObjectPtr[jnt_ctr]->encoder_resolution=encoder_resolution[jnt_ctr];
        driveObjectPtr[jnt_ctr]->axis_direction=axis_direction[jnt_ctr];
        driveObjectPtr[jnt_ctr]->torque_axis_direction=torque_axis_direction[jnt_ctr];
    }

}

Logic::~Logic()
{
}

void Logic::stackPrefault()
{
    unsigned char dummy[MAX_SAFE_STACK];
    memset(dummy, 0, MAX_SAFE_STACK);
}

void Logic::cyclicTask()
{
    struct period_info pinfo;

    periodic_task_init(&pinfo);

    //separate thread for user input
    while (!exitFlag)
    {
        do_rt_task();
        wait_rest_of_period(&pinfo);
    }
}

void Logic::inc_period(struct period_info *pinfo)
{

    pinfo->next_period.tv_nsec += pinfo->period_ns;

    while (pinfo->next_period.tv_nsec >= 1000000000)
    {
        /* timespec nsec overflow */
        pinfo->next_period.tv_sec++;
        pinfo->next_period.tv_nsec -= 1000000000;
    }
}

void Logic::periodic_task_init(struct period_info *pinfo)
{
    /* for simplicity, hardcoding a 10ms period */
    pinfo->period_ns = 10000000;

    clock_gettime(CLOCK_MONOTONIC, &(pinfo->next_period));
}

void Logic::wait_rest_of_period(struct period_info *pinfo)
{
    inc_period(pinfo);

    /* for simplicity, ignoring possibilities of signal wakes */
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, NULL);
}

void Logic::do_rt_task()
{
    
    // switch (logicStateDataPtr->userInput)
    // {
    // case UserInputState::INIT:
    //     //already initailized
    //     break;
    // case UserInputState::START:
    //     //start the ethercat state 
    //     break;
    // case UserInputState::SWITCH_ON:
    //     //all drives are in swicth_on State
    //     break;    
    // case UserInputState::IDLE:
    //     //All drives opreational , but 
        
    //     if(logicStateDataPtr->allDriveReady)
    //     {
    //     checkStateLogic();
    //     }
    //     driveLogic.run();
    //     break;
    // case UserInputState::HOMING:
    //     //function for taking target values from system(user defined)
    //     if(logicStateDataPtr->allDriveReady)
    //     {
    //     checkStateLogic();
    //     }
    //     driveLogic.run();
    //     break;
    // case UserInputState::OPERATIONAL:
    //    //Taking target values from handcontroller
    //    if(logicStateDataPtr->allDriveReady)
    //     {
    //     checkStateLogic();
    //     }
    //     driveLogic.run();
    //     break;
    
    // default:
    //     break;
    // }
    if(logicStateDataPtr->allDriveReady)
        {
        checkStateLogic();
        }
    driveLogic.run();

    
}

void Logic::run()
{
    // Lock memory
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        fprintf(stderr, "Warning: Failed to lock memory: %s\n", strerror(errno));
        // Handle the error appropriately based on your application's requirements
    }

    stackPrefault();

    // Register signal handler to gracefully stop the program
    // signal(SIGINT, SafetyController::signalHandler);

    struct sched_param param = {};
    param.sched_priority = 49;


    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
    {
        perror("sched_setscheduler failed");
    }


    cyclicTask();

}

void Logic::checkStateLogic()
{

    // for (int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    // {
    //     if (fabs(jointDataPtr[jnt_ctr]->actual_position) > 10 * 3.14)
    //     {
    //         logicStateDataPtr->state = LogicState::NOT_OK;
    //         return;
    //     }
    // }

    logicStateDataPtr->state = LogicLimitCheck::OK;
}

// void Logic::userInput() {
//     while (true) {
//         int userInput;
//         std::cout << "Current state is : " <<std::endl;

//         switch (logicStateDataPtr->userInput)
//         {
//         case UserInputState::IDLE:
//             std::cout<<"IDLE MODE"<<std::endl;
//             break;
//         case UserInputState::OPERATIONAL:
//             std::cout<<"Operational"<<std::endl;
//             break;
//         default:
//             break;
//         }

//         std::cout<<"choose 1 IDLE"<<std::endl;
//         std::cout<<"choose 2 OPERATIONAL"<<std::endl;
//         std::cin >> userInput;

//         // Write to the shared memory
//         if(userInput==1)
//         {
//             logicStateDataPtr->userInput=UserInputState::IDLE;
//         }
//         else if(userInput==2)
//         {
//             logicStateDataPtr->userInput=UserInputState::OPERATIONAL;
//         }
//         // Optionally, break the loop if a specific input is given
//         if (userInput == -1) break;
//     }
// }

int main()
{
    Logic logicObj;
    logicObj.run();
}