#include "logicSharedMemory.h"

Logic::Logic(/* args */)
{
    configureSharedMemory();


    logicStateDataPtr->state = LogicState::NOT_EVAL;
    logicStateDataPtr->allDriveReady = false;
    fieldbusSharedDataPtr->allDrivesOpEnabled = false;
    fieldbusSharedDataPtr->operationMode = OperationModeState::POSITION_MODE;
    fieldbusSharedDataPtr->state = FieldbusState::INIT;

    for (int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        driveObjectPtr[jnt_ctr]->actual_position = 0;
        driveObjectPtr[jnt_ctr]->actual_velocity = 0;
        driveObjectPtr[jnt_ctr]->actual_torque = 0;
        driveObjectPtr[jnt_ctr]->target_position = 0;
        driveObjectPtr[jnt_ctr]->target_velocity = 0;
        driveObjectPtr[jnt_ctr]->target_torque = 0;
        driveObjectPtr[jnt_ctr]->controlword = ControlWordValues::CW_DISABLE_VOLTAGE;
        driveObjectPtr[jnt_ctr]->statusword = StatusWordValues::SW_NOT_READY_TO_SWITCH_ON;
        driveObjectPtr[jnt_ctr]->mode_of_operation = OperationModeState::POSITION_MODE;
        driveObjectPtr[jnt_ctr]->alias = 0;
        driveObjectPtr[jnt_ctr]->position = jnt_ctr;
        driveObjectPtr[jnt_ctr]->vendor_id = 0x0000029c;
        driveObjectPtr[jnt_ctr]->product_code = 0x03831002;
        jointDataPtr[jnt_ctr]->actual_position = 0;
        jointDataPtr[jnt_ctr]->actual_velocity = 0;
        jointDataPtr[jnt_ctr]->actual_torque = 0;
        jointDataPtr[jnt_ctr]->target_position = 0;
        jointDataPtr[jnt_ctr]->target_velocity = 0;
        jointDataPtr[jnt_ctr]->target_torque = 0;
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
    if (logicStateDataPtr->allDriveReady)
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

    for (int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        if (fabs(jointDataPtr[jnt_ctr]->actual_position) > 10 * 3.14)
        {
            logicStateDataPtr->state = LogicState::NOT_OK;
            return;
        }
    }

    logicStateDataPtr->state = LogicState::OK;
}

int main()
{
    Logic logicObj;
    logicObj.run();
}