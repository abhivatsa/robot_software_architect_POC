#pragma once
#include "drive_logic.cpp"

volatile sig_atomic_t exitFlag = 0;
#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is  \
                                     guranteed safe to access without \
                                     faulting */

class Logic
{
private:
    DriveLogic driveLogic;
    ServoDrives *driveObjectPtr[NUM_JOINTS];
    EthercatStateData *fieldbusSharedDataPtr;
    LogicStateData *logicStateDataPtr;
    JointOutputData *jointDataPtr[NUM_JOINTS];

    void configureSharedMemory();
    void createSharedMemory(int &shm_fd, const char *name, int size);
    void* mapSharedMemory(int shm_fd, int size);

public:
    Logic(/* args */);
    ~Logic();
    void run();
    void checkStateLogic();
    // void checkApplicationLogic();

    struct period_info
    {
        struct timespec next_period;
        long period_ns;
    };

    void stackPrefault();
    void cyclicTask();
    static void inc_period(struct period_info *pinfo);
    static void periodic_task_init(struct period_info *pinfo);
    void do_rt_task();
    static void wait_rest_of_period(struct period_info *pinfo);

    void userInput();
    
};
