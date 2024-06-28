#pragma once

#include <iostream>
#include <chrono>
#include <cstring>
#include <algorithm>
#include <thread>
#include <signal.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sched.h>
#include <stdbool.h>
#include <csignal>
#include <cstdlib>
#include <cstdint>
#include <ecrt.h>
#include "../global/SharedObject.h"

struct pdoDomainMapping
{
    unsigned int statusword;
    unsigned int position_actual_value;
    unsigned int velocity_actual_value;
    unsigned int torque_actual_value;
    unsigned int controlword;
    unsigned int modes_of_operation;
    unsigned int target_torque;
    unsigned int target_position;
    unsigned int target_velocity;
};



class EthercatMaster
{
public:
    EthercatMaster();
    ~EthercatMaster();
    void run();

private:
    ec_master_t *master;
    ec_master_state_t masterState;
    ec_domain_t *domain;
    ec_domain_state_t domainState;
    static uint8_t *domainPd;

    pdoDomainMapping driveOffset[NUM_JOINTS];

    ServoDrives *driveObjectPtr[NUM_JOINTS];
    EthercatStateData *fieldbusSharedDataPtr;

    void checkDomainState();
    void checkMasterState();

    void pdoMappingSlave(ec_slave_config_t *sc);

    void configureSharedMemory();
    void createSharedMemory(int &shm_fd, const char *name, int size);
    void mapSharedMemory(void *&ptr, int shm_fd, int size);
    // void initializeSharedData();

    void stackPrefault();

    // static void signalHandler(int signum);

        // Functions in transistionState.h
    StatusWordValues readDriveState(int joint_num);
    void transitionToState(ControlWordValues value, int jnt_ctr);

    // Functions in cyclicTask.h
    struct period_info
    {
        struct timespec next_period;
        long period_ns;
    };

    static void inc_period(struct period_info *pinfo);
    static void periodic_task_init(struct period_info *pinfo);
    static void wait_rest_of_period(struct period_info *pinfo);
    void cyclicTask();
    void do_rt_task();

    void handleNotReadyState();
    void handleReadyState();
    void handleOperationalState();
    void handleErrorState();

};

uint8_t *EthercatMaster::domainPd = NULL;



// Example definition, replace it with your actual slave configuration

#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is  \
                                     guranteed safe to access without \
                                     faulting */

volatile sig_atomic_t exitFlag = 0;
