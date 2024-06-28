#pragma once

#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>
#include <bits/stdc++.h>
#include <sys/time.h>
#include "actuator_control.cpp"

class control
{
private:
    ActuatorControl actuatorControl;
public:
    control(/* args */);
    ~control();
    void run();
    void stackPrefault();

     // Functions in cyclicTask.h
    struct period_info
    {
        struct timespec next_period;
        long period_ns;
    };

    void cyclicTask();
    static void inc_period(struct period_info *pinfo);
    static void periodic_task_init(struct period_info *pinfo);
    void do_rt_task();
    static void wait_rest_of_period(struct period_info *pinfo);
    
};

#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is  \
                                     guranteed safe to access without \
                                     faulting */

volatile sig_atomic_t exitFlag = 0;
