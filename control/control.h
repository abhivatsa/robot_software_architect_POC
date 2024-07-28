#ifndef CONTROL_H
#define CONTROL_H

#include "robot_state.h"
#include "manipulator_control.h"
#include "actuator_control.h"
#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>
#include <bits/stdc++.h>
#include <sys/time.h>

class Control {
public:
    Control();
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
    // Other control-related methods

private:
    RobotState robotState;            // Shared state
    ManipulatorControl manipulatorControl;
    ActuatorControl actuatorControl;
};

#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is  \
                                     guranteed safe to access without \
                                     faulting */

volatile sig_atomic_t exitFlag = 0;


#endif // CONTROL_H
