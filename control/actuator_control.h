#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H

#include "robot_state.h"
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
#include <math.h>
#include "../global/SharedObject.h"


class ActuatorControl {
public:
    ActuatorControl(RobotState& state);
    void communicateWithEthercat();
    void readDriveData();
    void writeDriveData();
    void initializeWriteData();
    // Other actuator-related methods

private:
    RobotState& robotState; // Reference to shared state
    JointParameters driveList[NUM_JOINTS];
    ServoDrives *driveObjectPtr[NUM_JOINTS];
    EthercatStateData *fieldbusSharedDataPtr;
    JointOutputData *jointDataPtr[NUM_JOINTS];

    void configureSharedMemory();
    void createSharedMemory(int &shm_fd, const char *name, int size);
    void* mapSharedMemory(int shm_fd, int size);

};

#endif // ACTUATOR_CONTROL_H