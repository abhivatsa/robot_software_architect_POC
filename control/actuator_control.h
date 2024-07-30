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
#include <vector>
#include "../global/SharedObject.h"


class ActuatorControl {
public:
    ActuatorControl(RobotState& state);
    void communicateWithEthercat();
    void readDriveData();
    void writeDriveData();
    void initializeWriteData();
    void frictionModel(std::vector<double> &friction_torque);
    // Other actuator-related methods

private:
    RobotState& robotState; // Reference to shared state

    ServoDrives *driveObjectPtr[NUM_JOINTS];
    EthercatStateData *fieldbusSharedDataPtr;
    JointOutputData *jointDataPtr[NUM_JOINTS];
    // std::vector<double> posFrictionCoff;
    std::array< std::array<double, 4>, NUM_JOINTS> posFrictionCoff;
    std::array< std::array<double, 4>, NUM_JOINTS> negFrictionCoff;
    // double posFrictionCoff[NUM_JOINTS][4];
    // double negFrictionCoff[NUM_JOINTS][4];


    void configureSharedMemory();
    void createSharedMemory(int &shm_fd, const char *name, int size);
    void* mapSharedMemory(int shm_fd, int size);

};

#endif // ACTUATOR_CONTROL_H