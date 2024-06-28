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
#include <math.h>
#include "SharedObject.h"

class ActuatorControl
{
public:
    ActuatorControl();
    ~ActuatorControl();
    void run();

private:
    JointParameters driveList[NUM_JOINTS];
    ServoDrives *driveObjectPtr[NUM_JOINTS];
    EthercatStateData *fieldbusSharedDataPtr;
    JointOutputData jointData[NUM_JOINTS];

    void configureSharedMemory();
    void createSharedMemory(int &shm_fd, const char *name, int size);
    void mapSharedMemory(void *&ptr, int shm_fd, int size);

    void readDriveData();
    void writeDriveData();
    void initializeWriteData();
};