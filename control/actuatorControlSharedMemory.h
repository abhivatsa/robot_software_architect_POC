#pragma once

#include "actuator_control.h"

void ActuatorControl::configureSharedMemory()
{
    int shm_fd_driveObject;
    int shm_fd_fieldbusSharedData;
    int shm_fd_jointSharedData;

    createSharedMemory(shm_fd_driveObject, "ServoDrivesData", NUM_JOINTS*sizeof(ServoDrives));
    createSharedMemory(shm_fd_fieldbusSharedData, "EthercatStateData", sizeof(EthercatStateData));
    createSharedMemory(shm_fd_jointSharedData, "JointOutputData", sizeof(JointOutputData));

    mapSharedMemory((void *&)fieldbusSharedDataPtr, shm_fd_fieldbusSharedData, sizeof(EthercatStateData));
    mapSharedMemory((void *&)jointDataPtr, shm_fd_jointSharedData, sizeof(JointOutputData));

    for (int i = 0; i < NUM_JOINTS; ++i) {
        mapSharedMemory((void *&)driveObjectPtr[i], shm_fd_driveObject, sizeof(ServoDrives));
    }

    // initializeSharedData();
}

void ActuatorControl::createSharedMemory(int &shm_fd, const char *name, int size)
{
    shm_fd = shm_open(name, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1)
    {
        throw std::runtime_error("Failed to create shared memory object.");
    }
    ftruncate(shm_fd, size);
}

void ActuatorControl::mapSharedMemory(void *&ptr, int shm_fd, int size)
{
    ptr = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (ptr == MAP_FAILED)
    {
        throw std::runtime_error("Failed to map shared memory.");
    }
}

// void EthercatMaster::initializeSharedData()
// {
//     jointDataPtr->setZero();
//     systemStateDataPtr->setZero();
// }