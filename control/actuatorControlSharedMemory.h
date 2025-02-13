#pragma once

#include "actuator_control.h"

void ActuatorControl::configureSharedMemory()
{
    int shm_fd_driveObject;
    int shm_fd_fieldbusSharedData;
    int shm_fd_jointSharedData;

    createSharedMemory(shm_fd_driveObject, "ServoDrivesData", NUM_JOINTS*sizeof(ServoDrives));
    createSharedMemory(shm_fd_fieldbusSharedData, "EthercatStateData", sizeof(EthercatStateData));
    createSharedMemory(shm_fd_jointSharedData, "JointOutputData", NUM_JOINTS*sizeof(JointOutputData));

    fieldbusSharedDataPtr = static_cast<EthercatStateData*>(mapSharedMemory(shm_fd_fieldbusSharedData, sizeof(EthercatStateData)));
    ServoDrives* driveObjectBasePtr = static_cast<ServoDrives*>(mapSharedMemory(shm_fd_driveObject, NUM_JOINTS * sizeof(ServoDrives)));
    JointOutputData* jointDataBasePtr = static_cast<JointOutputData*>(mapSharedMemory(shm_fd_jointSharedData, NUM_JOINTS * sizeof(JointOutputData)));

    for (int i = 0; i < NUM_JOINTS; ++i) {
        driveObjectPtr[i] = &driveObjectBasePtr[i];
        jointDataPtr[i] = &jointDataBasePtr[i];
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

void* ActuatorControl::mapSharedMemory(int shm_fd, int size) {
    void* ptr = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (ptr == MAP_FAILED) {
        throw std::runtime_error("Failed to map shared memory.");
    }
    return ptr;
}



// void EthercatMaster::initializeSharedData()
// {
//     jointDataPtr->setZero();
//     systemStateDataPtr->setZero();
// }