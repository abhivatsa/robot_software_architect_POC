#pragma once

#include "master.h"

void EthercatMaster::configureSharedMemory()
{
    int shm_fd_driveObject;
    int shm_fd_fieldbusSharedData;

    createSharedMemory(shm_fd_driveObject, "ServoDrivesData", NUM_JOINTS*sizeof(ServoDrives));
    createSharedMemory(shm_fd_fieldbusSharedData, "EthercatStateData", sizeof(EthercatStateData));

    mapSharedMemory((void *&)fieldbusSharedDataPtr, shm_fd_fieldbusSharedData, sizeof(EthercatStateData));

    for (int i = 0; i < NUM_JOINTS; i++) {
        mapSharedMemory((void *&)driveObjectPtr[i], shm_fd_driveObject, sizeof(ServoDrives));
    }

    // initializeSharedData();
}

void EthercatMaster::createSharedMemory(int &shm_fd, const char *name, int size)
{
    shm_fd = shm_open(name, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1)
    {
        throw std::runtime_error("Failed to create shared memory object.");
    }
    ftruncate(shm_fd, size);
}

void EthercatMaster::mapSharedMemory(void *&ptr, int shm_fd, int size)
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