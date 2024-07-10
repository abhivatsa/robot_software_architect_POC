#pragma once

#include "master.h"

void EthercatMaster::configureSharedMemory()
{
    int shm_fd_driveObject;
    int shm_fd_fieldbusSharedData;

    createSharedMemory(shm_fd_driveObject, "ServoDrivesData", NUM_JOINTS*sizeof(ServoDrives));
    createSharedMemory(shm_fd_fieldbusSharedData, "EthercatStateData", sizeof(EthercatStateData));

    fieldbusSharedDataPtr = static_cast<EthercatStateData*>(mapSharedMemory(shm_fd_fieldbusSharedData, sizeof(EthercatStateData)));
    ServoDrives* driveObjectBasePtr = static_cast<ServoDrives*>(mapSharedMemory(shm_fd_driveObject, NUM_JOINTS * sizeof(ServoDrives)));

    for (int i = 0; i < NUM_JOINTS; ++i) {
        driveObjectPtr[i] = &driveObjectBasePtr[i];
    }

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

void* EthercatMaster::mapSharedMemory(int shm_fd, int size) {
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