#include "userInput.h"

void userInput::configureSharedMemory()
{
     int shm_fd_logicSharedData;

     createSharedMemory(shm_fd_logicSharedData, "LogicStateData", sizeof(LogicStateData));

     logicStateDataPtr = static_cast<LogicStateData*>(mapSharedMemory(shm_fd_logicSharedData, sizeof(LogicStateData)));


}
void userInput::createSharedMemory(int &shm_fd, const char *name, int size)
{
    shm_fd = shm_open(name, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1)
    {
        throw std::runtime_error("Failed to create shared memory object.");
    }
    ftruncate(shm_fd, size);
}

void* userInput::mapSharedMemory(int shm_fd, int size) {
    void* ptr = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (ptr == MAP_FAILED) {
        throw std::runtime_error("Failed to map shared memory.");
    }
    return ptr;
}
