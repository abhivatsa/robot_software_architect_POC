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
#include "../global/SharedObject.h"

class userInput
{
    private:
    LogicStateData *logicStateDataPtr;

    void configureSharedMemory();
    void createSharedMemory(int &shm_fd, const char *name, int size);
    void* mapSharedMemory(int shm_fd, int size);

    public:
    userInput(/* args */);
    ~userInput();
    void run();


};
