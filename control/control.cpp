#include "control.h"

control::control(/* args */)
{
}

control::~control()
{
}

void control::cyclicTask()
{
    struct period_info pinfo;

    periodic_task_init(&pinfo);

    while (!exitFlag)
    {
        do_rt_task();
        wait_rest_of_period(&pinfo);
    }
}

void control::inc_period(struct period_info *pinfo)
{

    pinfo->next_period.tv_nsec += pinfo->period_ns;

    while (pinfo->next_period.tv_nsec >= 1000000000)
    {
        /* timespec nsec overflow */
        pinfo->next_period.tv_sec++;
        pinfo->next_period.tv_nsec -= 1000000000;
    }
}

void control::periodic_task_init(struct period_info *pinfo)
{
    /* for simplicity, hardcoding a 1ms period */
    pinfo->period_ns = 1000000;

    clock_gettime(CLOCK_MONOTONIC, &(pinfo->next_period));
}

void control::wait_rest_of_period(struct period_info *pinfo)
{
    inc_period(pinfo);

    /* for simplicity, ignoring possibilities of signal wakes */
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, NULL);
}

void control::do_rt_task()
{
    actuatorControl.run();
}

void control::run(){

    // Set CPU affinity for real-time thread
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(1, &cpuset); // Set to the desired CPU core

    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) == -1)
    {
        perror("Error setting CPU affinity");
        // Handle the error appropriately based on your application's requirements
    }

    // Lock memory
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        fprintf(stderr, "Warning: Failed to lock memory: %s\n", strerror(errno));
        // Handle the error appropriately based on your application's requirements
    }

    stackPrefault();

    // Register signal handler to gracefully stop the program
    // signal(SIGINT, SafetyController::signalHandler);

    struct sched_param param = {};
    param.sched_priority = 49;

    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
    {
        perror("sched_setscheduler failed");
    }

    cyclicTask();
    
    // while (true){
    //     actuatorControl.run();
    // }
    

}

void control::stackPrefault()
{
    unsigned char dummy[MAX_SAFE_STACK];
    memset(dummy, 0, MAX_SAFE_STACK);
}

int main(){

    control controlobj;
    controlobj.run();

}