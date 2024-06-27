#pragma once

#include "transistionState.h"

using namespace std;

void EthercatMaster::inc_period(struct period_info *pinfo)
{

    pinfo->next_period.tv_nsec += pinfo->period_ns;

    while (pinfo->next_period.tv_nsec >= 1000000000)
    {
        /* timespec nsec overflow */
        pinfo->next_period.tv_sec++;
        pinfo->next_period.tv_nsec -= 1000000000;
    }
}

void EthercatMaster::periodic_task_init(struct period_info *pinfo)
{
    /* for simplicity, hardcoding a 1ms period */
    pinfo->period_ns = 1000000;

    clock_gettime(CLOCK_MONOTONIC, &(pinfo->next_period));
}

void EthercatMaster::wait_rest_of_period(struct period_info *pinfo)
{
    inc_period(pinfo);

    /* for simplicity, ignoring possibilities of signal wakes */
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, NULL);
}

void EthercatMaster::cyclicTask()
{
    struct period_info pinfo;

    periodic_task_init(&pinfo);

    while (true)
    // while (!exitFlag)
    {
        ecrt_master_application_time(master, ((uint64_t)pinfo.next_period.tv_sec * 1000000000 + pinfo.next_period.tv_nsec));
        ecrt_master_receive(master);
        ecrt_domain_process(domain);
        checkDomainState();
        checkMasterState();
        do_rt_task();
        ecrt_domain_queue(domain);
        ecrt_master_send(master);
        wait_rest_of_period(&pinfo);

    }

    return;
}

void EthercatMaster::do_rt_task()
{
    switch (fieldbusSharedDataPtr->state)
    {
    case FieldbusState::NOT_READY:
        handleNotReadyState();
        break;
    case FieldbusState::READY:
        handleReadyState();
        break;
    case FieldbusState::OPERATIONAL:
        handleOperationalState();
        break;
    case FieldbusState::ERROR:
        handleErrorState();
        break;
    default:
        break;
    }
}

void EthercatMaster::handleNotReadyState()
{
    
}

void EthercatMaster::handleReadyState()
{
    
}

void EthercatMaster::handleOperationalState()
{
 
}


void EthercatMaster::handleErrorState()
{
    
}
