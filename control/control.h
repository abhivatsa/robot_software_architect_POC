#pragma once

#include "actuator_control.h"

class control
{
private:
    ActuatorControl actuatorControl;
public:
    control(/* args */);
    ~control();
    void run();
};