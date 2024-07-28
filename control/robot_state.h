#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <array>

struct RobotState
{
    std::array<double, 6> actualJointAngles;     // Joint angles for the 6-axis robotic arm
    std::array<double, 6> actualJointVelocities; // Joint velocities for the 6-axis robotic arm
    std::array<double, 6> actualJointTorques;    // Joint velocities for the 6-axis robotic arm

    std::array<double, 6> targetJointAngles;     // Joint angles for the 6-axis robotic arm
    std::array<double, 6> targetJointVelocities; // Joint velocities for the 6-axis robotic arm
    std::array<double, 6> targetJointTorques;    // Joint velocities for the 6-axis robotic arm
    // Add other shared variables here if needed
};

#endif // ROBOT_STATE_H