/*
 * RecursiveNewtonEuler.h
 *
 *  Created on: 31-Dec-2022
 *      Author: abhishek
 */

#ifndef MANIPULATOR_CONTROL_H
#define MANIPULATOR_CONTROL_H

#include "robot_state.h"
#include <iostream>
#include <math.h>
#include <array>
#include <eigen3/Eigen/Dense>
#include "../global/SharedObject.h"

class ManipulatorControl
{
public:
    // Constructor
    explicit ManipulatorControl(RobotState &state);

    // Public methods
    void updateKinematics();
    void updateDynamics();


private:
    // Private methods
    int computeTorque(const std::array<double, NUM_JOINTS>& joint_pos,
                      const std::array<double, NUM_JOINTS>& joint_vel,
                      const std::array<double, NUM_JOINTS>& joint_acc,
                      std::array<double, NUM_JOINTS>& joint_torque) noexcept;

    int computeTransformationMat(double joint_pos, int joint_num,
                                 Eigen::Matrix3d &rotation_mat, Eigen::Vector3d &pos_mat) const noexcept;

    // Member variables
    RobotState &robotState;

    std::array<double, 7> alpha;
    std::array<double, 7> a;
    std::array<double, 7> d;
    std::array<double, 7> theta;
    std::array<double, 7> m;
    std::array<Eigen::Vector3d, 7> pos_com;
    std::array<Eigen::Matrix3d, 7> inertia_com;
};

#endif // MANIPULATOR_CONTROL_H
