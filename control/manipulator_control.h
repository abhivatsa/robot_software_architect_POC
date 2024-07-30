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
#include <eigen3/Eigen/Dense>
#include <vector>

class ManipulatorControl {
public:
    ManipulatorControl(RobotState& state);
    void updateKinematics();
    void updateDynamics();
    int computeTorque(std::vector<double> joint_pos, std::vector<double> joint_vel, std::vector<double> joint_acc,
			std::vector<double>& joint_torque);
	int computeTransformationMat(double joint_pos, int joint_num, Eigen::Matrix3d& rotation_mat,
			Eigen::Vector3d& pos_mat);
    // Other methods related to kinematics and dynamics

private:
    RobotState& robotState; // Reference to shared state
    std::vector<double> alpha, a, d, theta, m;
	std::vector<Eigen::Vector3d> pos_com;
	std::vector<Eigen::Matrix3d> inertia_com;
};

#endif // MANIPULATOR_CONTROL_H

// #pragma once


// #include <iostream>
// #include <math.h>
// #include <eigen3/Eigen/Dense>
// #include <vector>

// class RecursiveNewtonEuler {
// public:
// 	RecursiveNewtonEuler();
// 	virtual ~RecursiveNewtonEuler();
// 	int computeTorque(std::vector<double> joint_pos, std::vector<double> joint_vel, std::vector<double> joint_acc,
// 			std::vector<double>& joint_torque);
// 	int computeTransformationMat(double joint_pos, int joint_num, Eigen::Matrix3d& rotation_mat,
// 			Eigen::Vector3d& pos_mat);
// private:
// 	std::vector<double> alpha, a, d, theta, m;
// 	std::vector<Eigen::Vector3d> pos_com;
// 	std::vector<Eigen::Matrix3d> inertia_com;

// };