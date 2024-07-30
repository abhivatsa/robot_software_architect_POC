#include "manipulator_control.h"

ManipulatorControl::ManipulatorControl(RobotState &state)
    : robotState(state)
{
    // Initialization code

    alpha.resize(7);
    a.resize(7);
    d.resize(7);
    theta.resize(7);
    m.resize(7);
    pos_com.resize(7);
    inertia_com.resize(7);

    double d_off1, d_off2, l0, l1, l2, l3;

    d_off1 = 0.06;
    d_off2 = 0.07;
    l0 = 0.296;
    l1 = 0.442;
    l2 = 0.408;
    l3 = 0.206;

    /* Inline */
    alpha = {0, -M_PI/2, M_PI/2, -M_PI/2, M_PI/2, -M_PI/2, M_PI/2};
    a = {0, 0, 0, d_off1, -d_off2, 0, 0};
    d = {l0, 0, l1, 0, l2, 0, l3};
    theta = {0, 0, 0, 0, 0, 0, 0};

    m = {5.357, 5.268, 3.6714, 3.4606, 0.8253, 3.5134, 0.869};

    pos_com[0] << 0.000008, 0.000026, 0.100712;
    pos_com[1] << -0.0000133, -0.01105, -0.00954;
    pos_com[2] << -0.000121, -0.20089, -0.01227;
    pos_com[3] << -0.006485, 0.01121588, -0.01131;
    pos_com[4] << -0.02844, -0.041345, 0.0423812;
    pos_com[5] << -0.000054, 0.00484, -0.113126;
    pos_com[6] << -0.0002455, -0.05731, 0.00387;
    //	pos_com[5] << 0, 0, 0;

    inertia_com[0].setZero(3, 3);
    inertia_com[1].setZero(3, 3);
    inertia_com[2].setZero(3, 3);
    inertia_com[3].setZero(3, 3);
    inertia_com[4].setZero(3, 3);
    inertia_com[5].setZero(3, 3);
    inertia_com[6].setZero(3, 3);
}

void ManipulatorControl::updateKinematics()
{
    // Example: Calculate new joint angles and velocities
    // This is just a placeholder for the actual kinematics calculations
    // for (size_t i = 0; i < 6; ++i) {
    //     robotState.jointAngles[i] += 0.1; // Update angles
    //     robotState.jointVelocities[i] = 1.0; // Example velocity
    // }
}

void ManipulatorControl::updateDynamics()
{
    // Example: Update dynamics based on the current joint angles and velocities

    // std::cout<<"Line 65 ud"<<std::endl;
    std::vector<double> joint_pos, joint_vel, joint_acc, joint_torq;

    // std::cout<<"Line 68 ud"<<std::endl;

    joint_pos.resize(7, 0.0);
    joint_vel.resize(7, 0.0);
    joint_acc.resize(7, 0.0);
    joint_torq.resize(7, 0.0);

    double gravity_coeff[7] = {0.9, 0.85, 0.8, 0.9, 0.9, 0.6, 0.8};

    // std::cout<<"Line 75 ud"<<std::endl;

    for (int jnt_ctr = 0; jnt_ctr < 7; jnt_ctr++)
    {
        
        joint_pos[jnt_ctr] = robotState.actualJointAngles[jnt_ctr];
        joint_vel[jnt_ctr] = 0;
        // joint_vel[jnt_ctr] = robotState.actualJointVelocities[jnt_ctr];
    }

    // std::cout<<"pos j1 : "<<joint_pos[0]<<", j2 : "<<joint_pos[1]<<", j3 : "<<joint_pos[2]<<", j4 : "<<joint_pos[3]<<", j5 : "<<joint_pos[4]<<", j6 : "<<joint_pos[5]<<", j7 : "<<joint_pos[6]<<std::endl;
    // std::cout<<"vel j1 : "<<joint_vel[0]<<", j2 : "<<joint_vel[1]<<", j3 : "<<joint_vel[2]<<", j4 : "<<joint_vel[3]<<", j5 : "<<joint_vel[4]<<", j6 : "<<joint_vel[5]<<", j7 : "<<joint_vel[6]<<std::endl;


    // joint_pos = {0, M_PI/2, M_PI/2,M_PI/2,0,0,0};

    // std::cout<<"Line 83 ud"<<std::endl;

    // std::cout<<"joint_pos 2 : "<<joint_pos[1]<<" joint vel : "<<joint_vel[1]<<std::endl;

    // std::cout<<"Line 87 ud"<<std::endl;

    if (computeTorque(joint_pos, joint_vel, joint_acc, joint_torq) == 0)
    {

        // std::cout<<"Line 88 ud"<<std::endl;

        for (int jnt_ctr = 0; jnt_ctr < 7; jnt_ctr++)
        {
            robotState.targetJointTorques[jnt_ctr] = gravity_coeff[jnt_ctr]*joint_torq[jnt_ctr];
            
        }

        // std::cout<<"joint_torq 2 : "<<joint_torq[1]<<", joint_torq 4 : "<<joint_torq[3]<<", joint_torq 6 : "<<joint_torq[5]<<", joint_pos 2 : "<<joint_pos[1]<<" joint vel : "<<joint_vel[1]<<std::endl;

        

        // std::cout<<"Line 95 ud"<<std::endl;
    }
}

int ManipulatorControl::computeTorque(std::vector<double> joint_pos, std::vector<double> joint_vel, std::vector<double> joint_acc,
                                      std::vector<double> &joint_torque)
{

    // std::cout<<"line 99"<<std::endl;
    Eigen::Matrix3d rotation_mat;
    Eigen::Vector3d pos_vec, z_dir;
    // std::cout<<"line 103"<<std::endl;

    // joint_torque.resize(joint_pos.sizejoint_pos());

    z_dir << 0, 0, 1;

    // std::cout<<"line 109"<<std::endl;

    std::vector<Eigen::Vector3d> force_com, torque_com;

    force_com.resize(8);
    torque_com.resize(8);

    force_com[0].setZero(3);
    torque_com[0].setZero(3);

    // std::cout<<"line 119"<<std::endl;

    Eigen::Vector3d omega, omega_next, omega_dot, omega_dot_next, vel_dot, vel_dot_next;

    omega.setZero(3);
    omega_dot.setZero(3);
    vel_dot.setZero(3);
    vel_dot.z() = -10;

    // std::cout<<"line 128"<<std::endl;

    for (unsigned int joint_ctr = 0; joint_ctr < joint_pos.size(); joint_ctr++)
    {

        // std::cout<<"line 133"<<std::endl;<<",

        // std::cout<<"joint_pos : "<<joint_pos[joint_ctr]<<std::endl;

        int solve_chk = computeTransformationMat(joint_pos[joint_ctr], joint_ctr, rotation_mat, pos_vec);

        // std::cout<<"rotation_mat : \n"<<rotation_mat<<std::endl;

        // std::cout<<"line 137"<<std::endl;

        if (solve_chk == 0)
        {

            // std::cout<<"line 142"<<std::endl;

            omega_next = rotation_mat * omega + joint_vel[joint_ctr] * z_dir;

            omega_dot_next = rotation_mat * omega_dot + rotation_mat * (omega.cross(joint_vel[joint_ctr] * z_dir)) + joint_acc[joint_ctr] * z_dir;

            vel_dot_next = rotation_mat * (omega_dot.cross(pos_vec) + omega.cross(omega.cross(pos_vec)) + vel_dot);

            Eigen::Vector3d vel_dot_com = omega_dot_next.cross(pos_com[joint_ctr]) + omega_next.cross(omega_next.cross(pos_com[joint_ctr])) + vel_dot_next;


            force_com[joint_ctr + 1] = m[joint_ctr] * vel_dot_com;
            torque_com[joint_ctr + 1] = inertia_com[joint_ctr] * omega_dot_next + omega_next.cross(inertia_com[joint_ctr] * omega_next);

            omega = omega_next;
            omega_dot = omega_dot_next;
            vel_dot = vel_dot_next;

            // if (joint_ctr == 1){

            //     std::cout<<"rotation_mat : \n"<<rotation_mat<<std::endl;

            // }
            // std::cout<<"line 159"<<std::endl;
        }
    }

    // std::cout<<"line 163"<<std::endl;

    Eigen::Vector3d force, force_prev, torque, torque_prev;

    force_prev.setZero(3);
    torque_prev.setZero(3);
            // std::cout<<"target torq : "<<(-1*driveObjectPtr[jnt_ctr]->axis_direction*robotState.targetJointTorques[jnt_ctr]* 1000 / (driveObjectPtr[jnt_ctr]->gear_ratio * driveObjectPtr[jnt_ctr]->motor_rated_torque))<<std::endl;
            // std::cout<<"act pos :" <<jnt_ctr<<" : "<<jointDataPtr[jnt_ctr]->actual_position<<std::endl;
    for (unsigned int joint_ctr = joint_pos.size(); joint_ctr > 0; joint_ctr--)
    {
        // std::cout<<"line 172"<<std::endl;
        int solve_chk = computeTransformationMat(joint_pos[joint_ctr], joint_ctr, rotation_mat, pos_vec);

        // std::cout<<"line 175"<<std::endl;

        if (solve_chk == 0)
        {
            force = rotation_mat.transpose() * force_prev + force_com[joint_ctr];
            torque = torque_com[joint_ctr] + rotation_mat.transpose() * torque_prev + pos_com[joint_ctr].cross(force_com[joint_ctr]) + pos_vec.cross(rotation_mat.transpose()* force_prev);

            force_prev = force;
            torque_prev = torque;

            // std::cout<<"jnt_ctr : "<<joint_ctr<<", force : x "<<force(0)<<", y : "<<force(1)<<", z : "<<force(2)<<", torque : x "<<torque(0)<<", y : "<<torque(1)<<", z : "<<torque(2)<<std::endl;
        }

        // std::cout<<"line 186"<<std::endl;

        joint_torque[joint_ctr - 1] = torque[2];

        // joint_torque[joint_ctr - 1] = 2;

        // std::cout<<"joint_ctr - 1 : "<<joint_ctr - 1<<" , "<<torque[2]<<std::endl;
    }

    // std::cout<<"line 201"<<std::endl;
    return 0;

}

int ManipulatorControl::computeTransformationMat(double joint_pos, int joint_num, Eigen::Matrix3d &rotation_mat,
                                                 Eigen::Vector3d &pos_mat)
{



    rotation_mat.setIdentity(3, 3);
    pos_mat.setZero(3);

    rotation_mat << cos(joint_pos + theta[joint_num]), -sin(joint_pos + theta[joint_num]), 0,
        sin(joint_pos + theta[joint_num]) * cos(alpha[joint_num]), cos(joint_pos + theta[joint_num]) * cos(alpha[joint_num]), -sin(alpha[joint_num]),
        sin(joint_pos + theta[joint_num]) * sin(alpha[joint_num]), cos(joint_pos + theta[joint_num]) * sin(alpha[joint_num]), cos(alpha[joint_num]);

    pos_mat << a[joint_num], -sin(alpha[joint_num]) * d[joint_num], cos(alpha[joint_num]) * d[joint_num];

    Eigen::Matrix3d rotation_mat1 = rotation_mat.transpose();

    rotation_mat = rotation_mat1;

    return 0;
}
