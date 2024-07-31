#include "manipulator_control.h"

ManipulatorControl::ManipulatorControl(RobotState &state)
    : robotState(state),
      alpha{0, -M_PI/2, M_PI/2, -M_PI/2, M_PI/2, -M_PI/2, M_PI/2},
      a{0, 0, 0, 0.06, -0.07, 0, 0},
      d{0.296, 0, 0.442, 0, 0.408, 0, 0.206},
      theta{0, 0, 0, 0, 0, 0, 0},
      m{5.357, 5.268, 3.6714, 3.4606, 0.8253, 3.5134, 0.869},
      pos_com{
          Eigen::Vector3d(0.000008, 0.000026, 0.100712),
          Eigen::Vector3d(-0.0000133, -0.01105, -0.00954),
          Eigen::Vector3d(-0.000121, -0.20089, -0.01227),
          Eigen::Vector3d(-0.006485, 0.01121588, -0.01131),
          Eigen::Vector3d(-0.02844, -0.041345, 0.0423812),
          Eigen::Vector3d(-0.000054, 0.00484, -0.113126),
          Eigen::Vector3d(-0.0002455, -0.05731, 0.00387)
      },
      inertia_com{}
{
    // Initialize inertia_com with zero matrices
    for (auto &I : inertia_com)
        I.setZero();
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
    std::array<double, NUM_JOINTS> joint_pos, joint_vel, joint_acc, joint_torq;

    joint_pos.fill(0.0);
    joint_vel.fill(0.0);
    joint_acc.fill(0.0);
    joint_torq.fill(0.0);

    double gravity_coeff[7] = {0.9, 0.85, 0.8, 0.9, 0.9, 0.6, 0.8};

    // std::cout<<"Line 75 ud"<<std::endl;

    for (int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        
        joint_pos[jnt_ctr] = robotState.actualJointAngles[jnt_ctr];
        // joint_vel[jnt_ctr] = robotState.actualJointVelocities[jnt_ctr];
    }

    if (computeTorque(joint_pos, joint_vel, joint_acc, joint_torq) == 0)
    {
        for (int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
        {
            robotState.targetJointTorques[jnt_ctr] = gravity_coeff[jnt_ctr]*joint_torq[jnt_ctr];       
        }

    }
}

int ManipulatorControl::computeTorque(const std::array<double, NUM_JOINTS>& joint_pos,
                      const std::array<double, NUM_JOINTS>& joint_vel,
                      const std::array<double, NUM_JOINTS>& joint_acc,
                      std::array<double, NUM_JOINTS>& joint_torque) noexcept
{

    Eigen::Matrix3d rotation_mat;
    Eigen::Vector3d pos_vec, z_dir;

    z_dir << 0, 0, 1;

    std::vector<Eigen::Vector3d> force_com, torque_com;

    force_com.resize(8);
    torque_com.resize(8);

    force_com[0].setZero(3);
    torque_com[0].setZero(3);

    Eigen::Vector3d omega, omega_next, omega_dot, omega_dot_next, vel_dot, vel_dot_next;

    omega.setZero(3);
    omega_dot.setZero(3);
    vel_dot.setZero(3);
    vel_dot.z() = -10;

    for (unsigned int joint_ctr = 0; joint_ctr < joint_pos.size(); joint_ctr++)
    {

        int solve_chk = computeTransformationMat(joint_pos[joint_ctr], joint_ctr, rotation_mat, pos_vec);

        if (solve_chk == 0)
        {

            omega_next = rotation_mat * omega + joint_vel[joint_ctr] * z_dir;

            omega_dot_next = rotation_mat * omega_dot + rotation_mat * (omega.cross(joint_vel[joint_ctr] * z_dir)) + joint_acc[joint_ctr] * z_dir;

            vel_dot_next = rotation_mat * (omega_dot.cross(pos_vec) + omega.cross(omega.cross(pos_vec)) + vel_dot);

            Eigen::Vector3d vel_dot_com = omega_dot_next.cross(pos_com[joint_ctr]) + omega_next.cross(omega_next.cross(pos_com[joint_ctr])) + vel_dot_next;


            force_com[joint_ctr + 1] = m[joint_ctr] * vel_dot_com;
            torque_com[joint_ctr + 1] = inertia_com[joint_ctr] * omega_dot_next + omega_next.cross(inertia_com[joint_ctr] * omega_next);

            omega = omega_next;
            omega_dot = omega_dot_next;
            vel_dot = vel_dot_next;

        }
    }

    Eigen::Vector3d force, force_prev, torque, torque_prev;

    force_prev.setZero(3);
    torque_prev.setZero(3);

    for (unsigned int joint_ctr = joint_pos.size(); joint_ctr > 0; joint_ctr--)
    {
        int solve_chk = computeTransformationMat(joint_pos[joint_ctr], joint_ctr, rotation_mat, pos_vec);

        if (solve_chk == 0)
        {
            force = rotation_mat.transpose() * force_prev + force_com[joint_ctr];
            torque = torque_com[joint_ctr] + rotation_mat.transpose() * torque_prev + pos_com[joint_ctr].cross(force_com[joint_ctr]) + pos_vec.cross(rotation_mat.transpose()* force_prev);

            force_prev = force;
            torque_prev = torque;
        }

        joint_torque[joint_ctr - 1] = torque[2];

    }

    return 0;

}

int ManipulatorControl::computeTransformationMat(double joint_pos, int joint_num,
                                 Eigen::Matrix3d &rotation_mat, Eigen::Vector3d &pos_mat) const noexcept
{

    rotation_mat.setIdentity(3, 3);
    pos_mat.setZero(3);

    double cos_theta = cos(joint_pos + theta[joint_num]);
    double sin_theta = sin(joint_pos + theta[joint_num]);

    rotation_mat << cos_theta, -sin_theta, 0,
                    sin_theta * cos(alpha[joint_num]), cos_theta * cos(alpha[joint_num]), -sin(alpha[joint_num]),
                    sin_theta * sin(alpha[joint_num]), cos_theta * sin(alpha[joint_num]), cos(alpha[joint_num]);


    pos_mat << a[joint_num], -sin(alpha[joint_num]) * d[joint_num], cos(alpha[joint_num]) * d[joint_num];

    Eigen::Matrix3d rotation_mat1 = rotation_mat.transpose();

    rotation_mat = rotation_mat1;

    return 0;
}
