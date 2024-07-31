#include "actuatorControlSharedMemory.h"

ActuatorControl::ActuatorControl(RobotState &state)
    : robotState(state)
{
    configureSharedMemory();

    initializeFrictionCoefficients();
    // Initialization code
    
}

void ActuatorControl::communicateWithEthercat()
{

    switch (fieldbusSharedDataPtr->state)
    {
    case FieldbusState::NOT_READY:
        /* code */
        break;
    case FieldbusState::READY:
        readDriveData();
        initializeWriteData();
        break;
    case FieldbusState::OPERATIONAL:
        readDriveData();
        writeDriveData();
        break;
    case FieldbusState::ERROR:
        /* code */
        break;

    default:
        break;
    }
}

void ActuatorControl::readDriveData()
{

    for (unsigned int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        auto &joint = *jointDataPtr[jnt_ctr];
        auto &drive = *driveObjectPtr[jnt_ctr];

        joint.actual_position = drive.axis_direction * (drive.actual_position * 2 * M_PI) / drive.encoder_resolution;
        joint.actual_velocity = drive.axis_direction * (drive.actual_velocity * 2 * M_PI) / 60000;
        joint.actual_torque = drive.axis_direction * (drive.actual_torque * drive.motor_rated_torque * drive.gear_ratio) / 1000;

        robotState.actualJointAngles[jnt_ctr] = joint.actual_position;
        robotState.actualJointVelocities[jnt_ctr] = joint.actual_velocity;
        robotState.actualJointTorques[jnt_ctr] = joint.actual_torque;
    }
}

void ActuatorControl::writeDriveData()
{

    std::array<double, NUM_JOINTS> fric_torq;
    frictionModel(fric_torq);

    for (unsigned int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        auto &drive = *driveObjectPtr[jnt_ctr];
        auto &joint = *jointDataPtr[jnt_ctr];
        
        switch (fieldbusSharedDataPtr->operationMode)
        {
        case OperationModeState::POSITION_MODE:
            drive.target_position = (int32_t)(drive.axis_direction * joint.target_position / (2 * M_PI) * drive.encoder_resolution);
            break;
        case OperationModeState::VELOCITY_MODE:
            // drive.target_velocity = (int32_t)(driveObjectPtr[jnt_ctr]->axis_direction * jointDataPtr[jnt_ctr]->target_position * 60 / (2 * M_PI));
            // Need to Fix
            break;
        case OperationModeState::TORQUE_MODE:
            drive.target_torque = (int16_t)((-1 * drive.torque_axis_direction * robotState.targetJointTorques[jnt_ctr] * 1000 / (drive.gear_ratio * drive.motor_rated_torque)) + fric_torq[jnt_ctr]);
            break;
        default:
            break;
        }
    }
}

void ActuatorControl::initializeWriteData()
{

    for (unsigned int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        driveObjectPtr[jnt_ctr]->target_position = driveObjectPtr[jnt_ctr]->actual_position;
        driveObjectPtr[jnt_ctr]->target_velocity = 0;
        driveObjectPtr[jnt_ctr]->target_torque = 0;
    }
}

void ActuatorControl::frictionModel(std::array<double, NUM_JOINTS> &friction_torque)
{
    friction_torque.fill(0.0);

    for (int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        int vel = driveObjectPtr[jnt_ctr]->actual_velocity;

        if (vel > VELOCITY_THRESHOLD) {
            vel = std::min(vel, VELOCITY_MAX);
            friction_torque[jnt_ctr] = posFrictionCoff[jnt_ctr][0] + posFrictionCoff[jnt_ctr][1] * vel + posFrictionCoff[jnt_ctr][2] * vel * vel + posFrictionCoff[jnt_ctr][3] * vel * vel * vel;
        } else if (vel < -VELOCITY_THRESHOLD) {
            vel = std::max(vel, -VELOCITY_MAX);
            friction_torque[jnt_ctr] = negFrictionCoff[jnt_ctr][0] + negFrictionCoff[jnt_ctr][1] * vel + negFrictionCoff[jnt_ctr][2] * vel * vel + negFrictionCoff[jnt_ctr][3] * vel * vel * vel;
        }
    }
}

void ActuatorControl::initializeFrictionCoefficients() {
    posFrictionCoff[0] = {1.8353e+01, 1.9756e-02, -1.9185e-06, 7.1482e-11};
    posFrictionCoff[1] = {1.8353e+01, 1.9756e-02, -1.9185e-06, 7.1482e-11};
    posFrictionCoff[2] = {13.89006355525087*0.8, 0.01257823874029, -0.00000160346214, 0.00000000013339};
    posFrictionCoff[3] = {13.89006355525087, 0.01257823874029, -0.00000160346214, 0.00000000013339};
    posFrictionCoff[4] = {28.17743436155735*0.7, 0.01382548863850, -0.00000231961904, 0.00000000022080};
    posFrictionCoff[5] = {28.17743436155735*0.7, 0.01382548863850, -0.00000231961904, 0.00000000022080};

    negFrictionCoff[0] = {-17.31286532389658, 0.02199648128339, 0.00000301188168, 0.00000000019084};
    negFrictionCoff[1] = {-17.31286532389658, 0.02199648128339, 0.00000301188168, 0.00000000019084};
    negFrictionCoff[2] = {-13.76663281161308*0.8, 0.01280380751465, 0.00000184258772, 0.00000000017014};
    negFrictionCoff[3] = {-13.76663281161308, 0.01280380751465, 0.00000184258772, 0.00000000017014};
    negFrictionCoff[4] = {-29.58013827603584*0.7, 0.01470026315148, 0.00000259310969, 0.00000000023595};
    negFrictionCoff[5] = {-29.58013827603584*0.7, 0.01470026315148, 0.00000259310969, 0.00000000023595};
}