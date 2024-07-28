#include "actuatorControlSharedMemory.h"

ActuatorControl::ActuatorControl(RobotState& state)
    : robotState(state) {
    configureSharedMemory();
    // Initialization code
}

void ActuatorControl::communicateWithEthercat() {
    
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
        jointDataPtr[jnt_ctr]->actual_position = (driveObjectPtr[jnt_ctr]->actual_position * 2 * M_PI) / driveList[jnt_ctr].encoder_resolution;
        jointDataPtr[jnt_ctr]->actual_velocity = (driveObjectPtr[jnt_ctr]->actual_velocity * 2 * M_PI) / 60;
        jointDataPtr[jnt_ctr]->actual_torque = (driveObjectPtr[jnt_ctr]->actual_torque * driveList[jnt_ctr].motor_rated_torque * driveList[jnt_ctr].gear_ratio) / 1000;

        robotState.actualJointAngles[jnt_ctr] = jointDataPtr[jnt_ctr]->actual_position;
        robotState.actualJointVelocities[jnt_ctr] = jointDataPtr[jnt_ctr]->actual_velocity;
        robotState.actualJointTorques[jnt_ctr] = jointDataPtr[jnt_ctr]->actual_torque;
        
    }
}

void ActuatorControl::writeDriveData()
{

    for (unsigned int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        switch (fieldbusSharedDataPtr->operationMode)
        {
        case OperationModeState::POSITION_MODE:
            driveObjectPtr[jnt_ctr]->target_position = (int32_t)(jointDataPtr[jnt_ctr]->target_position / (2 * M_PI) * driveList[jnt_ctr].encoder_resolution);
            break;
        case OperationModeState::VELOCITY_MODE:
            driveObjectPtr[jnt_ctr]->target_velocity = (int32_t)(jointDataPtr[jnt_ctr]->target_position * 60 / (2 * M_PI));
            break;
        case OperationModeState::TORQUE_MODE:
            // robotState.targetJointTorques[jnt_ctr];
            driveObjectPtr[jnt_ctr]->target_torque = (int16_t)(jointDataPtr[jnt_ctr]->target_torque * 1000 / (driveList[jnt_ctr].gear_ratio * driveList[jnt_ctr].motor_rated_torque));
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

// #include "actuatorControlSharedMemory.h"

// ActuatorControl::ActuatorControl()
// {
//     configureSharedMemory();
// }

// ActuatorControl::~ActuatorControl()
// {
// }

// void ActuatorControl::run()
// {

//     switch (fieldbusSharedDataPtr->state)
//     {
//     case FieldbusState::NOT_READY:
//         /* code */
//         break;
//     case FieldbusState::READY:
//         readDriveData();
//         initializeWriteData();
//         break;
//     case FieldbusState::OPERATIONAL:
//         readDriveData();
//         writeDriveData();
//         break;
//     case FieldbusState::ERROR:
//         /* code */
//         break;

//     default:
//         break;
//     }
// }

