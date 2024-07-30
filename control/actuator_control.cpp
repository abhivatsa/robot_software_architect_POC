#include "actuatorControlSharedMemory.h"

ActuatorControl::ActuatorControl(RobotState &state)
    : robotState(state)
{
    configureSharedMemory();
    // Initialization code

    // joint_1
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
    //     posFrictionCoff[0][0]=1.8353e+01;
    //     posFrictionCoff[0][1]=1.9756e-02;
    //     posFrictionCoff[0][2]=-1.9185e-06;
    //     posFrictionCoff[0][3]=7.1482e-11;
    //     negFrictionCoff[0][0]=-17.31286532389658;
    //     negFrictionCoff[0][1]=0.02199648128339;
    //     negFrictionCoff[0][2]=0.00000301188168;
    //     negFrictionCoff[0][3]=0.00000000019084;
    //     //Joint_3
    //     posFrictionCoff[2][0]=13.89006355525087;
    //     posFrictionCoff[2][1]=0.01257823874029;
    //     posFrictionCoff[2][2]=-0.00000160346214;
    //     posFrictionCoff[2][3]=0.00000000013339;
    //     negFrictionCoff[2][0]=-13.76663281161308;
    //     negFrictionCoff[2][1]=0.01280380751465;
    //     negFrictionCoff[2][2]=0.00000184258772;
    //     negFrictionCoff[2][3]=0.00000000017014;
    //     //Joint_5
    //     posFrictionCoff[4][0]=28.17743436155735;
    //     posFrictionCoff[4][1]=0.01382548863850;
    //     posFrictionCoff[4][2]=-0.00000231961904;
    //     posFrictionCoff[4][3]=0.00000000022080;
    //     negFrictionCoff[4][0]=-29.58013827603584;
    //     negFrictionCoff[4][1]=0.01470026315148;
    //     negFrictionCoff[4][2]=0.00000259310969;
    //     negFrictionCoff[4][3]= 0.00000000023595;
    //
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
        jointDataPtr[jnt_ctr]->actual_position = driveObjectPtr[jnt_ctr]->axis_direction * (driveObjectPtr[jnt_ctr]->actual_position * 2 * M_PI) / driveObjectPtr[jnt_ctr]->encoder_resolution;
        jointDataPtr[jnt_ctr]->actual_velocity = driveObjectPtr[jnt_ctr]->axis_direction * (driveObjectPtr[jnt_ctr]->actual_velocity * 2 * M_PI) / 60000;
        jointDataPtr[jnt_ctr]->actual_torque = driveObjectPtr[jnt_ctr]->axis_direction * (driveObjectPtr[jnt_ctr]->actual_torque * driveObjectPtr[jnt_ctr]->motor_rated_torque * driveObjectPtr[jnt_ctr]->gear_ratio) / 1000;

        robotState.actualJointAngles[jnt_ctr] = jointDataPtr[jnt_ctr]->actual_position;
        robotState.actualJointVelocities[jnt_ctr] = jointDataPtr[jnt_ctr]->actual_velocity;
        robotState.actualJointTorques[jnt_ctr] = jointDataPtr[jnt_ctr]->actual_torque;
    }
}

void ActuatorControl::frictionModel(std::vector<double> &friction_torque)
{
    friction_torque.resize(7);

    for (int jnt_ctr = 0; jnt_ctr < 7; jnt_ctr++)
    {

        if (driveObjectPtr[jnt_ctr]->actual_velocity > 100)
        {

            int vel = driveObjectPtr[jnt_ctr]->actual_velocity;

            if (vel > 4000)
            {
                vel = 4000;
            }

            friction_torque[jnt_ctr] = posFrictionCoff[jnt_ctr][0] + posFrictionCoff[jnt_ctr][1] * vel + posFrictionCoff[jnt_ctr][2] * vel * vel + posFrictionCoff[jnt_ctr][3] * vel * vel * vel;
        }
        else if (driveObjectPtr[jnt_ctr]->actual_velocity < -100)
        {

            int vel = driveObjectPtr[jnt_ctr]->actual_velocity;

            if (vel < -4000)
            {
                vel = -4000;
            }

            friction_torque[jnt_ctr] = negFrictionCoff[jnt_ctr][0] + negFrictionCoff[jnt_ctr][1] * vel + negFrictionCoff[jnt_ctr][2] * vel * vel + negFrictionCoff[jnt_ctr][3] * vel * vel * vel;
        }
    }
}

void ActuatorControl::writeDriveData()
{

    std::vector<double> fric_torq;
    fric_torq.resize(7, 0.0);
    frictionModel(fric_torq);

    for (unsigned int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        switch (fieldbusSharedDataPtr->operationMode)
        {
        case OperationModeState::POSITION_MODE:
            driveObjectPtr[jnt_ctr]->target_position = (int32_t)(driveObjectPtr[jnt_ctr]->axis_direction * jointDataPtr[jnt_ctr]->target_position / (2 * M_PI) * driveObjectPtr[jnt_ctr]->encoder_resolution);
            break;
        case OperationModeState::VELOCITY_MODE:
            driveObjectPtr[jnt_ctr]->target_velocity = (int32_t)(driveObjectPtr[jnt_ctr]->axis_direction * jointDataPtr[jnt_ctr]->target_position * 60 / (2 * M_PI));
            break;
        case OperationModeState::TORQUE_MODE:
            // std::cout<<"torq : "<<robotState.targetJointTorques[jnt_ctr]<<std::endl;
            // std::cout<<"pos : "<< robotState.actualJointAngles[jnt_ctr]<<"torq : "<<robotState.targetJointTorques[jnt_ctr]<<", target torq : "<<jnt_ctr<<" : "<<(-1*driveObjectPtr[jnt_ctr]->axis_direction*robotState.targetJointTorques[jnt_ctr]* 1000 / (driveObjectPtr[jnt_ctr]->gear_ratio * driveObjectPtr[jnt_ctr]->motor_rated_torque))<<std::endl;
            // std::cout<<"act pos :" <<jnt_ctr<<" : "<<jointDataPtr[jnt_ctr]->actual_position<<std::endl;
            // robotState.targetJointTorques[jnt_ctr];
            //             int16_t fric_torque  = 0;

            // int32_t vel =  driveObjectPtr[i]->actual_velocity;

            // if (vel > 0){

            //     if (vel > 4000){
            //         vel = 4000;
            //     }
            //     fric_torque = 0.85*posFrictionCoff[NUM_JOINTS][0] + posFrictionCoff[NUM_JOINTS][1]*vel +posFrictionCoff[NUM_JOINTS][2]*vel*vel + posFrictionCoff[NUM_JOINTS][3]*vel*vel*vel;

            // }else if (vel < 0){
            //     if (vel <  -4000){
            //         vel = -4000;
            //     }

            //     fric_torque = 0.85*negFrictionCoff[NUM_JOINTS][0] + negFrictionCoff[NUM_JOINTS][1]*vel + negFrictionCoff[NUM_JOINTS][2]*vel*vel + negFrictionCoff[NUM_JOINTS][3]*vel*vel*vel;
            // }

            // std::cout<<"fric_torque : "<<fric_torque<<std::endl;

            std::cout << "jnt_ctr  : " << jnt_ctr << ", fric torq : " << fric_torq[jnt_ctr] << std::endl;

            driveObjectPtr[jnt_ctr]->target_torque = (int16_t)((-1 * driveObjectPtr[jnt_ctr]->torque_axis_direction * robotState.targetJointTorques[jnt_ctr] * 1000 / (driveObjectPtr[jnt_ctr]->gear_ratio * driveObjectPtr[jnt_ctr]->motor_rated_torque)) + fric_torq[jnt_ctr]);
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
