#include "AutoTrack.hpp"
#include <stdint.h>

typedef struct
{
    float motor0;
    float motor1;
    float motor2;
    float motor3;
} MotorRPM;

MotorRPM* motorRPM;


float kp = 5.0f;
float ki = 1.0f;
float kd = 0.025f;
Control::PID autoPID[4]{{kp, ki, kd}, {kp, ki, kd}, {kp, ki, kd}, {kp, ki, kd}};
Control::PID turningPID {kp, ki, kd};

uint16_t estimatedTravelledDistance = 0;
uint16_t sideWallDistance;
bool leftWallMode = 0; // 0 = left, 1 = right
float turningWeight = 0.0f;
namespace AutoTrack 
{




bool checkIfArrived() {
    return false;
}

bool setInitialHorizontalDistance() {
    uint16_t currentSideWallDistance;
    if (ToFSensor::getDistance(&currentSideWallDistance)) {
        sideWallDistance = currentSideWallDistance;
        return true;
    }
    return false;
}

void adjustForHorizontalMovement() {
    uint16_t currentSideWallDistance;
    if (ToFSensor::getDistance(&currentSideWallDistance)) {
        turningWeight = turningPID.update(currentSideWallDistance, sideWallDistance);

        if (leftWallMode) {
            turningWeight = -turningWeight;
        }
        
        motorRPM->motor0 += turningWeight;
        motorRPM->motor1 += turningWeight;
        motorRPM->motor2 -= turningWeight;
        motorRPM->motor3 -= turningWeight;
    }
}



void executeMovement() {
    taskENTER_CRITICAL();
    float targetCurrent0 = autoPID[0].update(motorRPM->motor0, DJIMotor::getRPM(0));
    float targetCurrent1 = autoPID[1].update(motorRPM->motor1, DJIMotor::getRPM(1));
    float targetCurrent2 = autoPID[2].update(motorRPM->motor2, DJIMotor::getRPM(2));
    float targetCurrent3 = autoPID[3].update(motorRPM->motor3, DJIMotor::getRPM(3));

    DJIMotor::setOutput(targetCurrent0, 0);
    DJIMotor::setOutput(targetCurrent1, 1);
    DJIMotor::setOutput(targetCurrent2, 2);
    DJIMotor::setOutput(targetCurrent3, 3);

    DJIMotor::transmit();
    taskEXIT_CRITICAL();
}
}