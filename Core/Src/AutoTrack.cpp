#include "AutoTrack.hpp"
#include <stdint.h>

typedef struct
{
    float motor0;
    float motor1;
    float motor2;
    float motor3;
} MotorRPM;

static MotorRPM autoMotorRPM = {};


float kp = 5.0f;
float ki = 1.0f;
float kd = 0.025f;
Control::PID autoPID[4]{{kp, ki, kd}, {kp, ki, kd}, {kp, ki, kd}, {kp, ki, kd}};
// Control::PID turningPID {0.01, 0, 0};

uint16_t estimatedTravelledDistance = 0;
uint16_t sideWallDistance;

uint16_t forwardRPM = 9000;
uint16_t turningConstant = 2;

float turningWeight = 0.0f;

static volatile uint16_t currentSideWallDistance = 60;
namespace AutoTrack 
{




bool checkIfArrived() {
    return false;
}


#define abs(x) ((x) > 0 ? (x) : -(x))


void adjustForHorizontalMovement() {
    // uint16_t currentSideWallDistance;
    // if (ToFSensor::getDistance(&currentSideWallDistance)) {
    if (true) {
        // currentSideWallDistance = 60;
        // turningWeight = turningPID.update(currentSideWallDistance, sideWallDistance);
        turningWeight = turningConstant * (abs(currentSideWallDistance - sideWallDistance) * (currentSideWallDistance - sideWallDistance));
        if (DR16::leftWallMode) {
            turningWeight = -turningWeight;
        }
        
        autoMotorRPM.motor0 = forwardRPM + turningWeight;
        autoMotorRPM.motor1 = forwardRPM + turningWeight;
        autoMotorRPM.motor2 = forwardRPM - turningWeight;
        autoMotorRPM.motor3 = forwardRPM - turningWeight;
    }
}



void executeMovement() {
    taskENTER_CRITICAL();
    float targetCurrent0 = autoPID[0].update(autoMotorRPM.motor0, DJIMotor::getRPM(0));
    float targetCurrent1 = autoPID[1].update(autoMotorRPM.motor1, DJIMotor::getRPM(1));
    float targetCurrent2 = autoPID[2].update(autoMotorRPM.motor2, DJIMotor::getRPM(2));
    float targetCurrent3 = autoPID[3].update(autoMotorRPM.motor3, DJIMotor::getRPM(3));

    DJIMotor::setOutput(targetCurrent0, 0);
    DJIMotor::setOutput(targetCurrent1, 1);
    DJIMotor::setOutput(targetCurrent2, 2);
    DJIMotor::setOutput(targetCurrent3, 3);

    DJIMotor::transmit();
    taskEXIT_CRITICAL();
}
}