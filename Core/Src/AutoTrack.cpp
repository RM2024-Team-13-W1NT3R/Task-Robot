#include "AutoTrack.hpp"

namespace AutoTrack 
{
uint32_t startTime;
uint32_t lastUpdatedTime;

uint32_t FORWARD_TIME = 5000;
uint32_t HORIZONTAL_TIME = 500;
uint8_t horizontalCount = 0;

static volatile float kp = 1;
static volatile float ki = 2;
static volatile float kd = 0.025;
Control::PID autoPID[4] {{kp, ki, kd}, {kp, ki, kd}, {kp, ki, kd}, {kp, ki, kd}};

#define FORWARD_SPEED 3000
#define HORIZONTAL_SPEED 1000


enum class State {
    START,
    FORWARD,
    MOVE_OUT_HORIZONTAL,
    MOVE_IN_HORIZONTAL,
    STOP
};

State state;


void controlMotor(State state, bool leftMode) {
    uint32_t targetMotorSpeed[4];
    float targetMotorOutput[4];
    switch (state)
    {
    case State::FORWARD:
        targetMotorSpeed[0] = FORWARD_SPEED;
        targetMotorSpeed[1] = FORWARD_SPEED;
        targetMotorSpeed[2] = FORWARD_SPEED;
        targetMotorSpeed[3] = FORWARD_SPEED;
        break;
    case State::MOVE_IN_HORIZONTAL:
        targetMotorSpeed[0] = HORIZONTAL_SPEED;
        targetMotorSpeed[1] = HORIZONTAL_SPEED;
        targetMotorSpeed[2] = -HORIZONTAL_SPEED;
        targetMotorSpeed[3] = -HORIZONTAL_SPEED;
        break;
    case State::MOVE_OUT_HORIZONTAL:
        targetMotorSpeed[0] = -HORIZONTAL_SPEED;
        targetMotorSpeed[1] = -HORIZONTAL_SPEED;
        targetMotorSpeed[2] = HORIZONTAL_SPEED;
        targetMotorSpeed[3] = HORIZONTAL_SPEED;
        break;
    default:
        break;
    }

    if (leftMode && (state == State::MOVE_IN_HORIZONTAL || state == State::MOVE_OUT_HORIZONTAL)) {
        targetMotorSpeed[0] = -targetMotorSpeed[0];
        targetMotorSpeed[1] = -targetMotorSpeed[1];
        targetMotorSpeed[2] = -targetMotorSpeed[2];
        targetMotorSpeed[3] = -targetMotorSpeed[3];
    }

    for (int canID = 1; canID <= 4; canID++) {
        switch (canID)
        {
        case 1:
            targetMotorOutput[0] = targetMotorSpeed[0];
            break;
        case 2:
            targetMotorOutput[1] = targetMotorSpeed[1];
            break;
        case 3:
            targetMotorOutput[2] = targetMotorSpeed[2];
            break;
        case 4:
            targetMotorOutput[3] = targetMotorSpeed[3];
            break;
        default:
            break;
        }
        float targetCurrent = autoPID[canID - 1].update(targetMotorOutput[canID - 1], DJIMotor::getRPM(canID));
        DJIMotor::setWheelsOutput(canID, targetCurrent);
    }   
    // DJIMotor::transmitWheels();
}

void executeMovement(bool leftMode) {
    // switch (state)
    // {
    // case State::START:
    //     startTime = HAL_GetTick();
    //     lastUpdatedTime = HAL_GetTick();
    //     state = State::MOVE_OUT_HORIZONTAL;
    //     break;
    // case State::FORWARD:
    //     if (HAL_GetTick() - lastUpdatedTime > FORWARD_TIME) {
    //         startTime = HAL_GetTick();
    //     } else {
    //         state = State::MOVE_IN_HORIZONTAL;
    //     }
    //     controlMotor(state, leftMode);
    //     break;
    // case State::MOVE_OUT_HORIZONTAL:
    //     if (HAL_GetTick() - lastUpdatedTime > HORIZONTAL_TIME) {
    //         startTime = HAL_GetTick();
    //     } else {
    //         state = State::FORWARD;
    //     }
    //     controlMotor(state, leftMode);
    //     break;
    // case State::MOVE_IN_HORIZONTAL:
    //     if (HAL_GetTick() - lastUpdatedTime > HORIZONTAL_TIME + 500 && horizontalCount < 2) {
    //         startTime = HAL_GetTick();
    //         horizontalCount++;
    //     } else {
    //         state = State::MOVE_OUT_HORIZONTAL;
    //     }
    //     controlMotor(state, leftMode);
    //     break;
    // default:
    //     break;
    // }
    
    controlMotor(State::FORWARD, leftMode);
    
}


void setStart() {
    state = State::START;
}

}