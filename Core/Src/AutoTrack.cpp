#include "AutoTrack.hpp"

static volatile float targetAutoCurrent;
static volatile uint16_t autoCanID;
namespace AutoTrack 
{
uint32_t startTime;
uint32_t autoLastUpdatedTime;
uint32_t currentTime;


uint32_t FORWARD_TIME = 3000;
uint32_t HORIZONTAL_TIME = 100;
uint8_t horizontalCount = 0;

static volatile float kp = 1;
static volatile float ki = 2;
static volatile float kd = 0;
Control::PID autoPID[4] {{kp, ki, kd}, {kp, ki, kd}, {kp, ki, kd}, {kp, ki, kd}};

int16_t FORWARD_SPEED = 2000;
int16_t HORIZONTAL_SPEED = 2000;


enum class State {
    START,
    FORWARD,
    MOVE_OUT_HORIZONTAL,
    MOVE_IN_HORIZONTAL,
    STOP
};

State state;

static volatile int32_t targetMotorSpeed[4];
float targetMotorOutput[4];

void controlMotor(State state, bool leftMode) {
    switch (state)
    {
    case State::FORWARD:
        targetMotorSpeed[0] = FORWARD_SPEED;
        targetMotorSpeed[1] = -FORWARD_SPEED;
        targetMotorSpeed[2] = FORWARD_SPEED;
        targetMotorSpeed[3] = -FORWARD_SPEED;
        break;
    case State::MOVE_IN_HORIZONTAL:
        targetMotorSpeed[0] = FORWARD_SPEED + HORIZONTAL_SPEED;
        targetMotorSpeed[1] = -FORWARD_SPEED + HORIZONTAL_SPEED;
        targetMotorSpeed[2] = FORWARD_SPEED + -HORIZONTAL_SPEED;
        targetMotorSpeed[3] = -FORWARD_SPEED + -HORIZONTAL_SPEED;
        break;
    case State::MOVE_OUT_HORIZONTAL:
        targetMotorSpeed[0] = FORWARD_SPEED + -HORIZONTAL_SPEED;
        targetMotorSpeed[1] = -FORWARD_SPEED + -HORIZONTAL_SPEED;
        targetMotorSpeed[2] = FORWARD_SPEED + HORIZONTAL_SPEED;
        targetMotorSpeed[3] = -FORWARD_SPEED + HORIZONTAL_SPEED;
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

    // if (state == State::MOVE_IN_HORIZONTAL || state == State::MOVE_OUT_HORIZONTAL) {
    //     if (leftMode) {
    //         targetMotorSpeed[0] = 0;
    //         targetMotorSpeed[2] = 0;
    //     } else {
    //         targetMotorSpeed[1] = 0;
    //         targetMotorSpeed[3] = 0;
    //     }
    // }
    
    for (autoCanID = 1; autoCanID <= 4; autoCanID++) {
        // switch (autoCanID)
        // {
        // case 1:
        //     targetMotorOutput[0] = targetMotorSpeed[0];
        //     break;
        // case 2:
        //     targetMotorOutput[1] = targetMotorSpeed[1];
        //     break;
        // case 3:
        //     targetMotorOutput[2] = targetMotorSpeed[2];
        //     break;
        // case 4:
        //     targetMotorOutput[3] = targetMotorSpeed[3];
        //     break;
        // default:
        //     break;
        // }
        targetAutoCurrent = autoPID[autoCanID - 1].update(targetMotorSpeed[autoCanID - 1], DJIMotor::getRPM(autoCanID));
        DJIMotor::setWheelsOutput(targetAutoCurrent, autoCanID);
    }   
    // DJIMotor::transmitWheels();
}

void executeMovement(bool leftMode) {
    currentTime = HAL_GetTick();
    switch (state)
    {
    case State::START:
        startTime = HAL_GetTick();
        state = State::MOVE_OUT_HORIZONTAL;
        autoLastUpdatedTime = HAL_GetTick();
        break;
    case State::FORWARD:
        if (currentTime - autoLastUpdatedTime > FORWARD_TIME) {
            state = State::MOVE_IN_HORIZONTAL;
            autoLastUpdatedTime = HAL_GetTick();
        }
        controlMotor(state, leftMode);
        break;
    case State::MOVE_OUT_HORIZONTAL:
        if (currentTime - autoLastUpdatedTime > HORIZONTAL_TIME - HORIZONTAL_TIME) {
            state = State::FORWARD;
            autoLastUpdatedTime = HAL_GetTick();
        }
        controlMotor(state, leftMode);
        break;
    case State::MOVE_IN_HORIZONTAL:
        // if (currentTime - lastUpdatedTime > HORIZONTAL_TIME + 500 && horizontalCount < 2) {
        if (currentTime - autoLastUpdatedTime > HORIZONTAL_TIME) {
            state = State::MOVE_OUT_HORIZONTAL;
            autoLastUpdatedTime = HAL_GetTick();
            horizontalCount++;
        }
        controlMotor(state, leftMode);
        break;
    default:
        break;
    }
}


void setStart() {
    state = State::START;
}

} 