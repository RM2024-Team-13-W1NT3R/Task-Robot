#include "AutoTrack.hpp"

static volatile float targetAutoCurrent;
static volatile uint16_t autoCanID;
namespace AutoTrack 
{
uint32_t startTime;
uint32_t autoLastUpdatedTime;
uint32_t currentTime;


uint32_t FORWARD_TIME = 3000;
uint32_t HORIZONTAL_TIME = 0;
uint8_t horizontalCount = 0;

static volatile float kp = 1;
static volatile float ki = 2;
static volatile float kd = 0;
Control::PID autoPID[4] {{kp, ki, kd}, {kp, ki, kd}, {kp, ki, kd}, {kp, ki, kd}};

int16_t FORWARD_SPEED = 1800;
int16_t HORIZONTAL_SPEED = 1350;


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

    if (leftMode && (state == State::FORWARD)) {
        targetMotorSpeed[0] = -targetMotorSpeed[0];
        targetMotorSpeed[1] = -targetMotorSpeed[1];
        targetMotorSpeed[2] = -targetMotorSpeed[2];
        targetMotorSpeed[3] = -targetMotorSpeed[3];
    }

    for (autoCanID = 1; autoCanID <= 4; autoCanID++) {
        targetAutoCurrent = autoPID[autoCanID - 1].update(targetMotorSpeed[autoCanID - 1], DJIMotor::getRPM(autoCanID));
        DJIMotor::setWheelsOutput(targetAutoCurrent, autoCanID);
    }   
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