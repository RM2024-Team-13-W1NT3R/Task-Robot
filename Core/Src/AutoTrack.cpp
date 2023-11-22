#include "AutoTrack.hpp"

static volatile float targetAutoCurrent;
static volatile uint16_t autoCanID;
namespace AutoTrack 
{
uint32_t startTime;
uint32_t autoLastUpdatedTime;
uint32_t currentTime;


uint32_t FORWARD_TIME = 100000;
uint32_t HORIZONTAL_TIME = 0;
uint8_t horizontalCount = 0;

// static volatile float kp = 1; 
// static volatile float ki = 2;
// static volatile float kd = 0;x
static volatile float autokp = 2.0f;
static volatile float autoki = 15.0f;
static volatile float autokd = 0.0f;
Control::PID autoPID[4] {{autokp, autoki, autokd}, {autokp, autoki, autokd}, {autokp, autoki, autokd}, {autokp, autoki, autokd}};

int16_t FORWARD_SPEED = 1800;
int16_t HORIZONTAL_SPEED = 0;


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

// Auto shortcut flow
// 1. Goes and sticks to the wall to align
// 2. Move a bit away from the wall
// 3. Move forward along the wall
// Steps 2 and 3 were removed due to the auto shortcut being very straight
void controlMotor(State state, bool Mode) {
    switch (state)
    {
    // Move forward
    case State::FORWARD:
        targetMotorSpeed[0] = FORWARD_SPEED;
        targetMotorSpeed[1] = -FORWARD_SPEED;
        targetMotorSpeed[2] = FORWARD_SPEED;
        targetMotorSpeed[3] = -FORWARD_SPEED;
        break;
    // Go and stick to the wall
    case State::MOVE_IN_HORIZONTAL:
        targetMotorSpeed[0] = FORWARD_SPEED + HORIZONTAL_SPEED;
        targetMotorSpeed[1] = -FORWARD_SPEED + HORIZONTAL_SPEED;
        targetMotorSpeed[2] = FORWARD_SPEED + -HORIZONTAL_SPEED;
        targetMotorSpeed[3] = -FORWARD_SPEED + -HORIZONTAL_SPEED;
        break;
    // Move away from the wall
    case State::MOVE_OUT_HORIZONTAL:
        targetMotorSpeed[0] = FORWARD_SPEED + -HORIZONTAL_SPEED;
        targetMotorSpeed[1] = -FORWARD_SPEED + -HORIZONTAL_SPEED;
        targetMotorSpeed[2] = FORWARD_SPEED + HORIZONTAL_SPEED;
        targetMotorSpeed[3] = -FORWARD_SPEED + HORIZONTAL_SPEED;
        break;
    default:
        break;
    }
    // Reverse the forward direction if the robot is in reverse mode
    if (Mode && (state == State::FORWARD)) {
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

void executeMovement(bool Mode) {
    currentTime = HAL_GetTick();
    switch (state)
    {
    // Start the auto shortcut mode
    case State::START:
        startTime = HAL_GetTick();
        state = State::FORWARD;
        for (int i = 0; i < 4; i++) {
            autoPID[i].resetPID();
        }
        autoLastUpdatedTime = HAL_GetTick();
        break;
    
    // Start the forward mode
    case State::FORWARD:
        if (currentTime - autoLastUpdatedTime > FORWARD_TIME) {
            state = State::MOVE_IN_HORIZONTAL;
            autoLastUpdatedTime = HAL_GetTick();
        }
        controlMotor(state, Mode);
        break;

    // Start moving away from the wall
    case State::MOVE_OUT_HORIZONTAL:
        if (currentTime - autoLastUpdatedTime > HORIZONTAL_TIME - HORIZONTAL_TIME) {
            state = State::FORWARD;
            autoLastUpdatedTime = HAL_GetTick();
        }
        controlMotor(state, Mode);
        break;
    
    // Start moving towards the wall
    case State::MOVE_IN_HORIZONTAL:
        if (currentTime - autoLastUpdatedTime > HORIZONTAL_TIME) {
            state = State::MOVE_OUT_HORIZONTAL;
            autoLastUpdatedTime = HAL_GetTick();
            horizontalCount++;
        }
        controlMotor(state, Mode);
        break;
    default:
        break;
    }
}


void setStart() {
    state = State::START;
}

} 