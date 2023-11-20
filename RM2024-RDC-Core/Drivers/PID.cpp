#include "PID.hpp"
#if USE_PID

namespace Control
{
float MAX_OUTPUT = 16834/2;
/*This is where you implement the PID algorithm*/

// Find the Absolute Value of a Number
#define Abs(N) ((N<0)?(-N):(N))

float PID::update(float target, float measurement, float dt)
{
    /*=====================================================================*/
    // Your implementation of the PID algorithm begins here
    /*=====================================================================*/
    error = target - measurement;

    // proportional term
    pOut = Kp * error;

    // integral term
    iOut += Ki * error * dt;

    // Limit the integral to prevent huge overshoot when quickly switching between the max values
    if (iOut > 16834) {
        iOut = 16834;
    } else if (iOut < -16834) {
        iOut = -16834;
    }
    
    // integral term reset
    if (Abs(error) < 10) {
        iOut = iOut * 0.9;
    }
    
    // derivative term
    dOut = Kd * (error - lastError) / dt  - Kdamp * dOut;
    lastError = error;

    output = pOut + iOut + dOut;
    output = output > MAX_OUTPUT ? MAX_OUTPUT : output;
    output = output < -MAX_OUTPUT ? -MAX_OUTPUT : output;
    /*=====================================================================*/
    // Your implementation of the PID algorithm ends here
    /*=====================================================================*/
    return this->output;  // You need to give your user the output for every update
}

float PID::updateAngle(int32_t target, int32_t measurement, float dt)
{
    /*=====================================================================*/
    // Your implementation of the PID algorithm begins here
    /*=====================================================================*/
    error = target - measurement;
    error = error < 0 ? -error : error;
    pOut = Kp * error;

    iOut += Ki * error * dt;

    // Limit the integral to prevent huge overshoot when quickly switching between the max values
    if (iOut > 16834) {
        iOut = 16834;
    } else if (iOut < -16834) {
        iOut = -16834;
    }

    dOut = Kd * (error - lastError) / dt;

    output = pOut + iOut + dOut;
    output = output > MAX_OUTPUT ? MAX_OUTPUT : output;
    output = output < -MAX_OUTPUT ? -MAX_OUTPUT : output;
    lastError = error;
    /*=====================================================================*/
    // Your implementation of the PID algorithm ends here
    /*=====================================================================*/
    output = output > 16834/2 ? 16834 : output;
    output = output < -16834 ? -16834 : output;
    return this->output;  // You need to give your user the output for every update
    
}

void PID::resetPID() {
    dOut = 0;
    error = 0;
    output = 0;
    iOut = 0;

}
}  // namespace Control
#endif
