#include "PID.hpp"
#if USE_PID
namespace Control
{

/*This is where you implement the PID algorithm*/
float PID::update(float target, float measurement, float dt)
{
    /*=====================================================================*/
    // Your implementation of the PID algorithm begins here
    /*=====================================================================*/
    error = target - measurement;
    pOut = Kp * error;

    iOut += Ki * error * dt;

    dOut = Kd * (error - lastError) / dt;

    output = pOut + iOut + dOut;

    lastError = error;
    /*=====================================================================*/
    // Your implementation of the PID algorithm ends here
    /*=====================================================================*/
    return this->output;  // You need to give your user the output for every update
}

}  // namespace Control
#endif
