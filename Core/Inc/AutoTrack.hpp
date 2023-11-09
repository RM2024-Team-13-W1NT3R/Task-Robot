#pragma once

#include "ToFSensor.hpp"
#include "DJIMotor.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "PID.hpp"
#include "DR16.hpp"
namespace AutoTrack 
{

void adjustForHorizontalMovement();

bool checkIfArrived();

void executeMovement();

bool setLeftWallMode(bool mode);

}
