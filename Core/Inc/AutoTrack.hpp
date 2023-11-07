#pragma once

#include "ToFSensor.hpp"
#include "DJIMotor.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "PID.hpp"

namespace AutoTrack 
{

bool setInitialHorizontalDistance();

void adjustForHorizontalMovement();

bool checkIfArrived();

void executeMovement();

}
