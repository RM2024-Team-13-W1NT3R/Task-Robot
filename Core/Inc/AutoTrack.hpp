#pragma once
#include "main.h"
#include "DJIMotor.hpp"
#include <stdint.h>
#include "PID.hpp"

namespace AutoTrack
{
    void executeMovement(bool leftMode);
    void setStart();
}