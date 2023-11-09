/**
 * @file ToFSensor.hpp
 * @author - POON, Yiu Yeung
 * @date 2023-11-06
 * @brief This file is the header of ToFSensor Driver
 */
#pragma once
#include <stdint.h>
#include "stm32f1xx_hal.h"

namespace Ultrasonic
{

bool getDistance(uint32_t distance);

}