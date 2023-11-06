/**
 * @file ToFSensor.hpp
 * @author - POON, Yiu Yeung
 * @date 2023-11-06
 * @brief This file is the header of ToFSensor Driver
 */

#pragma once

#include "stdint.h"
#include "vl35l1/VL53L1X_api.h"

namespace Control 
{


void init();

void getBootState(uint16_t device);

}