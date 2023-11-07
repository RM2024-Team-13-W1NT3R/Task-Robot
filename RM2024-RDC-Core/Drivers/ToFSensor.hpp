/**
 * @file ToFSensor.hpp
 * @author - POON, Yiu Yeung
 * @date 2023-11-06
 * @brief This file is the header of ToFSensor Driver
 */

#pragma once

#include "stdint.h"
#include "vl53l1/VL53L1X_api.h"
#include "i2c.h"
#include "stm32f1xx_hal.h"
namespace ToFSensor 
{


void init();
uint16_t getDistance();

}