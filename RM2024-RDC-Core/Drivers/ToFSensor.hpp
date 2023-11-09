/**
 * @file ToFSensor.hpp
 * @author - POON, Yiu Yeung
 * @date 2023-11-06
 * @brief This file is the header of ToFSensor Driver
 */

#pragma once

#include "stdint.h"
#include "vl53l1/VL53L1X_api.hpp"
#include "i2c.h"
#include "stm32f1xx_hal.h"
#include "usart.h"
namespace ToFSensor 
{


void init();
bool getDistance(uint16_t*);

extern uint16_t measureRate;

uint16_t* getMeasureRate();

}