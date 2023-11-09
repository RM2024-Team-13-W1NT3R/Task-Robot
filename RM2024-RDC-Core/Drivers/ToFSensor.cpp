#include "ToFSensor.hpp"

namespace ToFSensor 
{

uint8_t bootState = 0;
VL53L1X_ERROR vl53_status = 99;

uint16_t deviceAddress = 0x50;

uint16_t measureRate;

uint16_t* getMeasureRate() {
    vl53_status = VL53L1X_GetSignalRate(deviceAddress, &measureRate);
    return &measureRate;
}

static volatile uint32_t testCode = 0;

void init() {
    testCode = 1;
    while (!bootState) {
        testCode = 2;
        HAL_Delay(2);
        vl53_status =  VL53L1X_BootState(deviceAddress, &bootState);
        // deviceAddress++;
    }
    // deviceAddress--;
    
    testCode = 3;
    VL53L1X_SensorInit(deviceAddress);
    vl53_status = VL53L1X_SetDistanceMode(deviceAddress, 1); /* 1=short, 2=long */
    vl53_status = VL53L1X_SetTimingBudgetInMs(deviceAddress, 100); /* in ms possible values [20, 50, 100, 200, 500] */
    vl53_status = VL53L1X_SetInterMeasurementInMs(deviceAddress, 100); /* in ms, IM must be > = TB */
    vl53_status = VL53L1X_StartRanging(deviceAddress);
}

bool getDistance(uint16_t* distance) {
    uint16_t targetDistance;
    vl53_status = VL53L1X_GetDistance(deviceAddress, &targetDistance);
    if (vl53_status == VL53L1_ERROR_NONE) {
        vl53_status = VL53L1X_ClearInterrupt(deviceAddress);
        distance = &targetDistance;
        return true;
    } else {
        return false;
    }
}


}
