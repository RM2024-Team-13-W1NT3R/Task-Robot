#include "ToFSensor.hpp"

namespace ToFSensor 
{

uint8_t bootState = 0;
VL53L1X_ERROR status;

uint16_t deviceAddress = 0x53;




void init() {
    while (!bootState) {
        HAL_Delay(2);
        status =  VL53L1X_BootState(deviceAddress, &bootState);
    }
    VL53L1X_SensorInit(deviceAddress);
    status = VL53L1X_SetDistanceMode(deviceAddress, 2); /* 1=short, 2=long */
    status = VL53L1X_SetTimingBudgetInMs(deviceAddress, 100); /* in ms possible values [20, 50, 100, 200, 500] */
    status = VL53L1X_SetInterMeasurementInMs(deviceAddress, 100); /* in ms, IM must be > = TB */
    status = VL53L1X_StartRanging(deviceAddress);
}

uint16_t getDistance() {
    uint16_t targetDistance;
    status = VL53L1X_GetDistance(deviceAddress, &targetDistance);
    if (status == VL53L1_ERROR_NONE) {
        return targetDistance;
    } else {
        return 0;
    }
}


}
