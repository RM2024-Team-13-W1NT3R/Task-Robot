#include "ToFSensor.hpp"

uint8_t tofRxBuffer[1];

namespace ToFSensor 
{

uint8_t bootState = 0;
VL53L1X_ERROR status;

uint16_t deviceAddress = 0x53;

uint16_t measureRate;
#define ToF_FRAME_LENGTH 1

uint16_t* getMeasureRate() {
    status = VL53L1X_GetSignalRate(deviceAddress, &measureRate);
    return &measureRate;
}

static volatile uint32_t testCode = 0;

void rxEventCallback(UART_HandleTypeDef *huart, uint16_t datasize) {
    status = HAL_UARTEx_ReceiveToIdle_IT(huart, tofRxBuffer, ToF_FRAME_LENGTH); // start the next round of UART data reception
    // reset the rcData if the data is invalid
   
}


void init() {
    HAL_UART_RegisterRxEventCallback(&huart1, rxEventCallback);
    status = HAL_UARTEx_ReceiveToIdle_IT(&huart1, tofRxBuffer, ToF_FRAME_LENGTH);
   
}

// bool getDistance(uint16_t* distance) {
//     uint16_t targetDistance;
//     status = VL53L1X_GetDistance(deviceAddress, &targetDistance);
//     if (status == VL53L1_ERROR_NONE) {
//         status = VL53L1X_ClearInterrupt(deviceAddress);
//         distance = &targetDistance;
//         return true;
//     } else {
//         return false;
//     }
// }


}
