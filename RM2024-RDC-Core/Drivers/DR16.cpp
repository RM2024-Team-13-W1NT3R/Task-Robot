/**
 * @file DR16.cpp
 * @brief Implement the function definition that is declared in DR16.hpp file
 */

#include "DR16.hpp"

#if USE_DR16

namespace DR16
{

/**
 * @brief Define a singleton RcData structure instance here.
 * @remark If you wish to, please decode the DR16 data from the buffer to here
 * @remark Refer to the definition of the structure in the "DR16.hpp" files
 */
static RcData rcData;
bool rcConnected = false;
/*Return the constant pointer of the current decoded data*/
const RcData *getRcData() { return &rcData; }
uint32_t lastUpdatedTime;

/*================================================================================*/
/*You are free to declare your buffer, or implement your own function(callback, decoding) here*/
const uint8_t DR16_FRAME_LENGTH = 18;
uint8_t rcRxBuffer[DR16::DR16_FRAME_LENGTH];

void resetRcData() {
    rcData.channel0 = 1024;
    rcData.channel1 = 1024;
    rcData.channel2 = 1024;
    rcData.channel3 = 1024;
    rcData.s1 = 3;
    rcData.s2 = 3;
}

bool validateRcData() {
    if ((rcData.channel0 > 1684 || rcData.channel0 < 364) ||
        (rcData.channel1 > 1684 || rcData.channel1 < 364) ||
        (rcData.channel2 > 1684 || rcData.channel2 < 364) ||
        (rcData.channel3 > 1684 || rcData.channel3 < 364) ||
        (rcData.s1 > 3 || rcData.s1 < 1) ||
        (rcData.s2 > 3 || rcData.s2 < 1)) {
        return false;
    }
    return true;
}

void decodeRcData() {
    rcData.channel0 = ((uint16_t) rcRxBuffer[0] | (uint16_t) rcRxBuffer[1] << 8) & 0x07FF;
    rcData.channel1 = ((uint16_t) rcRxBuffer[1] >> 3 | (uint16_t) rcRxBuffer[2] << 5) & 0x07FF;
    rcData.channel2 = ((uint16_t) rcRxBuffer[2] >> 6 | (uint16_t) rcRxBuffer[3] << 2 | rcRxBuffer[4] << 10) & 0x07FF;
    rcData.channel3 = ((uint16_t) rcRxBuffer[4] >> 1 | (uint16_t) rcRxBuffer[5] << 7) & 0x07FF;
    rcData.s1 = ((uint16_t) rcRxBuffer[5] >> 4) & 0x0003;
    rcData.s2 = ((uint16_t) rcRxBuffer[5] >> 6) & 0x0003;
}

void rxEventCallback(UART_HandleTypeDef *huart, uint16_t datasize) {
    if (huart->Instance == USART1) {
        decodeRcData();
        HAL_UARTEx_ReceiveToIdle_IT(huart, rcRxBuffer, DR16::DR16_FRAME_LENGTH);
        if (!validateRcData()) {
            resetRcData();
        }
        lastUpdatedTime = HAL_GetTick();
    }
}



bool getIsRcConnected() {
    uint32_t currentTime = HAL_GetTick();
    if ((lastUpdatedTime + 1000 > currentTime )) {
        rcConnected = true;
        return rcConnected;
    } else {
        rcConnected = false;
        resetRcData();
        return rcConnected;
    }
}

const bool *getRcConnected() {
    getIsRcConnected();
    return &rcConnected;
}



/*================================================================================*/
void init()
{
    /*If you would like to, please implement your function definition here*/
    resetRcData();
    HAL_UART_RegisterRxEventCallback(&huart1, DR16::rxEventCallback);
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, rcRxBuffer, DR16::DR16_FRAME_LENGTH);
}

}  // namespace DR16

#endif