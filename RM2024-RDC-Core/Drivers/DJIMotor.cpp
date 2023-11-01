#include "DJIMotor.hpp"

// DEF
#ifdef USE_DJI_MOTOR
#ifndef RDC_DJIMotor_MAX_NUM
#define RDC_DJIMotor_MAX_NUM 8
#endif

namespace DJIMotor
{

// Initialize motor's controller instance

DJIMotor motors[8];
uint32_t mailbox;
uint8_t rxData1[8];
//uint8_t rxData2[8];
//uint8_t rxData3[8];
//uint8_t rxData4[8];
uint8_t txData[8];
CAN_TxHeaderTypeDef txHeader = {0x200,0,CAN_ID_STD,CAN_RTR_DATA,8,DISABLE};
CAN_RxHeaderTypeDef rxHeader;
CAN_FilterTypeDef filter = {0x201<<5,0x202<<5,0x203<<5,0x204<<5,CAN_FILTER_FIFO0,0,CAN_FILTERMODE_IDLIST,CAN_FILTERSCALE_16BIT,CAN_FILTER_ENABLE,0};
typedef struct
{
    uint16_t rotorAngle;
    uint16_t rpm;
    uint16_t torqueCurrent;
    uint8_t temperature;
}motorFeedback;
motorFeedback motor1feedback;
//motorFeedback motor2feedback = {rxData2[0]<<8|rxData2[1],rxData2[2]<<8|rxData2[3],rxData2[4]<<8|rxData2[5],rxData2[6]};
//motorFeedback motor3feedback = {rxData3[0]<<8|rxData3[1],rxData3[2]<<8|rxData3[3],rxData3[4]<<8|rxData3[5],rxData3[6]};
//motorFeedback motor4feedback = {rxData4[0]<<8|rxData4[1],rxData4[2]<<8|rxData4[3],rxData4[4]<<8|rxData4[5],rxData4[6]};
 
/*========================================================*/
// Your implementation of the function, or even your customized function, should
// be implemented here
/*========================================================*/
/**
 * @todo
 */
void init() {
    HAL_CAN_ConfigFilter(&hcan, &filter);
    HAL_CAN_Start(&hcan);
}

/**
 * @todo
 */
float getEncoder(uint16_t canID) { 
    HAL_CAN_AddRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, rxData1);
    motor1feedback = {rxData1[0]<<8|rxData1[1],rxData1[2]<<8|rxData1[3],rxData1[4]<<8|rxData1[5],rxData1[6]};
    return motor1feedback;
 }

/**
 * @todo
 */
float getRPM(uint16_t canID) { 
    uint16_t rpm;
    rpm = motor1feedback.rpm;
    return rpm;
 }

/**
 * @todo
 */
void setOutput(int16_t output, uint16_t canID) {
    int16_t maxCurrent = 16384;
    if(output > maxCurrent)
    {
        output = maxCurrent;
    }
    else if(output < -maxCurrent)
    {
        output = -maxCurrent;
    }
    txData[(canID-1)*2] = output >> 8;
    txData[(canID-1)*2 + 1] = output << 8;
}

/**
 * @todo
 */
void transmit(uint16_t header) {
    HAL_CAN_AddTxMessage(&hcan,&txHeader,txData, &mailbox);
}

}  // namespace DJIMotor
#endif