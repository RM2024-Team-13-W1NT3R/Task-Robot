#include "DJIMotor.hpp"

// DEF
#ifdef USE_DJI_MOTOR
#ifndef RDC_DJIMotor_MAX_NUM
#define RDC_DJIMotor_MAX_NUM 8
#endif

namespace DJIMotor
{
#define DJI_MOTOR_COUNT 4
// Initialize motor's controller instance

DJIMotor motorFeedback[DJI_MOTOR_COUNT];

uint32_t mailbox;
uint8_t rxData[8];
uint8_t txData[8];
CAN_TxHeaderTypeDef txHeader = {0x200, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};
CAN_RxHeaderTypeDef rxHeader;
CAN_FilterTypeDef filter[DJI_MOTOR_COUNT];
CAN_FilterTypeDef filterlist = {0x201 << 5,
                 0x202 << 5,
                 0x203 << 5,
                 0x204 << 5,
                 CAN_FILTER_FIFO0,
                 0,
                 CAN_FILTERMODE_IDLIST,
                 CAN_FILTERSCALE_16BIT,
                 CAN_FILTER_ENABLE,
                 0};
                 
volatile HAL_StatusTypeDef status;
/*========================================================*/
// Your implementation of the function, or even your customized function, should
// be implemented here
/*========================================================*/
/**
 * @todo
 */
void init()
{
    HAL_CAN_ConfigFilter(&hcan, &filterlist);
    HAL_CAN_Start(&hcan);
}

bool getRxMessage(uint16_t canID)
{
    // HAL_CAN_Stop(&hcan);
    // HAL_CAN_Start(&hcan);
    status =
        HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, rxData);
        //status = 0 means successfully receive, status = 1 means error


    if (rxHeader.StdId != ((uint32_t) (0x200 + canID)))
    {
        status = HAL_ERROR;
        return false; // receiving failed
    }


    if (!status)
    {
        motorFeedback[canID - 1].canID = canID;

        motorFeedback[canID - 1].rotorAngle =
            rxData[0] << 8 | rxData[1];

        motorFeedback[canID - 1].rpm =
            rxData[2] << 8 | rxData[3];

        motorFeedback[canID - 1].torqueCurrent =
            rxData[4] << 8 | rxData[5];

        motorFeedback[canID - 1].temperature = rxData[6];
        return true; // receiving complete
    }
    return false; // receiving failed 
}
/**
 * @todo
 */
float getEncoder(uint16_t canID) { return motorFeedback[canID - 1].rotorAngle; }

/**
 * @todo
 */
float getRPM(uint16_t canID) { return motorFeedback[canID - 1].rpm; }
 
/**
 * @todo
 */
void setOutput(float output, uint16_t canID)
{
    if (output > maxCurrent)
    {
        output = maxCurrent;
    }
    else if (output < -maxCurrent)
    {
        output = -maxCurrent;
    }
    // uint8_t mask                = 0xff;
    txData[(canID - 1) * 2 + 1] = (static_cast<int> (output));
    txData[(canID - 1) * 2]     = (static_cast<int> (output) >> 8);
    //why casting to int??
}

/**
 * @todo
 */
void transmit() { HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &mailbox); }

//void callback() {}

}  // namespace DJIMotor
#endif