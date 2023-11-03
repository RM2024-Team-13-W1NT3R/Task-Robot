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
uint8_t rxData[DJI_MOTOR_COUNT][8];
uint8_t txData[8];
CAN_TxHeaderTypeDef txHeader = {0x200, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};
CAN_RxHeaderTypeDef rxHeader;
CAN_FilterTypeDef filter[DJI_MOTOR_COUNT];

/*========================================================*/
// Your implementation of the function, or even your customized function, should
// be implemented here
/*========================================================*/
/**
 * @todo
 */
void init()
{
    // for (uint32_t i = 0; i < DJI_MOTOR_COUNT; i++) {
    filter[0] = {0x201 << 5,
                 0,
                 0,
                 0,
                 CAN_FILTER_FIFO0,
                 0,
                 CAN_FILTERMODE_IDLIST,
                 CAN_FILTERSCALE_16BIT,
                 CAN_FILTER_ENABLE,
                 0};
    filter[1] = {0x202 << 5,
                 0,
                 0,
                 0,
                 CAN_FILTER_FIFO0,
                 0,
                 CAN_FILTERMODE_IDLIST,
                 CAN_FILTERSCALE_16BIT,
                 CAN_FILTER_ENABLE,
                 0};
    filter[2] = {0x203 << 5,
                 0,
                 0,
                 0,
                 CAN_FILTER_FIFO0,
                 0,
                 CAN_FILTERMODE_IDLIST,
                 CAN_FILTERSCALE_16BIT,
                 CAN_FILTER_ENABLE,
                 0};
    filter[3] = {0x204 << 5,
                 0,
                 0,
                 0,
                 CAN_FILTER_FIFO0,
                 0,
                 CAN_FILTERMODE_IDLIST,
                 CAN_FILTERSCALE_16BIT,
                 CAN_FILTER_ENABLE,
                 0};
    // HAL_CAN_ConfigFilter(&hcan, &filter[0]);
    HAL_CAN_Start(&hcan);
}
bool getRxMessage(uint16_t canID)
{
    HAL_CAN_ConfigFilter(&hcan, &filter[canID - 1]);
    HAL_StatusTypeDef status =
        HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, rxData[canID - 1]);
        //status = 0 means successfully receive, status = 1 means error
    if (!status)
    {
        motorFeedback[canID - 1].canID = canID;

        motorFeedback[canID - 1].rotorAngle =
            rxData[canID - 1][0] << 8 | rxData[canID - 1][1];

        motorFeedback[canID - 1].rpm =
            rxData[canID - 1][2] << 8 | rxData[canID - 1][3];

        motorFeedback[canID - 1].torqueCurrent =
            rxData[canID - 1][4] << 8 | rxData[canID - 1][5];

        motorFeedback[canID - 1].temperature = rxData[canID - 1][6];
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
void setOutput(int16_t output, uint16_t canID)
{
    if (output > maxCurrent)
    {
        output = maxCurrent;
    }
    else if (output < -maxCurrent)
    {
        output = -maxCurrent;
    }
    uint8_t mask                = 0xff;
    txData[(canID - 1) * 2 + 1] = mask & output;
    txData[(canID - 1) * 2]     = output >> 8;
}

/**
 * @todo
 */
void transmit() { HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &mailbox); }

//void callback() {}

}  // namespace DJIMotor
#endif