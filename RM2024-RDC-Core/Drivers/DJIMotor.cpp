#include "DJIMotor.hpp"

// DEF
#ifdef USE_DJI_MOTOR
#ifndef RDC_DJIMotor_MAX_NUM
#define RDC_DJIMotor_MAX_NUM 8
#endif

namespace DJIMotor
{
#define DJI_MOTOR_COUNT 6
// Initialize motor's controller instance

static int32_t targetClampAngle = 0;



int32_t* getTargetClampAngle() {
    return &targetClampAngle;
}


DJIMotor motorFeedback[DJI_MOTOR_COUNT];
uint32_t mailbox;
uint8_t rxData[8];
uint8_t txWheelsData[8];
uint8_t txClampData[8];

uint8_t clampRotation = 5;
uint32_t clampAngleOffset = 0;
bool clampRotationCalibrated = false;

int16_t measured_motorAngle = 0;
int32_t newAngle;

CAN_TxHeaderTypeDef txWheelsHeader = {0x200, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};
CAN_TxHeaderTypeDef txClampHeader  = {0x1ff, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};
CAN_RxHeaderTypeDef rxHeader;
CAN_FilterTypeDef filter[DJI_MOTOR_COUNT];
CAN_FilterTypeDef filterlist = {
                0x201 << 5,
                0x202 << 5,
                0x203 << 5,
                0x204 << 5,
                CAN_FILTER_FIFO0,
                0,
                CAN_FILTERMODE_IDLIST,
                CAN_FILTERSCALE_16BIT,
                CAN_FILTER_ENABLE,
                0};
// use mask mode instead
CAN_FilterTypeDef filterlist1 = {
                0x205 << 5,
                0x206 << 5,
                0,
                0,
                CAN_FILTER_FIFO0,
                1,
                CAN_FILTERMODE_IDMASK,
                CAN_FILTERSCALE_16BIT,
                CAN_FILTER_ENABLE,
                0};

                 
uint16_t getTorqueCurrent(uint16_t canID) {
    return motorFeedback[canID - 1].torqueCurrent;
}

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
    HAL_CAN_ConfigFilter(&hcan, &filterlist1);
    HAL_CAN_Start(&hcan);
}
uint32_t fifoLevel;
uint32_t curFifoLevel;
void getRxMessage()
{
    // HAL_CAN_Stop(&hcan);
    // HAL_CAN_Start(&hcan);

    fifoLevel = HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);
    for (int i = 0; i < fifoLevel; i++) {
        HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, rxData);
        curFifoLevel = HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);
        uint16_t canID = rxHeader.StdId - 0x200;
        
        motorFeedback[canID - 1].canID = canID;

        motorFeedback[canID - 1].rpm =
            rxData[2] << 8 | rxData[3];
        
        // if (canID != 6) {
        // motorFeedback[canID - 1].motorAngle =
        // rxData[0] << 8 | rxData[1];
        // } else {
        //     measured_motorAngle = rxData[0] << 8 | rxData[1];
        //     newAngle = measured_motorAngle + 8192 * clampRotation - clampAngleOffset;
        //     if (!clampRotationCalibrated) {
        //         clampRotationCalibrated = true;
        //         clampRotation = 5;
        //         clampAngleOffset = rxData[0] << 8 | rxData[1];
        //         motorFeedback[canID - 1].motorAngle =  + 8192 * clampRotation - clampAngleOffset;
        //     } else {
        //         if (motorFeedback[5].rpm < 0 && newAngle - motorFeedback[5].motorAngle < 0) {
        //             clampRotation++;
        //         } else if (motorFeedback[canID - 1].rpm > 0 && newAngle - motorFeedback[5].motorAngle > 0) {
        //             clampRotation--;
        //         }
        //         motorFeedback[canID - 1].motorAngle = measured_motorAngle + 8192 * clampRotation - clampAngleOffset;

        //     }
        // }

        motorFeedback[canID - 1].torqueCurrent =
            rxData[4] << 8 | rxData[5];

        motorFeedback[canID - 1].temperature = rxData[6];
    }
    for (int canID = 1; canID <= DJI_MOTOR_COUNT; canID++) {
        motorFeedback[canID - 1].motorAngle += motorFeedback[canID - 1].rpm;
    
    }

}
/**
 * @todo
 */
float getEncoder(uint16_t canID) { return motorFeedback[canID - 1].motorAngle; }

/**
 * @todo
 */
float getRPM(uint16_t canID) { return motorFeedback[canID - 1].rpm; }

int32_t getMotorAngle(uint16_t canID) { return motorFeedback[canID - 1].motorAngle; }
/**
 * @todo
 */
void setWheelsOutput(float output, uint16_t canID)
{
    if (canID > 4) {return;}
    // if (output > maxCurrent)
    // {
    //     output = maxCurrent;
    // }
    // else if (output < -maxCurrent)
    // {
    //     output = -maxCurrent;
    // }
    
    output = output > maxCurrent ? maxCurrent : output;
    output = output < -maxCurrent ? -maxCurrent : output;
    
    // uint8_t mask                = 0xff;
    txWheelsData[(canID - 1) * 2 + 1] = (static_cast<int> (output));
    txWheelsData[(canID - 1) * 2]     = (static_cast<int> (output) >> 8);
}

void setClampsOutput(float output, uint16_t canID)
{
    if (canID <= 4) {return;}
    if (output > maxCurrent)
    {
        output = maxCurrent;
    }
    else if (output < -maxCurrent)
    {
        output = -maxCurrent;
    }
    // uint8_t mask                = 0xff; 
    txClampData[(canID - 5) * 2 + 1] = (static_cast<int> (output));
    txClampData[(canID - 5) * 2]     = (static_cast<int> (output) >> 8);
}

void setTargetClampAngle(uint16_t canID, uint16_t angle = 0) {
    targetClampAngle = getMotorAngle(canID) + angle;
}

/**
 * @todo
 */
void transmitWheels() { HAL_CAN_AddTxMessage(&hcan, &txWheelsHeader, txWheelsData, &mailbox); }

void transmitClamps() { HAL_CAN_AddTxMessage(&hcan, &txClampHeader, txClampData, &mailbox); }

//void callback() {}

}  // namespace DJIMotor
#endif