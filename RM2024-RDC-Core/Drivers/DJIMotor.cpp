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


// This function is to set a target angle for our clamp rotation
int32_t* getTargetClampAngle() {
    return &targetClampAngle;
}

// declartion of feedbacks, data buffers
DJIMotor motorFeedback[DJI_MOTOR_COUNT];
uint32_t mailbox;
uint8_t rxData[8];
uint8_t txWheelsData[8];
uint8_t txClampData[8];

// used for calculation for angle (but not not in use)
uint8_t clampRotation = 5;
uint32_t clampAngleOffset = 0;
bool clampRotationCalibrated = false;

// buffer for calculating angle
int16_t measured_motorAngle = 0;
int32_t newAngle;

// used for transmission headers
CAN_TxHeaderTypeDef txWheelsHeader = {0x200, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};
CAN_TxHeaderTypeDef txClampHeader  = {0x1ff, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};
CAN_RxHeaderTypeDef rxHeader;

// can filter used for receiving messages
CAN_FilterTypeDef filter[DJI_MOTOR_COUNT];

// Chassis motor filters
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

// Robotic arm filters
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

// getting the torque current of a motor     
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

// initialize the motor controller's filter to receive messages
void init()
{
    HAL_CAN_ConfigFilter(&hcan, &filterlist);
    HAL_CAN_ConfigFilter(&hcan, &filterlist1);
    HAL_CAN_Start(&hcan);
}

// initialize the motor controller's recieving fifo
uint32_t fifoLevel;
uint32_t curFifoLevel;

// getting the feedback of motor (maximum 3 motors at once)
void getRxMessage()
{
    // Get the number of messages available
    fifoLevel = HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);

    // Get and process each message
    for (int i = 0; i < fifoLevel; i++) {
        HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, rxData);
        curFifoLevel = HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);
        uint16_t canID = rxHeader.StdId - 0x200;

        // Check if the CAN ID is valid
        if (canID < 1 || canID > DJI_MOTOR_COUNT) {
            continue;
        }

        // Check if the data length matches the expected length
        if (rxHeader.DLC != 8) {
            continue;
        }
        
        motorFeedback[canID - 1].canID = canID;

        motorFeedback[canID - 1].rpm =
            rxData[2] << 8 | rxData[3];

        motorFeedback[canID - 1].torqueCurrent =
            rxData[4] << 8 | rxData[5];

        motorFeedback[canID - 1].temperature = rxData[6];
    }

    // Self defined motor angle variable for use in clamp rotation
    for (int canID = 1; canID <= DJI_MOTOR_COUNT; canID++) {
        motorFeedback[canID - 1].motorAngle += motorFeedback[canID - 1].rpm;
    
    }

}
/**
 * @todo
 */

// getting the angle of a motor
float getEncoder(uint16_t canID) { return motorFeedback[canID - 1].motorAngle; }

/**
 * @todo
 */

// getting the rpm of a motor
float getRPM(uint16_t canID) { return motorFeedback[canID - 1].rpm; }

// getting the angle of a motor
int32_t getMotorAngle(uint16_t canID) { return motorFeedback[canID - 1].motorAngle; }
/**
 * @todo
 */

// setting the output of a motor
void setWheelsOutput(float output, uint16_t canID)
{
    // Only the chassis motors are used
    if (canID > 4) {return;}
    
    // Limiting the output current
    output = output > maxCurrent ? maxCurrent : output;
    output = output < -maxCurrent ? -maxCurrent : output;
    
    // Sending data into the CAN bus
    txWheelsData[(canID - 1) * 2 + 1] = (static_cast<int> (output));
    txWheelsData[(canID - 1) * 2]     = (static_cast<int> (output) >> 8);
}

// setting the output of a motor
void setClampsOutput(float output, uint16_t canID)
{
    // Only the clamp motors are used
    if (canID <= 4) {return;}

    // Limiting the output current
    output = output > maxCurrent ? maxCurrent : output;
    output = output < -maxCurrent ? -maxCurrent : output;

    // Sending data into the CAN bus
    txClampData[(canID - 5) * 2 + 1] = (static_cast<int> (output));
    txClampData[(canID - 5) * 2]     = (static_cast<int> (output) >> 8);
}

// setting the target angle of a motor
void setTargetClampAngle(uint16_t canID, uint16_t angle = 0) {
    targetClampAngle = getMotorAngle(canID) + angle;
}

/**
 * @todo
 */

// transmit the set current to the motor controller
void transmitWheels() { HAL_CAN_AddTxMessage(&hcan, &txWheelsHeader, txWheelsData, &mailbox); }

// transmit the set current to the motor controller
void transmitClamps() { HAL_CAN_AddTxMessage(&hcan, &txClampHeader, txClampData, &mailbox); }

//void callback() {}

}  // namespace DJIMotor
#endif