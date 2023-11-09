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

// Declaring variables
static RcData rcData;
static MotorRPM motorRPM = {};
bool rcConnected = false;
uint32_t lastUpdatedTime;
uint32_t lastConnectedTime;

static HAL_StatusTypeDef status;

float maxMotorRPM = 3000;

// Internal function declarations
void setRPM(RcData);
void limitRPM(MotorRPM*);

/*================================================================================*/
/*You are free to declare your buffer, or implement your own function(callback, decoding) here*/
const uint8_t DR16_FRAME_LENGTH = 18;
uint8_t rcRxBuffer[DR16::DR16_FRAME_LENGTH];

const RcData *getRcData() { return &rcData;}

const MotorRPM *getMotorRPM() { return &motorRPM;}

void resetRcData() {
    rcData.channel0 = 1024;
    rcData.channel1 = 1024;
    rcData.channel2 = 1024;
    rcData.channel3 = 1024;
    rcData.s1 = 3;
    rcData.s2 = 3;
    motorRPM.motor0 = 0;
    motorRPM.motor1 = 0;
    motorRPM.motor2 = 0;
    motorRPM.motor3 = 0;
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
    decodeRcData(); // decode the controller data to rcData
    status = HAL_UARTEx_ReceiveToIdle_IT(huart, rcRxBuffer, DR16::DR16_FRAME_LENGTH); // start the next round of UART data reception
    getRcConnected();
    // reset the rcData if the data is invalid
    if (!validateRcData()) {
        resetRcData();
        return;
    }

    // update the last updated time
    lastUpdatedTime = HAL_GetTick();

    /**
     * @brief update the motor RPM here
     * @todo implement the function
     */
    setRPM(rcData);
}

bool getIsRcConnected() {
    uint32_t currentTime = HAL_GetTick();
    if ((lastUpdatedTime + 1000 > currentTime )) {
        lastConnectedTime = currentTime;
        rcConnected = true;
        
    } else {
        rcConnected = false;
        resetRcData();
    }
    return rcConnected;
}

const bool *getRcConnected() {
    getIsRcConnected();
    return &rcConnected;
}

// Max/Min ^ 2 * RPMConstant = maxMotorRPM
const float RPMConstant = maxMotorRPM / 10000;

// Find the Absolute Value of a Number
#define Abs(N) ((N<0)?(-N):(N))

/**
 * @brief Converts the signals from the DR16 controller to RPM
*/
void setRPM(RcData originalData) {
    // Convert the channel data into a range between -100 and 100
    // Also allows us to set the RPM easier with positive and negative numbers
    MotorRPM updateRPM {};
    int robotRotation = (originalData.channel0 - 1024)/6.6;
    int idkwhatthischannelwillbeusedfor = (originalData.channel1 - 1024)/6.6;
    int robotHorizontal = (originalData.channel2 - 1024)/6.6;
    int robotVertical = (originalData.channel3 - 1024)/6.6;

    // Forward and Backwards (Vertical) Motion, forward = positive
    int motor0Vertical = robotVertical * Abs(robotVertical) * RPMConstant;
    int motor1Vertical = - robotVertical * Abs(robotVertical) * RPMConstant;
    int motor2Vertical = robotVertical * Abs(robotVertical) * RPMConstant;
    int motor3Vertical = - robotVertical * Abs(robotVertical) * RPMConstant;

    // Left and Right (Horizontal) Motion, right = positive
    int motor0Horizontal = robotHorizontal * Abs(robotHorizontal) * RPMConstant;
    int motor1Horizontal = robotHorizontal * Abs(robotHorizontal) * RPMConstant;
    int motor2Horizontal = - robotHorizontal * Abs(robotHorizontal) * RPMConstant;
    int motor3Horizontal = - robotHorizontal * Abs(robotHorizontal) * RPMConstant;

    // Rotational Motion, clockwise = positive
    int motor0Rotational = robotRotation * Abs(robotRotation) * RPMConstant;
    int motor1Rotational = robotRotation * Abs(robotRotation) * RPMConstant;
    int motor2Rotational = robotRotation * Abs(robotRotation) * RPMConstant;
    int motor3Rotational = robotRotation * Abs(robotRotation) * RPMConstant;


    // Add all of the motor controls together
    // MotorRPM motorRPM;
    updateRPM.motor0 = motor0Horizontal + motor0Vertical + motor0Rotational;
    updateRPM.motor1 = motor1Horizontal + motor1Vertical + motor1Rotational;
    updateRPM.motor2 = motor2Horizontal + motor2Vertical + motor2Rotational;
    updateRPM.motor3 = motor3Horizontal + motor3Vertical + motor3Rotational;

    // Limit the calculated values and transmit to the motors
    limitRPM(&updateRPM);
    motorRPM.motor0 = updateRPM.motor0;
    motorRPM.motor1 = updateRPM.motor1;
    motorRPM.motor2 = updateRPM.motor2;
    motorRPM.motor3 = updateRPM.motor3;
}


/**
 * @brief Limit the RPM if RPM is > maxMotorRPM
 * @brief Uses the global variable maxMotorRPM and MotorRPM struct
 * 
 */
void limitRPM(MotorRPM* inputRPM) {

    // get the highest RPM out of all the motors
    float maxRPM = Abs(inputRPM->motor0);
    if (Abs(inputRPM->motor1) > maxRPM) {
        maxRPM = inputRPM->motor1;
    } else if (Abs(inputRPM->motor2) > maxRPM) {
        maxRPM = inputRPM->motor2;
    } else if (Abs(inputRPM->motor3) > maxRPM) {
        maxRPM = inputRPM->motor3;
    }
    maxRPM = Abs(maxRPM);
    // limit the RPM if the highest RPM is higher than the maxMotorRPM
    if (maxRPM > maxMotorRPM) {
        inputRPM->motor0 = inputRPM->motor0 / maxRPM * maxMotorRPM;
        inputRPM->motor1 = inputRPM->motor1 / maxRPM * maxMotorRPM;
        inputRPM->motor2 = inputRPM->motor2 / maxRPM * maxMotorRPM;
        inputRPM->motor3 = inputRPM->motor3 / maxRPM * maxMotorRPM;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    HAL_UART_Abort_IT(huart);

    rcConnected = false;
    resetRcData();
    HAL_UARTEx_ReceiveToIdle_IT(huart, rcRxBuffer, DR16::DR16_FRAME_LENGTH);
}

/*================================================================================*/
void init()
{
    /*If you would like to, please implement your function definition here*/
    resetRcData();
    huart1.ErrorCallback = HAL_UART_ErrorCallback;
    HAL_UART_RegisterRxEventCallback(&huart1, rxEventCallback);
    status = HAL_UARTEx_ReceiveToIdle_IT(&huart1, rcRxBuffer, DR16_FRAME_LENGTH);
}

}  // namespace DR16

#endif