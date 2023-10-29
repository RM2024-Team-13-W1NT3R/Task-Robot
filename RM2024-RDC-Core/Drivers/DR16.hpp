/**
 * @file DR16.hpp template
 * @author - GUO, Zilin
 *         - Your Name 
 * @brief  This is the DR16 template file directly copy from the PA2
 * @date 2023-10-18
 *
 * @copyright This file is only for HKUST Enterprize RM2024 internal
 * competition. All Rights Reserved.
 */
#pragma once // In case of multiple inclusion
#include "AppConfig.h"
#include "stdint.h"
#include "usart.h"
#if USE_DR16
namespace DR16
{

const float maxMotorRPM = 50; // defin

/**
 * @brief the DR16 remote controller structure
 *
 * @param channel0 The channel0 data from the remote controller
 * @param channel1 The channel1 data from the remote controller
 * @param channel2 The channel2 data from the remote controller
 * @param channel3 The channel3 data from the remote controller
 *
 * @param s1       The switch 1 data from the remote controller
 * @param s2       The switch 2 data from the remote controller
 */
typedef struct
{
    uint16_t channel0 : 11;  // Channel 0 data
    uint16_t channel1 : 11;  // Channel 1 data
    uint16_t channel2 : 11;  // Channel 2 data
    uint16_t channel3 : 11;  // Channel 3 data

    uint8_t s1 : 2;  // Switch 1 data
    uint8_t s2 : 2;  // Switch 2 data
} RcData;

typedef struct
{
    float motor0;
    float motor1;
    float motor2;
    float motor3;
} MotorRPM;

/**
 * @brief Access the decoded remote controller datat by this API
 * @remark You are recommended to:
 *         - Return a constant pointer of the decoded remote controller data
 * in this fucntion
 */
const RcData *getRcData();

/*===========================================================================*/
/*You can declare your own function here, supposing you would like to desgin a
 * more complicated DR16 module*/


/**
 * @brief Get the status of the remote controller
 * 
 * @return true if the remote controller is connected
 * @return false if the remote controller is not connected
 */
const bool *getRcConnected();

/*===========================================================================*/

/**
 * @brief The initialization of your assingment DR16 Module
 * @remark You are recommended to:
 *         - Register your callback in the function
 *         - Start the first round of the UART data reception
 * @remark If you wish to, please implement the function body in the DR16.cpp
 * file
 */
void init();

}  // namespace DR16
#endif  // DR16_UART