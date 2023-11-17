/**
 * @file UserTask.cpp
 * @author JIANG Yicheng  RM2024 (EthenJ@outlook.sg)
 * @author GUO, Zilin
 * @brief RDC user task files
 * @version 0.3
 * @date 2022-10-20
 *
 * @copyright Copyright (c) 2024
 */
#include "AppConfig.h"   // Include our customized configuration
#include "DJIMotor.hpp"  // Include DR16
#include "DR16.hpp"
#include "FreeRTOS.h"  // Include FreeRTOS.h
#include "PID.hpp"     // Include PID
#include "main.h"
#include "task.h"  // Include task
#include "Servo.hpp"

/*Allocate the stack for our PID task*/
StackType_t uxPIDTaskStack[512];
/*Declare the PCB for our PID task*/
StaticTask_t xPIDTaskTCB;

StackType_t uxRxTaskStack[512];
StaticTask_t xRxTaskTCB;

static volatile float kp = 1.0f;
static volatile float ki = 5.0f;
static volatile float kd = 0.02f;
static volatile float udkp = 1.0f;
static volatile float udki = 5.0f;
static volatile float udkd = 0.025f;
static volatile float anglekp = 10.0f;
static volatile float angleki = 5.0f;
static volatile float anglekd = 0.02f;
static volatile float angleStaykp = 0.5f;
static volatile float angleStayki = 0.5f;
static volatile float angleStaykd = 0.02f;
bool started = false;
bool stayDown = true;

float volatile angleCurrent;
static volatile float targetAngleTest = 6000;

static float targetCurrent = -2000;

static int32_t curAngle;

static int goingUpTime;
static volatile bool overCurrent = false;
static volatile uint16_t canID = 0;
Control::PID pid[6] {{kp, ki, kd}, {kp, ki, kd}, {kp, ki, kd}, {kp, ki, kd}, {udkp, udki, udkd}, {anglekp, angleki, anglekd}};
Control::PID angleStayPID {angleStaykp, angleStayki, angleStaykd};
/**
 * @todo Show your control outcome of the M3508 motor as follows
 */
void userTask(void *)
{
    /* Your user layer codes begin here*/
    /*=================================================*/
    bool* resetTargetClamp = DR16::getResetAngle();

    /* Your user layer codes end here*/
    /*=================================================*/
    while (true)
    {

        /* Your user layer codes in loop begin here*/
        /*=================================================*/
        goingUpTime = HAL_GetTick();
        DR16::getRcConnected();
        // taskENTER_CRITICAL();
        if (*DR16::getAutoTrackEnabled()) {
            AutoTrack::executeMovement(*DR16::getLeftMode());
        } else {

        
        for (canID = 1; canID <= 6; canID++)
        {
            // if (!status)
            // {
            //     continue; // skip modulating when receiving data failed
            // }

            float targetMotorOutput[6];
            // float targetAngleCurrent = 4000;
            float rpmCurrent = 0;
  
            switch (canID)
            {
            case 1:
                targetMotorOutput[0] = DR16::getMotorRPM()->motor0;
                targetCurrent = pid[canID - 1].update(targetMotorOutput[canID - 1], DJIMotor::getRPM(canID));
                break;
            case 2:
                targetMotorOutput[1] = DR16::getMotorRPM()->motor1;
                targetCurrent = pid[canID - 1].update(targetMotorOutput[canID - 1], DJIMotor::getRPM(canID));
                break;
            case 3:
                targetMotorOutput[2] = DR16::getMotorRPM()->motor2;
                targetCurrent = pid[canID - 1].update(targetMotorOutput[canID - 1], DJIMotor::getRPM(canID));
                break;
            case 4:
                targetMotorOutput[3] = DR16::getMotorRPM()->motor3;
                targetCurrent = pid[canID - 1].update(targetMotorOutput[canID - 1], DJIMotor::getRPM(canID));
                break;
            case 5:
                targetMotorOutput[4] = DR16::getMotorRPM()->updownMotor;
                targetCurrent = pid[canID - 1].update(targetMotorOutput[canID - 1], DJIMotor::getRPM(canID)); 
                if (HAL_GPIO_ReadPin(SWITCH_TOP_GPIO_Port, SWITCH_TOP_Pin) == GPIO_PIN_RESET && targetCurrent < 0) {
                    targetCurrent = 0;
                }
                if (HAL_GPIO_ReadPin(SWITCH_BOT_GPIO_Port, SWITCH_BOT_Pin) == GPIO_PIN_RESET && targetCurrent > 0) {
                    targetCurrent = 0;
                }
                // DJIMotor::setClampOutput(targetCurrent, canID);
                break;
            case 6:
                angleCurrent = angleStayPID.update(*DJIMotor::getTargetClampAngle(), DJIMotor::getMotorAngle(canID));
                curAngle = *DJIMotor::getTargetClampAngle();
                
                angleCurrent = angleCurrent > 2000 ? 2000 : angleCurrent;
                // angleCurrent = angleCurrent < -16834 / 2 ? -16834 / 2 : angleCurrent;

                if (*resetTargetClamp) {
                    targetCurrent = 0;
                    DJIMotor::setTargetClampAngle(canID, 0);
                    // *resetTargetClamp = false;
                    rpmCurrent = pid[canID - 1].update(DR16::getMotorRPM()->clampMotor, DJIMotor::getRPM(canID));
                    rpmCurrent = rpmCurrent > 3000 ? 3000 : rpmCurrent;
                    targetCurrent = rpmCurrent;
                    
                    // do pid rpm for angle
                } else {
                    // angleCurrent = angleStayPID.update(*DJIMotor::getTargetClampAngle(), DJIMotor::getMotorAngle(canID));
                    targetCurrent = angleCurrent;
                }

                // targetCurrent = angleStayPID.update(DR16::getMotorRPM()->clampMotor, DJIMotor::getMotorAngle(canID));
//  

                // targetCurrent = angleStayPID.update(*DJIMotor::getTargetClampAngle(), DJIMotor::getMotorAngle(canID));

                // targetMotorOutput[5] = DR16::getMotorRPM()->clampMotor;
                // targetCurrent = pid[canID - 1].update(targetMotorOutput[5], DJIMotor::getRPM(canID));
                // targetCurrent = 0;
            //     if (targetCurrent > -4000) {
            //         targetCurrent--;
            //     }
                // if (DR16::getMotorRPM()->clampMotor > 0) {
                //     stayDown = false;
                //     if (!started) {
                //         targetCurrent = -3000;
                //         started = true;
                //     } else {
                //         targetCurrent = -6000;
                //     }
                // } else if (DR16::getMotorRPM()->clampMotor == 0) {
                //     if (stayDown) {
                //         targetCurrent = -1000;
                //     } else {
                //         targetCurrent = -4500;
                //     }
                // } else if (DR16::getMotorRPM()->clampMotor < 0) {
                //     targetCurrent = -1000;
                //     stayDown = true;
                // }
                break;
                 
            default: 
                break;
            }
            if (canID <= 4)
            {
            DJIMotor::setWheelsOutput(targetCurrent, canID);
            } else {
            DJIMotor::setClampsOutput(targetCurrent, canID);
            }
            

   
            
            
        } 
        }
        DJIMotor::transmitWheels();
        DJIMotor::transmitClamps();
        // taskEXIT_CRITICAL();
        /* Your user layer codes in loop end here*/
        /*=================================================*/

        // Servo::putdown();
        vTaskDelay(1);  // Delay and block the task for 1ms.
    }
}

void RxDataReceive(void *) {
    // DJIMotor::getRxMessage();
    // DJIMotor::setTargetClampAngle(canID, 0);
    while (true) {
        DJIMotor::getRxMessage();
        vTaskDelay(1);
    }
}

/**
 * @todo In case you like it, please implement your own tasks
 */

/**
 * @brief Intialize all the drivers and add task to the scheduler
 * @todo  Add your own task in this file
 */
void startUserTasks()
{
    DJIMotor::init();  // Initalize the DJIMotor driver
    DR16::init();      // Intialize the DR16 driver
    Servo::ServoInit();

    xTaskCreateStatic(userTask,
                      "user_default ",
                      configMINIMAL_STACK_SIZE,
                      NULL,
                      15,
                      uxPIDTaskStack,
                      &xPIDTaskTCB);  // Add the main task into the scheduler

    xTaskCreateStatic(RxDataReceive,
                        "user_default ",
                        configMINIMAL_STACK_SIZE,
                        NULL,
                        15,
                        uxRxTaskStack,
                        &xRxTaskTCB);  // Add the main task into the scheduler
    /**
     * @todo Add your own task here
     */
}
