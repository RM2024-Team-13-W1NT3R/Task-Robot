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

static volatile float kp = 10.0f;
static volatile float ki = 15.0f;
static volatile float kd = 0.025f;
static volatile float kp1 = 1.0f;
static volatile float ki1 = 2.0f;
static volatile float kd1 = 0.025f;
static volatile float kp2 = 1.0f;
static volatile float ki2 = 2.0f;
static volatile float kd2 = 0.025f;
static volatile float kp3 = 1.0f;
static volatile float ki3 = 2.0f;
static volatile float kd3 = 0.025f;
static volatile float kp4 = 1.0f;
static volatile float ki4 = 2.0f;
static volatile float kd4 = 0.025f;
static volatile float udkp = 0.25f;
static volatile float udki = 2.0f;
static volatile float udkd = 0.025f;
static volatile float anglekp = 5.0f;
static volatile float angleki = 1.0f;
static volatile float anglekd = 0.025f;

static volatile uint16_t canID = 0;
Control::PID pid[6]{{kp, ki, kd}, {kp, ki, kd}, {kp, ki, kd}, {kp, ki, kd}, {udkp, udki, udkd}, {anglekp, angleki, anglekd}};
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
        DR16::getRcConnected();
        bool status = DJIMotor::getRxMessage(canID);
        // taskENTER_CRITICAL();
        for (canID = 1; canID <= 5; canID++)
        {
            // if (!status)
            // {
            //     continue; // skip modulating when receiving data failed
            // }

            float targetMotorOutput[6];
            float targetCurrent = 0;

  
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
                // if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET && targetMotorOutput[5] < 0) {
                //     targetCurrent = 0;
                // }
                // DJIMotor::setClampOutput(targetCurrent, canID);
                break;
            case 6:
                if (*resetTargetClamp) {
                    DJIMotor::setTargetClampAngle(DJIMotor::getMotorAngle(canID));
                    *resetTargetClamp = false;
                    // do pid rpm for angle
                } else {
                    targetMotorOutput[5] = DR16::getMotorRPM()->clampMotor;
                    targetCurrent = pid[canID - 1].update(*DJIMotor::getTargetClampAngle(), DJIMotor::getMotorAngle(canID));
                    targetCurrent = 0;
                }
                
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
        DJIMotor::transmitWheels();
        DJIMotor::transmitClamps();
        // taskEXIT_CRITICAL();
        /* Your user layer codes in loop end here*/
        /*=================================================*/


        vTaskDelay(1);  // Delay and block the task for 1ms.
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
    /**
     * @todo Add your own task here
     */
}
