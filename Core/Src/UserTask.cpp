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

/*Allocate the stack for our PID task*/
StackType_t uxPIDTaskStack[configMINIMAL_STACK_SIZE];
/*Declare the PCB for our PID task*/
StaticTask_t xPIDTaskTCB;

static volatile float kp = 8.0f;
static volatile float ki = 0.0f;
static volatile float kd = 0.0f;

/**
 * @todo Show your control outcome of the M3508 motor as follows
 */
void userTask(void *)
{
    /* Your user layer codes begin here*/
    /*=================================================*/

    Control::PID pid[4]{{kp, ki, kd}, {kp, ki, kd}, {kp, ki, kd}, {kp, ki, kd}};
    /* Your user layer codes end here*/
    /*=================================================*/
    while (true)
    {
        /* Your user layer codes in loop begin here*/
        /*=================================================*/
        for (uint16_t canID = 1; canID <= 4; canID++)
        {
            
            DJIMotor::getEncoder(canID);
            float targetRPM{};
            switch (canID)
            {
            case 1:
                targetRPM = DR16::getMotorRPM()->motor0;
                break;
            case 2:
                targetRPM = DR16::getMotorRPM()->motor1;
                break;
            case 3:
                targetRPM = DR16::getMotorRPM()->motor2;
                break;
            case 4:
                targetRPM = DR16::getMotorRPM()->motor3;
                break;
            default:
                break;
            }
            float targetCurrent =
                pid[canID - 1].update(targetRPM, DJIMotor::getRPM(canID));

            DJIMotor::setOutput(targetCurrent, canID);
        }
        DJIMotor::transmit();
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

    xTaskCreateStatic(userTask,
                      "user_default ",
                      configMINIMAL_STACK_SIZE,
                      NULL,
                      1,
                      uxPIDTaskStack,
                      &xPIDTaskTCB);  // Add the main task into the scheduler
    /**
     * @todo Add your own task here
     */
}
