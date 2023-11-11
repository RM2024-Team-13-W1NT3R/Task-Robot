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
#include "tim.h"


/*Allocate the stack for our PID task*/
StackType_t uxPIDTaskStack[512];
/*Declare the PCB for our PID task*/
StaticTask_t xPIDTaskTCB;
StackType_t uxGM995taskStack[128];
StaticTask_t xGM995taskTCB;

static volatile float kp = 1.0f;
static volatile float ki = 2.0f;
static volatile float kd = 0.025f;
static volatile uint16_t canID = 0;
Control::PID pid[4]{{kp, ki, kd}, {kp, ki, kd}, {kp, ki, kd}, {kp, ki, kd}};
/**
 * @todo Show your control outcome of the M3508 motor as follows
 */
void userTask(void *)
{
    /* Your user layer codes begin here*/
    /*=================================================*/

    
    /* Your user layer codes end here*/
    /*=================================================*/
    while (true)
    {

        /* Your user layer codes in loop begin here*/
        /*=================================================*/
        DR16::getRcConnected();
        taskENTER_CRITICAL();
        for (canID = 1; canID <= 4; canID++)
        {
            bool status = DJIMotor::getRxMessage(canID);
            // if (!status)
            // {
            //     continue; // skip modulating when receiving data failed
            // }

            float targetRPM[4];
            switch (canID)
            {
            case 1:
                targetRPM[0] = DR16::getMotorRPM()->motor0;
                break;
            case 2:
                targetRPM[1] = DR16::getMotorRPM()->motor1;
                break;
            case 3:
                targetRPM[2] = DR16::getMotorRPM()->motor2;
                break;
            case 4:
                targetRPM[3] = DR16::getMotorRPM()->motor3;
                break;
            default:
                break;
            }
            float targetCurrent =
                pid[canID - 1].update(targetRPM[canID - 1], DJIMotor::getRPM(canID));

            DJIMotor::setOutput(targetCurrent, canID);

        }
        DJIMotor::transmit();
        taskEXIT_CRITICAL();
        /* Your user layer codes in loop end here*/
        /*=================================================*/

        vTaskDelay(1);  // Delay and block the task for 1ms.
    }
}
int angleToCCR(float angle)
{
  const int minAngle = 0;
  const int maxAngle = 180;
  const int minPulseWidth = 500;  // 最小脉冲宽度（微秒）
  const int maxPulseWidth = 2500; // 最大脉冲宽度（微秒）
  const int minCCR = 0;           // 最小CCR值
  const int maxCCR = 199;         // 最大CCR值（根据计数器周期）

  int pulseWidth = minPulseWidth + (angle - minAngle) * (maxPulseWidth - minPulseWidth) / (maxAngle - minAngle);
  int ccr = minCCR + (pulseWidth - minPulseWidth) * (maxCCR - minCCR) / (maxPulseWidth - minPulseWidth);

  return ccr;
}

void GM995task(void *)
{ 
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);//enable PWM output of tim3
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
    int ccr = angleToCCR(180);
    while (1)
    {
        __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,ccr);
        vTaskDelay(2000);
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
                      15,
                      uxPIDTaskStack,
                      &xPIDTaskTCB); // Add the main task into the scheduler
    xTaskCreateStatic(GM995task,
                      "GM995task ",
                      128,
                      NULL,
                      10,
                      uxGM995taskStack,
                      &xGM995taskTCB);
    /**
     * @todo Add your own task here
     */
}
