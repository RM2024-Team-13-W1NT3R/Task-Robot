#include "Servo.hpp"
#include "tim.h"

#if USE_Servo
#include "tim.h"



namespace Servo
{   
    //extern TIM_HandleTypeDef htim2;
   void ServoInit()
   {
     HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2); 
     HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
    }
    

    void pickup()
    {        
        __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,110);
        __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,40);
        __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,110);
        __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,40);
    }
    void putdown()
    {
        __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,110);
        __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,110);
        
    }

}
#endif

