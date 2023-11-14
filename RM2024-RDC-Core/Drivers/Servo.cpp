#include "Servo.hpp"
#include "tim.h"

#if USE_Servo
#include "tim.h"



namespace Servo
{   
    //extern TIM_HandleTypeDef htim2;
   void ServoInit()
   {
     HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3); 
     HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
    }
    

    void pickup ()
    {        
        __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,290);//90 degree
        __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,724);//180 degree
    }
    void putdown()
    {
        __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,290);
        
    }

}
#endif

