#include "Ultrasonic.hpp"

#define triggerPinPORT GPIOC
#define triggerPin GPIO_PIN_13

namespace Ultrasonic 
{

void triggerStart() {
    HAL_GPIO_WritePin(triggerPinPORT, triggerPin, GPIO_PIN_SET);
}

bool getDistance(uint32_t distance) {

}

}