#include "Ultrasonic.hpp"

#define triggerPinPort GPIOC
#define triggerPin GPIO_PIN_13
#define echoPinPort GPIOC
#define echoPin GPIO_PIN_14

uint32_t timeUntilSent;

static volatile GPIO_PinState echoPinState;


namespace Ultrasonic 
{

void triggerStart() {
    HAL_GPIO_WritePin(triggerPinPort, triggerPin, GPIO_PIN_SET);
}

bool getDistance(uint32_t* distance) {
    echoPinState = HAL_GPIO_ReadPin(echoPinPort, echoPin);

}

}