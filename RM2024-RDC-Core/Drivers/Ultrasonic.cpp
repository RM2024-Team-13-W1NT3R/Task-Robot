#include "Ultrasonic.hpp"

#define triggerPinPort GPIOC
#define triggerPin GPIO_PIN_13
#define echoPinPort GPIOC
#define echoPin GPIO_PIN_14
#define soundSpeed 343 // m/s

uint32_t timeUntilSent;

static volatile GPIO_PinState echoPinState;


namespace Ultrasonic 
{

void triggerStart() {
    HAL_GPIO_WritePin(triggerPinPort, triggerPin, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(triggerPinPort, triggerPin, GPIO_PIN_RESET);
}

uint32_t getDuration() {
    uint32_t start, stop;
    while (HAL_GPIO_ReadPin(echoPinPort, echoPin) == GPIO_PIN_RESET);
    start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(echoPinPort, echoPin) == GPIO_PIN_SET);
    stop = HAL_GetTick();
    return stop - start;
}

bool getDistance(uint32_t duration) {
    // echoPinState = HAL_GPIO_ReadPin(echoPinPort, echoPin); 
    float distance = ((float)duration / 2) * soundSpeed;
    return distance;
}

int main() {

    triggerStart();
    uint32_t duration = getDuration();
    float distance = getDistance(duration);
    return 0;
}


    // namespace Ultrasonic
}