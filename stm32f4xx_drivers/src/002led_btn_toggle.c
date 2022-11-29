#include "stm32f407xx.h"
#include "stm32f407xx_gpio.h"

#define HIGH 1

void delay(void)
{
    for(uint32_t i = 0; i < 500000; i++);
}

int main(void)
{
  GPIO_Handle_t GpioLed, GpioBtn;

  GpioLed.pGPIOx = GPIOD;
  GpioBtn.pGPIOx = GPIOA;

  // exercise 1 use push pull configuration for the led pin
  GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
  GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
  GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
  GpioLed.GPIO_PinConfig.GPIO_PinAltFunMode = 0;

  // exercise 2 configure the button pin as input
  GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
  GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
  GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  GpioBtn.GPIO_PinConfig.GPIO_PinOPType = 0; // does not matter because it is input configuration.
  GpioBtn.GPIO_PinConfig.GPIO_PinAltFunMode = 0; // does not matter because not using alternate function.

  GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);
  GPIO_PeriClockControl(GpioBtn.pGPIOx, ENABLE);

  GPIO_Init(&GpioLed);
  GPIO_Init(&GpioBtn);

  while(1)
  {
    if (GPIO_ReadFromInputPin(GpioBtn.pGPIOx, GpioBtn.GPIO_PinConfig.GPIO_PinNumber) == HIGH)
    {
      GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber);
      delay();
    }
  }

  return 0;
}