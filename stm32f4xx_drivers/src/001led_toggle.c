#include "stm32f407xx.h"
#include "stm32f407xx_gpio.h"

void delay(void)
{
    for(uint32_t i = 0 ; i < 500000/2 ; i ++);
    // for (uint32_t i = 0; i < 500000; i++);
}

int main(void)
{
  GPIO_Handle_t GpioLed;
  //GPIO_Handle_t GpioLed;

  GpioLed.pGPIOx = GPIOD;

  // exercise 1 use push pull configuration for the led pin
  GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
  GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
  GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // push pull does not require pull up / pull down resistors, could use open drain with pu pd external resistor.
  GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  GpioLed.GPIO_PinConfig.GPIO_PinAltFunMode = 0;

  GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);
  GPIO_Init(&GpioLed);
  GPIO_WriteToOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);

  while (1)
  {
    GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber);
    // for (uint32_t i = 0; i < 500000; i++);
    delay();
  }

  // exercise 2 use open drain configuration for the led pin
  

  // exercise 3 use pull up configuration for the led pin

  return 0;
}