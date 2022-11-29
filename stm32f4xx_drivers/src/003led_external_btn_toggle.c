#include "stm32f407xx.h"
#include "stm32f407xx_gpio.h"
#include <string.h>

#define HIGH 1
#define LOW 0

void delay(void)
{
    for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{

  GPIO_Handle_t GpioLed, GpioBtn;
  // initialize every member element of the struct to 0
  memset(&GpioLed, 0, sizeof(GpioLed));
  memset(&GpioBtn, 0, sizeof(GpioBtn));

  /* On board LED */
  // GpioLed.pGPIOx = GPIOD;
  // GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
  // GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
  // GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  // GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  // GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
  // GpioLed.GPIO_PinConfig.GPIO_PinAltFunMode = 0;

  /* External LED */
  GpioLed.pGPIOx = GPIOA;
  GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
  GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
  GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
  GpioLed.GPIO_PinConfig.GPIO_PinAltFunMode = 0;

  /* External BTN */
  GpioBtn.pGPIOx = GPIOB;
  GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
  GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
  GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
  GpioBtn.GPIO_PinConfig.GPIO_PinOPType = 0; // Input mode not output
  GpioBtn.GPIO_PinConfig.GPIO_PinAltFunMode = 0; // Not using alternate function

  GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);
  GPIO_PeriClockControl(GpioBtn.pGPIOx, ENABLE);

  GPIO_Init(&GpioLed);
  GPIO_Init(&GpioBtn);

  while(1)
  {
    if (GPIO_ReadFromInputPin(GpioBtn.pGPIOx, GpioBtn.GPIO_PinConfig.GPIO_PinNumber) == LOW)
    {
      GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber);
      delay();
    }
  }

    return 0;
}