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
  GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FE; // falling edge
  GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
  GpioBtn.GPIO_PinConfig.GPIO_PinOPType = 0; // Input mode not output
  GpioBtn.GPIO_PinConfig.GPIO_PinAltFunMode = 0; // Not using alternate function

  // Enable clock for GPIOA and GPIOB
  GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);
  GPIO_PeriClockControl(GpioBtn.pGPIOx, ENABLE);

  // Initialize the GPIO pin
  GPIO_Init(&GpioLed);
  GPIO_Init(&GpioBtn);

  // IRQ configurations
  GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15); // not required because only one IRQ (interrupt request) is used
  GPIO_IRQConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15, ENABLE);

  // Enable the EXTI line
  // EXTI_IRQConfig(EXTI_IMR_MR0, ENABLE);
  
  // Infinite loop
  while(1);


  return 0;
}

// Interrupt handler override
void EXTI15_10_IRQHandler(void)
{
    delay();
    GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
    GPIO_IRQHandler(GPIO_PIN_NO_12);
}