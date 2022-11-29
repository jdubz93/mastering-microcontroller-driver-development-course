#ifndef INC_STM32F407XX_GPIO_H_
#define INC_STM32F407XX_GPIO_H_

// Includes
#include <stdint.h>
#include "stm32f407xx.h"

#define __vo volatile

typedef struct {
    uint8_t GPIO_PinNumber;     // This holds the pin number of the given GPIO pin |Range: GPIO_PIN_NO_0 - GPIO_PIN_NO_15 | Ref: @GPIO_PIN_NUMBERS in stm32f407xx_gpio_driver.h
    uint8_t GPIO_PinMode;       // This holds the pin mode of the given GPIO pin | Ref: @GPIO_PIN_MODES in stm32f407xx_gpio_driver.h
    uint8_t GPIO_PinSpeed;      // This holds the pin speed of the given GPIO pin
    uint8_t GPIO_PinPuPdControl;// This holds the pin pull-up/pull-down configuration of the given GPIO pin
    uint8_t GPIO_PinOPType;     // This holds the pin output type of the given GPIO pin
    uint8_t GPIO_PinAltFunMode; // This holds the pin alternate function mode of the given GPIO pin
} GPIO_PinConfig_t;

typedef struct {
    // pointer to hold the base address of the GPIO peripheral
    __vo GPIO_RegDef_t *pGPIOx; // This is going to hold the base address of the GPIO port to which the pin belongs
    GPIO_PinConfig_t GPIO_PinConfig; // This holds GPIO pin configuration settings
} GPIO_Handle_t;

/* **********************************************************************************
 *                          APIS supported by this driver                           *
 *      For more information about the APIs check the function definitions          *
 * **********************************************************************************/

/* 
 * Peripheral Clock Setup (En = Enable, Di = Disable)
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);


/* 
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


/* 
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandler(uint8_t PinNumber);

void GPIO_IRQInterruptConfig(void);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/*
 * Misc. Macros
 */

#define GPIO_PIN_SET    1
#define GPIO_PIN_RESET  0

/*
 * @GPIO_PIN_NUMBERS
 * GPIO Pin Numbers
 */
#define GPIO_PIN_NO_0   0
#define GPIO_PIN_NO_1   1
#define GPIO_PIN_NO_2   2
#define GPIO_PIN_NO_3   3
#define GPIO_PIN_NO_4   4
#define GPIO_PIN_NO_5   5
#define GPIO_PIN_NO_6   6
#define GPIO_PIN_NO_7   7
#define GPIO_PIN_NO_8   8
#define GPIO_PIN_NO_9   9
#define GPIO_PIN_NO_10  10
#define GPIO_PIN_NO_11  11
#define GPIO_PIN_NO_12  12
#define GPIO_PIN_NO_13  13
#define GPIO_PIN_NO_14  14
#define GPIO_PIN_NO_15  15

/*
 * @GPIO_PIN_MODES
 * GPIO pin modes
 */
#define GPIO_MODE_IN        0 // Input mode
#define GPIO_MODE_OUT       1 // Output mode
#define GPIO_MODE_ALTFN     2 // Alternate Function
#define GPIO_MODE_ANALOG    3 // Analog mode is not supported on all GPIO pins
#define GPIO_MODE_IT_FE     4 // interrupt trigger Falling edge
#define GPIO_MODE_IT_RE     5 // interrupt trigger Rising edge
#define GPIO_MODE_IT_RFE    6 // interrupt trigger Rising Falling edge

/*
 * GPIO pin output types
 */
#define GPIO_OP_TYPE_PP     0 // Output push-pull // default and does not require pull up / pull down resistors
#define GPIO_OP_TYPE_OD     1 // Output open-drain // requires pull up / pull down resistors

/*
 * GPIO pin output speeds
 */
#define GPIO_SPEED_LOW      0 // Low speed
#define GPIO_SPEED_MEDIUM   1 // Medium speed
#define GPIO_SPEED_FAST     2 // Fast speed
#define GPIO_SPEED_HIGH     3 // High speed

/*
 * GPIO pin pull-up/pull-down configuration macros
 */
#define GPIO_NO_PUPD        0 // No pull-up, pull-down
#define GPIO_PIN_PU         1 // Pull-up
#define GPIO_PIN_PD         2 // Pull-down


#endif /* INC_STM32F407XX_GPIO_H_ */