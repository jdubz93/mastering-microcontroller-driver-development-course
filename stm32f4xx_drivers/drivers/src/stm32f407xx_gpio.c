#include "stm32f407xx_gpio.h"

/******************************************************************************************
 * @fn       - GPIO_PeriClockControl                                                      *
 * @brief    - This function enables or disables peripheral clock for the given GPIO port *
 * @param1   - Base address of the GPIO peripheral                                        *
 * @param2   - Enable or Disable macro                                                    *
 * @return   - none                                                                       *
 * @Note     - none                                                                       *
 * ****************************************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_EN();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_EN();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_EN();
        }
        else if (pGPIOx == GPIOI)
        {
            GPIOI_PCLK_EN();
        }
        else
        {
            // Invalid GPIO port
        }
    }
    else
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DI();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DI();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DI();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_DI();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_DI();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_DI();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_DI();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_DI();
        }
        else if (pGPIOx == GPIOI)
        {
            GPIOI_PCLK_DI();
        }
        else
        {
            // Invalid GPIO port
        }
    }
}


/************************************************************
 * @fn       - GPIO_Init                                    *
 * @brief    - This function initializes the given GPIO pin *
 * @param1   - Pointer to the GPIO handle structure         *
 * @return   - none                                         *
 * @Note     - none                                         *
 * **********************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t temp = 0; // temp register
    // temp = (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    // //1. Configure the mode of GPIO pin
    // pGPIOHandle->pGPIOx->MODER &= ~(0x3 << temp); //clearing
    // pGPIOHandle->pGPIOx->MODER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << temp); //setting

    // Enable the clock for the GPIO port
    GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

    //1. Configure the mode of GPIO pin
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // the reason we multiply by 2 is because each pin takes two bit fields.
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clearing. the reason we multiply by 2 is because each pin takes two bit fields. ie: 0 * 2 = 0, 1 * 2 = 2, 2 * 2 = 4, etc..
        pGPIOHandle->pGPIOx->MODER |= temp; // setting
    }
    else
    {
        // (interrupt mode) 
        if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FE) // falling edge interrupt
        {
            // 1. configure the Falling Edge Selection Register (FESR)
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            // clear the corresponding RTSR bit (Rising Edge Selection Register)
            EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RE) // rising edge interrupt
        {
            // 1. configure the Rising Edge Selection Register (RESR)
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            // clear the corresponding FTSR bit (Falling Edge Selection Register)
            EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFE) // rising and falling edge interrupt
        {
            // 1. configure the Rising Edge Selection Register (RESR)
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            // 2. configure the Falling Edge Selection Register (FESR)
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else
        {
            // Invalid GPIO pin mode
        }

        // 2. configure the GPIO port selection in SYSCFG_EXTICR
        uint8_t temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4); // EXTI[0], EXTI[1], EXTI[2], etc... are in SYSCFG_EXTICR1, SYSCFG_EXTICR2, SYSCFG_EXTICR3, etc...
        uint8_t temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4); // starting point for the value to be written in SYSCFG_EXTICR bits.
        uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
        SYSCFG_PCLK_EN(); // have to enable the SYSCFG clock before writing to it.
        SYSCFG->EXTICR[temp1] = (portcode << (4 * temp2)); // 4 * temp2 is the starting point for the value to be written in SYSCFG_EXTICR bits.

        // 3. enable the exti interrupt delivery using IMR
        EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    }

    temp = 0; // reset temp register

    //2. Configure the speed of GPIO pin
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // save pin speed bitfields in temp register
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing previous values
    pGPIOHandle->pGPIOx->OSPEEDR |= temp; // setting

    temp = 0; // reset temp register

    //3. Configure the pupd settings of GPIO pin
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // save pin pupd bitfields in temp register
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing previous values
    pGPIOHandle->pGPIOx->PUPDR |= temp; // setting

    temp = 0; // reset temp register

    //4. Configure the output type of GPIO pin
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // save pin output type bitfields in temp register
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing previous values
    pGPIOHandle->pGPIOx->OTYPER |= temp; // setting

    temp = 0; // reset temp register

    //5. Configure the alternate functionality of GPIO pin, if required
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
    {
        //configure the alt function registers
        uint8_t temp1, temp2;
        temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8; // get the register number | Explained in udemy Section 25, Lecture 97. | pin number / 8 = 0 or 1. | pin number % 8 = bit number start index.
        temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8; // get the bit position | https://www.calculatorsoup.com/calculators/math/modulo-calculator.php | pin number / 8 = val R val | WholeNumVal * 8 + RemainderVal = modulo
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); // clearing previous values | Explained in udemy Section 25, Lecture 97. | 4 because there is 4 bits in each field. 0 * 4 = 0, 1 * 4 = 4, 2 * 4 = 8
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2)); // setting
    }

}

/****************************************************************
 * @fn       - GPIO_DeInit                                      *
 * @brief    - This function de-initializes the given GPIO port *
 * @param1   - Base address of the GPIO peripheral              *
 * @return   - none                                             *
 * @Note     - none                                             *
 * **************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
    if (pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET();
    }
    else if (pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET();
    }
    else if (pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET();
    }
    else if (pGPIOx == GPIOD)
    {
        GPIOD_REG_RESET();
    }
    else if (pGPIOx == GPIOE)
    {
        GPIOE_REG_RESET();
    }
    else if (pGPIOx == GPIOF)
    {
        GPIOF_REG_RESET();
    }
    else if (pGPIOx == GPIOG)
    {
        GPIOG_REG_RESET();
    }
    else if (pGPIOx == GPIOH)
    {
        GPIOH_REG_RESET();
    }
    else if (pGPIOx == GPIOI)
    {
        GPIOI_REG_RESET();
    }
    else
    {
        // Invalid GPIO port
    }

}

/***************************************************************************
 * @fn       - GPIO_ReadFromInputPin                                       *
 * @brief    - This function reads the input value from the given GPIO pin *
 * @param1   - Base address of the GPIO peripheral                         *
 * @param2   - Pin number                                                  *
 * @return   - none                                                        *
 * @Note     - This function reads a specific bit field within a gpio port *
 * Reference Manual Section: GPIO port input data register (GPIOx_IDR)     *
 * *************************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t value;
    value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001); // right shift by pin number and mask with 0x00000001 | that way you only return the value shifted.
    return value;
}

/****************************************************************************
 * @fn       - GPIO_ReadFromInputPort                                       *
 * @brief    - This function reads the input value from the given GPIO port *
 * @param1   - Base address of the GPIO peripheral                          *
 * @return   - none                                                         *
 * @Note     - This function reads the entire gpio port bits                *
 * **************************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    uint16_t value;
    value = (uint16_t)pGPIOx->IDR; // read the entire port bits | no need to shift or mask because you are reading the entire port.
    return value;
}

/**************************************************************************
 * @fn       - GPIO_WriteToOutputPin                                      *
 * @brief    - This function writes the given value to the given GPIO pin *
 * @param1   - Base address of the GPIO peripheral                        *
 * @param2   - Pin number                                                 *
 * @param3   - Value to be written                                        *
 * @return   - none                                                       *
 * @Note     - none                                                       *
 * ************************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    if (Value == GPIO_PIN_SET)
    {
        // write 1 to the output data register at the bit field corresponding to the pin number
        pGPIOx->ODR |= (1 << PinNumber);
    }
    else
    {
        // write 0
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}

/***************************************************************************
 * @fn       - GPIO_WriteToOutputPort                                      *
 * @brief    - This function writes the given value to the given GPIO port *
 * @param1   - Base address of the GPIO peripheral                         *
 * @param2   - Value to be written                                         *
 * @return   - none                                                        *
 * @Note     - none                                                        *
 * *************************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR = Value; // because we are writing the value to the whole port instead of a pin.
}

/********************************************************
 * @fn       - GPIO_ToggleOutputPin                     *
 * @brief    - This function toggles the given GPIO pin *
 * @param1   - Base address of the GPIO peripheral      *
 * @param2   - Pin number                               *
 * @return   - none                                     *
 * @Note     - none                                     *
 * ******************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (1 << PinNumber); // XOR with 1 to toggle the pin
}

/*************************************************************
 * @fn       - GPIO_IRQConfig                                *
 * @brief    - This function configures the given IRQ number *
 * @param1   - IRQ number                                    *
 * @param2   - Enable or Disable macro                       *
 * @return   - none                                          *
 * @Note     - none                                          *
 * ***********************************************************/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (IRQNumber <= 31)
        {
            // program ISER0 register
            *NVIC_ISER0 |= (1 << IRQNumber); // set the bit corresponding to the IRQ number
        }
        else if (IRQNumber > 31 && IRQNumber < 64)
        {
            // program ISER1 register
            *NVIC_ISER1 |= (1 << (IRQNumber % 32)); // % 32 because we are only interested in the 32 bit to 63 bit
        }
        else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            // program ISER2 register
            *NVIC_ISER2 |= (1 << (IRQNumber % 64)); // % 64 because you are only interested in the 64th bit to 95th bit
        }
    }
    else if (EnorDi == DISABLE)
    {
        if (IRQNumber <= 31)
        {
            // program ICER0 register
            *NVIC_ICER0 |= (1 << IRQNumber); // set the bit corresponding to the IRQ number
        }
        else if (IRQNumber > 31 && IRQNumber < 64)
        {
            // program ICER1 register
            *NVIC_ICER1 |= (1 << (IRQNumber % 32)); // % 32 because we are only interested in the 32 bit to 63 bit
        }
        else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            // program ICER2 register
            *NVIC_ICER2 |= (1 << (IRQNumber % 64)); // % 64 because you are only interested in the 64th bit to 95th bit
        }
    } 
    else {
        // Invalid IRQ number | do nothing maybe printf an error message
    }
}


/***************************************************************
 * @fn       - GPIO_IRQPriorityConfig                          *
 * @brief    - This function configures the given IRQ priority *
 * @param1   - IRQ number                                      *
 * @param2   - IRQ priority                                    *
 * @return   - none                                            *
 * @Note     - none                                            *
 * *************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    // 1. Find out the IPR register
    uint8_t ipr_x = IRQNumber / 4; // 4 because there are 4 IRQs sections  (0-3) per register 
    uint8_t ipr_x_section = IRQNumber % 4; // % 4 because there are 4 IRQs sections  (0-3) per register
    uint8_t shift_amount = (8 * ipr_x_section) + (8 - NUM_PR_BITS_IMPLEMENTED); // 8 because there are 8 bits per register
    *(NVIC_PR_BASE_ADDR + ipr_x) |= (IRQPriority << shift_amount); // shift the priority to the correct position
}

/***************************************************
 * @fn       - GPIO_IRQHandler                     *
 * @brief    - This function handles the given IRQ *
 * @param1   - Pin number                          *
 * @return   - none                                *
 * @Note     - none                                *
 * *************************************************/
void GPIO_IRQHandler(uint8_t PinNumber)
{
    // Clear the EXTI PR register corresponding to the pin number
    if (EXTI->PR & (1 << PinNumber))
    {
        EXTI->PR |= (1 << PinNumber);
    }
}
