#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

// Includes
#include <stdint.h>

#define __vo volatile

/******************** Processor Specific Details (Cortex M4 Processor)**********/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */
#define NVIC_ISER0      ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1      ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2      ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3      ((__vo uint32_t*)0xE000E10C)
#define NVIC_ISER4      ((__vo uint32_t*)0xE000E110)
#define NVIC_ISER5      ((__vo uint32_t*)0xE000E114)
#define NVIC_ISER6      ((__vo uint32_t*)0xE000E118)
#define NVIC_ISER7      ((__vo uint32_t*)0xE000E11C)

/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0      ((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1      ((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2      ((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3      ((__vo uint32_t*)0xE000E18C)
#define NVIC_ICER4      ((__vo uint32_t*)0xE000E190)
#define NVIC_ICER5      ((__vo uint32_t*)0xE000E194)
#define NVIC_ICER6      ((__vo uint32_t*)0xE000E198)
#define NVIC_ICER7      ((__vo uint32_t*)0xE000E19C)

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR   ((__vo uint32_t*)0xE000E400)

#define NUM_PR_BITS_IMPLEMENTED  4

/********************** End of Processor Specific Details (Cortex M4 Processor)**********/

// Base addresses of Flash and SRAM memories
#define FLASH_BASEADDR  0x08000000U      /* Base address of Flash memory */
#define SRAM1_BASEADDR  0x20000000U      /* 112 KB */
#define SRAM2_BASEADDR  0x2001C000U      /* SRAM2 is 112KB */
#define ROM_BASEADDR    0x1FFF0000U      /* System memory is ROM */
#define SRAM_BASEADDR   (SRAM1_BASEADDR) /* SRAM1 is 112KB and SRAM1 is also called main RAM also known as SRAM */

/*
 * Base addresses of AHBx and APBx buses (Bus matrix) peripherals
 * 
 */

#define PERIPH_BASEADDR     0x40000000U
#define APB1PERIPH_BASEADDR (PERIPH_BASEADDR) /* APB1 peripherals are connected to AHB1 bus */
#define APB2PERIPH_OFFSET   0x00010000U       /* APB2 peripherals are connected to AHB1 bus */

#define AHB1PERIPH_OFFSET   0x00020000U       /* AHB1 peripherals are connected to AHB1 bus */
#define AHB2PERIPH_OFFSET   0x10000000U       /* AHB2 peripherals are connected to AHB2 bus */

#define APB2PERIPH_BASEADDR (PERIPH_BASEADDR + APB2PERIPH_OFFSET) /* 64 KB - 0x4001 0000 */
#define AHB1PERIPH_BASEADDR (PERIPH_BASEADDR + AHB1PERIPH_OFFSET) /* 128 KB - 0x4002 0000 */
#define AHB2PERIPH_BASEADDR (PERIPH_BASEADDR + AHB2PERIPH_OFFSET) /* 128 KB - 0x5000 0000 */

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * 
 */

#define GPIOA_OFFSET   0x0000U
#define GPIOB_OFFSET   0x0400U
#define GPIOC_OFFSET   0x0800U
#define GPIOD_OFFSET   0x0C00U
#define GPIOE_OFFSET   0x1000U
#define GPIOF_OFFSET   0x1400U
#define GPIOG_OFFSET   0x1800U
#define GPIOH_OFFSET   0x1C00U
#define GPIOI_OFFSET   0x2000U
#define RCC_OFFSET     0x3800U

#define GPIOA_BASEADDR (AHB1PERIPH_BASEADDR + GPIOA_OFFSET)
#define GPIOB_BASEADDR (AHB1PERIPH_BASEADDR + GPIOB_OFFSET)
#define GPIOC_BASEADDR (AHB1PERIPH_BASEADDR + GPIOC_OFFSET)
#define GPIOD_BASEADDR (AHB1PERIPH_BASEADDR + GPIOD_OFFSET)
#define GPIOE_BASEADDR (AHB1PERIPH_BASEADDR + GPIOE_OFFSET)
#define GPIOF_BASEADDR (AHB1PERIPH_BASEADDR + GPIOF_OFFSET)
#define GPIOG_BASEADDR (AHB1PERIPH_BASEADDR + GPIOG_OFFSET)
#define GPIOH_BASEADDR (AHB1PERIPH_BASEADDR + GPIOH_OFFSET)
#define GPIOI_BASEADDR (AHB1PERIPH_BASEADDR + GPIOI_OFFSET)
#define RCC_BASEADDR   (AHB1PERIPH_BASEADDR + RCC_OFFSET)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * 
 */

/* I2C */
#define I2C1_OFFSET   0x5400U
#define I2C1_BASEADDR (APB1PERIPH_BASEADDR + I2C1_OFFSET)

#define I2C2_OFFSET   0x5800U
#define I2C2_BASEADDR (APB1PERIPH_BASEADDR + I2C2_OFFSET)

#define I2C3_OFFSET   0x5C00U
#define I2C3_BASEADDR (APB1PERIPH_BASEADDR + I2C3_OFFSET)

/* UART / USART */
#define USART2_OFFSET 0x4400U
#define USART2_BASEADDR (APB1PERIPH_BASEADDR + USART2_OFFSET)

#define USART3_OFFSET 0x4800U
#define USART3_BASEADDR (APB1PERIPH_BASEADDR + USART3_OFFSET)

#define UART4_OFFSET  0x4C00U
#define UART4_BASEADDR  (APB1PERIPH_BASEADDR + UART4_OFFSET)

#define UART5_OFFSET  0x5000U
#define UART5_BASEADDR  (APB1PERIPH_BASEADDR + UART5_OFFSET)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * 
 */

/* SPI */
#define SPI1_OFFSET   0x3000U
#define SPI1_BASEADDR (APB2PERIPH_BASEADDR + SPI1_OFFSET)

#define SPI2_OFFSET   0x3800U
#define SPI2_BASEADDR (APB1PERIPH_BASEADDR + SPI2_OFFSET)

#define SPI3_OFFSET   0x3C00U
#define SPI3_BASEADDR (APB1PERIPH_BASEADDR + SPI3_OFFSET)

#define SPI4_OFFSET   0x3400U
#define SPI4_BASEADDR (APB1PERIPH_BASEADDR + SPI4_OFFSET)

/* UART / USART */
#define USART1_OFFSET 0x1000U
#define USART1_BASEADDR (APB2PERIPH_BASEADDR + USART1_OFFSET)

#define USART6_OFFSET 0x1400U
#define USART6_BASEADDR (APB2PERIPH_BASEADDR + USART6_OFFSET)

/* SYSCFG */
#define SYSCFG_OFFSET 0x3800U // 0x4001 3800
#define SYSCFG_BASEADDR (APB2PERIPH_BASEADDR + SYSCFG_OFFSET)

/* EXTI */
#define EXTI_OFFSET   0x3C00U
#define EXTI_BASEADDR (APB2PERIPH_BASEADDR + EXTI_OFFSET) 


// Exercise SPI1 on the STM32F4xx Discovery Board
#define SPI_CR1     (SPI1_BASEADDR + 0x00U) // SPI control register 1
#define SPI_CR2     (SPI1_BASEADDR + 0x04U) // SPI control register 2
#define SPI_SR      (SPI1_BASEADDR + 0x08U) // SPI status register
#define SPI_DR      (SPI1_BASEADDR + 0x0CU) // SPI data register
#define SPI_CRCPR   (SPI1_BASEADDR + 0x10U) // SPI CRC polynomial register
#define SPI_RXCRCR  (SPI1_BASEADDR + 0x14U) // SPI RX CRC register
#define SPI_TXCRCR  (SPI1_BASEADDR + 0x18U) // SPI TX CRC register
#define SPI_I2SCFGR (SPI1_BASEADDR + 0x1CU) // SPI_I2S configuration register
#define SPI_I2SPR   (SPI1_BASEADDR + 0x20U) // SPI_I2S prescaler register

/*
 * peripheral register definition structure for GPIOx
 */

typedef struct {
  __vo uint32_t MODER;   // GPIO port mode register                Address Offset 0x00
  __vo uint32_t OTYPER;  // GPIO port output type register         Address Offset 0x04
  __vo uint32_t OSPEEDR; // GPIO port output speed register        Address Offset 0x08
  __vo uint32_t PUPDR;   // GPIO port pull-up/pull-down register   Address Offset 0x0C
  __vo uint32_t IDR;     // GPIO port input data register          Address Offset 0x10
  __vo uint32_t ODR;     // GPIO port output data register         Address Offset 0x14
  __vo uint32_t BSRR;    // GPIO port bit set/reset register       Address Offset: 0x0010
  __vo uint32_t LCKR;    // GPIO port configuration lock register  Address Offset: 0x1C
  __vo uint32_t AFR[2];  // GPIO alternate function low/high register AFR[0] -> AFRL, AFR[1] -> AFRH | Address Offset: 0x20-0x24
} GPIO_RegDef_t;

/*
 * peripheral register definition structure for RCC
 */
typedef struct {
  __vo uint32_t CR;           // RCC clock control register,                                  Address offset: 0x00
  __vo uint32_t PLLCFGR;      // RCC PLL configuration register,                              Address offset: 0x04
  __vo uint32_t CFGR;         // RCC clock configuration register,                            Address offset: 0x08
  __vo uint32_t CIR;          // RCC clock interrupt register,                                Address offset: 0x0C
  __vo uint32_t AHB1RSTR;     // RCC AHB1 peripheral reset register,                          Address offset: 0x10
  __vo uint32_t AHB2RSTR;     // RCC AHB2 peripheral reset register,                          Address offset: 0x14
  __vo uint32_t AHB3RSTR;     // RCC AHB3 peripheral reset register,                          Address offset: 0x18
  uint32_t      RESERVED0;    // Reserved, 0x1C
  __vo uint32_t APB1RSTR;     // RCC APB1 peripheral reset register,                          Address offset: 0x20
  __vo uint32_t APB2RSTR;     // RCC APB2 peripheral reset register,                          Address offset: 0x24
  uint32_t      RESERVED1[2]; // Reserved, 0x28-0x2C
  __vo uint32_t AHB1ENR;      // RCC AHB1 peripheral clock register,                          Address offset: 0x30
  __vo uint32_t AHB2ENR;      // RCC AHB2 peripheral clock register,                          Address offset: 0x34
  __vo uint32_t AHB3ENR;      // RCC AHB3 peripheral clock register,                          Address offset: 0x38
  uint32_t      RESERVED2;    // Reserved, 0x3C
  __vo uint32_t APB1ENR;      // RCC APB1 peripheral clock enable register,                   Address offset: 0x40
  __vo uint32_t APB2ENR;      // RCC APB2 peripheral clock enable register,                   Address offset: 0x44
  uint32_t      RESERVED3[2]; // Reserved, 0x48-0x4C
  __vo uint32_t AHB1LPENR;    // RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50
} RCC_RegDef_t;

/*
 * peripheral register definition structure for EXTI
 */
typedef struct {
  __vo uint32_t IMR;    // Interrupt mask register,                                           Address offset: 0x00
  __vo uint32_t EMR;    // Event mask register,                                               Address offset: 0x04
  __vo uint32_t RTSR;   // Rising trigger selection register,                                 Address offset: 0x08
  __vo uint32_t FTSR;   // Falling trigger selection register,                                Address offset: 0x0C
  __vo uint32_t SWIER;  // Software interrupt event register,                                 Address offset: 0x10
  __vo uint32_t PR;     // Pending register,                                                  Address offset: 0x14
} EXTI_RegDef_t;

/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct {
  __vo uint32_t MEMRMP;     // SYSCFG memory remap register,                                  Address offset: 0x00
  __vo uint32_t PMC;        // SYSCFG peripheral mode configuration register,                 Address offset: 0x04
  __vo uint32_t EXTICR[4];  // SYSCFG external interrupt configuration register 1, 2, 3, 4    Address offset: 0x08-0x14
  uint32_t      RESERVED1[2]; // Reserved, 0x18-0x1C
  __vo uint32_t CMPCR;      // Compensation cell control register,                            Address offset: 0x20
  uint32_t      RESERVED2[2]; // Reserved, 0x24-0x28
  __vo uint32_t CFGR;        // SYSCFG configuration register,                                 Address offset: 0x2C
} SYSCFG_RegDef_t;


/*
 * peripheral register definition structure for SPIx
 */

typedef struct {
    __vo uint32_t CR1;      // SPI control register 1
    __vo uint32_t CR2;      // SPI control register 2
    __vo uint32_t SR;       // SPI status register
    __vo uint32_t DR;       // SPI data register
    __vo uint32_t CRCPR;    // SPI CRC polynomial register
    __vo uint32_t RXCRCR;   // SPI RX CRC register
    __vo uint32_t TXCRCR;   // SPI TX CRC register
    __vo uint32_t I2SCFGR;  // SPI_I2S configuration register
    __vo uint32_t I2SPR;    // SPI_I2S prescaler register
} SPI_RegDef_t;


/*
 * peripheral definitions (peripheral base addresses typecasted to GPIO_RegDef_t)
 */ 
#define GPIOA     ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB     ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC     ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD     ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE     ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF     ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG     ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH     ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI     ((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC       ((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI      ((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG    ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)




/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN() (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN() (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN() (RCC->AHB1ENR |= (1 << 8))

/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 8))

/*
 * GPIO Reset Macros
 */
#define GPIOA_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while (0) // reset GPIOA peripheral register by setting and then clearing the bit | note: condition 0 only runs once
#define GPIOB_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while (0) 
#define GPIOC_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while (0)
#define GPIOD_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while (0)
#define GPIOE_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while (0)
#define GPIOF_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); } while (0)
#define GPIOG_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); } while (0)
#define GPIOH_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while (0)
#define GPIOI_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); } while (0)


/*
 * Returns port code for given GPIOx base address
 */
#define GPIO_BASEADDR_TO_CODE(x)   ( (x == GPIOA) ? 0 :\
                                     (x == GPIOB) ? 1 :\
                                     (x == GPIOC) ? 2 :\
                                     (x == GPIOD) ? 3 :\
                                     (x == GPIOE) ? 4 :\
                                     (x == GPIOF) ? 5 :\
                                     (x == GPIOG) ? 6 :\
                                     (x == GPIOH) ? 7 :\
                                     (x == GPIOI) ? 8 :0 )

/*
 * SPI clock enable macros
 */
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12)) // page 188 of the reference manual
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13)) // page 188 of the reference manual
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14)) // page 185 of the reference manual
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15)) // page 185 of the reference manual

/*
 * SPI clock disable macros
 */
#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12)) // page 188 of the reference manual'
#define SPI4_PCLK_DI() (RCC->APB2ENR &= ~(1 << 13)) // page 188 of the reference manual
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 14)) // page 185 of the reference manual
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 15)) // page 185 of the reference manual

/*
 * SPI reset macros
 */
#define SPI1_REG_RESET() do { (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); } while (0) // page 188 of the reference manual
#define SPI2_REG_RESET() do { (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); } while (0) // page 185 of the reference manual
#define SPI3_REG_RESET() do { (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); } while (0) // page 185 of the reference manual
#define SPI4_REG_RESET() do { (RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13)); } while (0) // page 188 of the reference manual


// /*
//  * Clock Enable Macros for I2Cx peripherals
//  */
// #define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
// #define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
// #define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))

// /*
//  * Clock Disable Macros for I2Cx peripherals
//  */
// #define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 21))
// #define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 22))
// #define I2C3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 23))




/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 4))

/*
 * Clock Enable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))

/*
 * Interrupt Request Number (IRQ Number) Macros (Check Vector Table of STM32F407 in Reference Manual)
 */
#define IRQ_NO_EXTI0    6    // EXTI0 
#define IRQ_NO_EXTI1    7    // EXTI1 
#define IRQ_NO_EXTI2    8    // EXTI2 
#define IRQ_NO_EXTI3    9    // EXTI3
#define IRQ_NO_EXTI4    10   // EXTI4_15
#define IRQ_NO_EXTI9_5  23   // EXTI 5 to 9 Offset: 0x0000 009C
#define IRQ_NO_EXTI15_10 40  // EXTI15_10 is connected to IRQ40

/*
 * IRQ Priority Macros
 */
#define NVIC_IRQ_PRI0   0
#define NVIC_IRQ_PRI15  15

/*
 * IRQ Number Masks
 */
#define EXTI_IMR_MR0    0   // Interrupt Mask Register
#define EXTI_IMR_MR1    1
#define EXTI_IMR_MR2    2
#define EXTI_IMR_MR3    3
#define EXTI_IMR_MR4    4
#define EXTI_IMR_MR5    5
#define EXTI_IMR_MR6    6
#define EXTI_IMR_MR7    7
#define EXTI_IMR_MR8    8
#define EXTI_IMR_MR9    9
#define EXTI_IMR_MR10   10
#define EXTI_IMR_MR11   11
#define EXTI_IMR_MR12   12
#define EXTI_IMR_MR13   13
#define EXTI_IMR_MR14   14
#define EXTI_IMR_MR15   15
#define EXTI_IMR_MR16   16
#define EXTI_IMR_MR17   17
#define EXTI_IMR_MR18   18
#define EXTI_IMR_MR19   19
#define EXTI_IMR_MR20   20
#define EXTI_IMR_MR21   21
#define EXTI_IMR_MR22   22
#define EXTI_IMR_MR23   23
#define EXTI_IMR_MR24   24
#define EXTI_IMR_MR25   25
#define EXTI_IMR_MR26   26
#define EXTI_IMR_MR27   27
#define EXTI_IMR_MR28   28
#define EXTI_IMR_MR29   29
#define EXTI_IMR_MR30   30
#define EXTI_IMR_MR31   31
// EXTI Event Mask Register
#define EXTI_RTSR_TR0   0   // Rising Trigger Selection Register
#define EXTI_FTSR_TR0   0   // Falling Trigger Selection Register
#define EXTI_PR_PR0     0   // Pending Register

// EXTI0 Interrupt Configuration Macros
#define EXTI_ClearPendingBit() (EXTI->PR |= (1 << 0)) // Clear Pending Register

#define EXTI0_IRQHandling()    EXTI_ClearPendingBit() // Clear Pending Register
// EXTI1 Interrupt Configuration Macros
#define EXTI1_IRQHandling(void)    EXTI_ClearPendingBit() // Clear Pending Register
// EXTI2 Interrupt Configuration Macros
#define EXTI2_IRQHandling(void)    EXTI_ClearPendingBit() // Clear Pending Register
// EXTI3 Interrupt Configuration Macros
#define EXTI3_IRQHandling(void)    EXTI_ClearPendingBit() // Clear Pending Register
// EXTI4 Interrupt Configuration Macros
#define EXTI4_IRQHandling(void)    EXTI_ClearPendingBit() // Clear Pending Register
// EXTI9_5 Interrupt Configuration Macros
#define EXTI9_5_IRQHandling(void)  EXTI_ClearPendingBit() // Clear Pending Register
// EXTI15_10 Interrupt Configuration Macros
#define EXTI15_10_IRQHandling(void) EXTI_ClearPendingBit() // Clear Pending Register
// IRQConfiguration and ISR Handling
// #define EXTI_IRQConfig(IRQNumber, EnorDi) do { if(EnorDi) { NVIC->ISER[IRQNumber / 32] |= (1 << (IRQNumber % 32)); } else { NVIC->ICER[IRQNumber / 32] |= (1 << (IRQNumber % 32)); } } while (0)

/*
 * Miscellaneous Macros
 */
#define ENABLE          1
#define DISABLE         0
#define SET             ENABLE
#define RESET           DISABLE
#define FLAG_RESET          RESET
#define FLAG_SET            SET

/*
 * Bit position definition of SPI peripheral
 */

// Control register 1 (SPI_CR1) bit macros
#define SPI_CR1_BIDIMODE    15
#define SPI_CR1_BIDIOE      14
#define SPI_CR1_CRCEN       13
#define SPI_CR1_CRCNEXT     12
#define SPI_CR1_DFF         11
#define SPI_CR1_RXONLY      10
#define SPI_CR1_SSM         9
#define SPI_CR1_SSI         8
#define SPI_CR1_LSBFIRST    7
#define SPI_CR1_SPE         6
#define SPI_CR1_BR          3
#define SPI_CR1_MSTR        2
#define SPI_CR1_CPOL        1
#define SPI_CR1_CPHA        0

// Control register 2 (SPI_CR2) bit macros
#define SPI_CR2_TXEIE       7
#define SPI_CR2_RXNEIE      6
#define SPI_CR2_ERRIE       5
#define SPI_CR2_FRF         4
#define SPI_CR2_SSOE        2
#define SPI_CR2_TXDMAEN     1
#define SPI_CR2_RXDMAEN     0

// Status register (SPI_SR) bit macros
#define SPI_SR_FRE          8
#define SPI_SR_BSY          7
#define SPI_SR_OVR          6
#define SPI_SR_MODF         5
#define SPI_SR_CRCERR       4
#define SPI_SR_UDR          3
#define SPI_SR_CHSIDE       2
#define SPI_SR_TXE          1
#define SPI_SR_RXNE         0

// SPI related status flags definitions
#define SPI_READY          1
#define SPI_BUSY_IN_RX     2
#define SPI_BUSY_IN_TX     3

// SPI Application Events
#define SPI_EVENT_TX_CMPLT  1
#define SPI_EVENT_RX_CMPLT  2
#define SPI_EVENT_OVR_ERR   3
#define SPI_EVENT_CRC_ERR   4


#endif /* INC_STM32F407XX_H_ */