#ifndef INC_STM32F407XX_SPI_H_
#define INC_STM32F407XX_SPI_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for SPIx peripheral
 */
typedef struct 
{
  uint8_t SPI_DeviceMode;		// Possible values from @SPI_DeviceMode
  uint8_t SPI_BusConfig;		// Possible values from @SPI_BusConfig
  uint8_t SPI_SclkSpeed;		// Possible values from @SPI_SclkSpeed
  uint8_t SPI_DFF;			// Possible values from @SPI_DFF
  uint8_t SPI_CPOL;			// Possible values from @SPI_CPOL
  uint8_t SPI_CPHA;			// Possible values from @SPI_CPHA
  uint8_t SPI_SSM;			// Possible values from @SPI_SSM
} stm32f407xx_spi_config_t;

/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
  SPI_RegDef_t *pSPIx;      // this holds the base address of the SPIx(x:0,1,2) peripheral
  stm32f407xx_spi_config_t SPIConfig;
} stm32f407xx_spi_handle_t;


/***********************************************************
 * APIs supported by this driver
 * For more information about the APIs check the function definitions
 * *********************************************************/
// Peripheral Clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

// Init and De-init
void SPI_Init(stm32f407xx_spi_handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

// Enable SPI peripheral
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

// Data send and receive
void SPI_SendData(SPI_RegDef_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

// IRQ Configuration and ISR handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi); // IRQNumber is MCU specific
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority); // IRQPriority is a number between 0 to 255
void SPI_IRQHandling(stm32f407xx_spi_handle_t *pHandle); // This function is implemented in the application

/*
 * @SPI_Handles
 */
#define SPI1    ((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2    ((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3    ((SPI_RegDef_t *)SPI3_BASEADDR)
#define SPI4    ((SPI_RegDef_t *)SPI4_BASEADDR)

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_SLAVE		      0
#define SPI_DEVICE_MODE_MASTER		    1

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD			         1 // Full duplex
#define SPI_BUS_CONFIG_HD			         2 // Half duplex
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	 3 // Simplex RX only

// #define SPI_BUS_CONFIG_BIDIRECTIONAL   0b10 // Bidirectional
// #define SPI_BUS_CONFIG_UNIDIRECTIONAL  0b01 // Unidirectional
// #define SPI_BUS_CONFIG_SIMPLEX_ONLY    0b11 // Unidirectional

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2		  0
#define SPI_SCLK_SPEED_DIV4		  1
#define SPI_SCLK_SPEED_DIV8		  2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BIT		  0 // 8 bits
#define SPI_DFF_16BIT		1 // 16 bits

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW		0 // Clock to low when idle
#define SPI_CPOL_HIGH		1 // Clock is high when idle

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_LOW		0 // The first clock transition is the first data capture edge
#define SPI_CPHA_HIGH		1 // The second clock transition is the first data capture edge

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI		0 // Software slave management disabled
#define SPI_SSM_EN		1 // Software slave management enabled

/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG		(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG		(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG		(1 << SPI_SR_BSY)
#define SPI_OVR_FLAG		(1 << SPI_SR_OVR)
#define SPI_FRE_FLAG		(1 << SPI_SR_FRE)
#define SPI_MODF_FLAG		(1 << SPI_SR_MODF)
#define SPI_CRCERR_FLAG	(1 << SPI_SR_CRCERR)
#define SPI_UDR_FLAG		(1 << SPI_SR_UDR)
#define SPI_CHSIDE_FLAG	(1 << SPI_SR_CHSIDE)

/*
 * SPI application states
 */

#define SPI_DISABLE_SR		0
#define SPI_ENABLE_SR		1

#define SPI_DISABLE_CR1		0
#define SPI_ENABLE_CR1		1

#define SPI_DISABLE_CR2		0
#define SPI_ENABLE_CR2		1

#define SPI_DISABLE_CRCPR	0
#define SPI_ENABLE_CRCPR	1

#define SPI_DISABLE_RXCRCR	0
#define SPI_ENABLE_RXCRCR	1

#define SPI_DISABLE_TXCRCR	0
#define SPI_ENABLE_TXCRCR	1

#define SPI_DISABLE_I2SCFGR	0
#define SPI_ENABLE_I2SCFGR	1

#define SPI_DISABLE_I2SPR	0
#define SPI_ENABLE_I2SPR	1




#endif /* INC_STM32F407XX_SPI_H_ */