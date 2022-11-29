#include "stm32f407xx.h"
#include "stm32f407xx_spi.h"

/*********************************************************************
 * @fn       - SPI_Peripheral Clock Control                          *
 * @brief    - This function configures the SPI clock                *
 * @param1   - ptr peripheral register definition structure for SPIx *
 * @param2   - Enable or Disable                                     *
 * @return   - none                                                  *
 * @Note     - none                                                  *
 * *******************************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
  // Enable or Disable peripheral clock for the given SPI
  if(EnOrDi == ENABLE)
  {
    if(pSPIx == SPI1)
    {
      SPI1_PCLK_EN();
    }
    else if(pSPIx == SPI2)
    {
      SPI2_PCLK_EN();
    }
    else if(pSPIx == SPI3)
    {
      SPI3_PCLK_EN();
    }
    else if(pSPIx == SPI4)
    {
      SPI4_PCLK_EN();
    }
  }
  else /* else disable */
  {
    if(pSPIx == SPI1)
    {
      SPI1_PCLK_DI();
    }
    else if(pSPIx == SPI2)
    {
      SPI2_PCLK_DI();
    }
    else if(pSPIx == SPI3)
    {
      SPI3_PCLK_DI();
    }
    else if(pSPIx == SPI4)
    {
      SPI4_PCLK_DI();
    }
  }
}

/***************************************************************
 * @fn       - SPI_init                      
 * @brief    - This function initializes the SPI peripheral 
 * @param1   - ptr to SPI handle                                   
 * @param2   - none                                  
 * @return   - none                                          
 * @Note     - this function configures the SPI_Config_t struct.
 *           - SPI_DeviceMode
 *           - SPI_BusConfig
 *           - SPI_SclkSpeed
 *           - SPI_DFF
 *           - SPI_CPOL
 *           - SPI_CPHA
 *           - SPI_SSM                                          
 * *************************************************************/
void SPI_Init(stm32f407xx_spi_handle_t *pSPIHandle)
{
  // Enable the clock for SPI2 peripheral
  SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

  // Configure the CR1 register
  uint32_t tempreg = 0;
  // 1. Configure the device mode
  tempreg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);
  // 2. Configure the bus config
  if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
  {
    // BIDI mode should be cleared
    tempreg &= ~(1 << SPI_CR1_BIDIMODE); // BIDI mode = Bi-Directional mode
  }
  else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
  {
    // BIDI mode should be set
    tempreg |= (1 << SPI_CR1_BIDIMODE);
  }
  else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
  {
    // BIDI mode should be cleared
    tempreg &= ~(1 << SPI_CR1_BIDIMODE);
    // RXONLY bit must be set
    tempreg |= (1 << SPI_CR1_RXONLY);
  }

  // 3. Configure the SCLK speed (baud rate)
  tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

  // 4. Configure the DFF
  tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

  // 5. Configure the CPOL
  tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

  // 6. Configure the CPHA
  tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

  // 7. Configure the SSM
  tempreg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

  pSPIHandle->pSPIx->CR1 = tempreg;  
}

/***************************************************************
 * @fn       - SPI_DeInit                      
 * @brief    - This function de-initializes the SPI peripheral 
 * @param1   - ptr to SPI struct                                   
 * @param2   - none                                  
 * @return   - none                                          
 * @Note     - none                                          
 * *************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
  // Reset the SPIx peripheral
  if(pSPIx == SPI1)
  {
    SPI1_REG_RESET();
  }
  else if(pSPIx == SPI2)
  {
    SPI2_REG_RESET();
  }
  else if(pSPIx == SPI3)
  {
    SPI3_REG_RESET();
  }
  else if(pSPIx == SPI4)
  {
    SPI4_REG_RESET();
  }
}

/***************************************************************
 * @fn       - SPI_PeripheralControl                       
 * @brief    - This function enables or disables the SPI peripheral
 * @param1   - ptr to SPI struct
 * @param2   - Enable or Disable
 * @return   - none
 * @Note     - none
 ***************************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
  // SPE = SPI Peripheral Enable
  if(EnOrDi == ENABLE)
  {
    pSPIx->CR1 |= (1 << SPI_CR1_SPE); // Enable the SPI peripheral
  }
  else
  {
    pSPIx->CR1 &= ~(1 << SPI_CR1_SPE); // Disable the SPI peripheral
  }
}

/***************************************************************
 * @fn       - SPI_SSIConfig                       
 * @brief    - This function configures the SSI bit in the CR1 register
 * @param1   - ptr to SPI struct
 * @param2   - Enable or Disable
 * @return   - none
 * @Note     - none
 ***************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
  // SSI = Software Slave Management
  if(EnOrDi == ENABLE)
  {
    pSPIx->CR1 |= (1 << SPI_CR1_SSI); // Enable the SSI
  }
  else
  {
    pSPIx->CR1 &= ~(1 << SPI_CR1_SSI); // Disable the SSI
  }
}

/***************************************************************
 * @fn       - SPI_SSOEConfig                       
 * @brief    - This function configures the SSOE bit in the CR2 register
 * @param1   - ptr to SPI struct
 * @param2   - Enable or Disable
 * @return   - none
 * @Note     - none
 ***************************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
  // SSI = Software Slave Management
  if(EnOrDi == ENABLE)
  {
    pSPIx->CR2 |= (1 << SPI_CR2_SSOE); // Enable the SSOE
  }
  else
  {
    pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE); // Disable the SSOE
  }
}

/***************************************************************
 * @fn       - SPI_GetFlagStatus                       
 * @brief    - This function returns the status of the flag 
 * @param1   - ptr to SPI struct                                   
 * @param2   - flag name                                   
 * @return   - none                                            
 * @Note     - none                                            
 * *************************************************************/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
  // Check the status of the flag
  if(pSPIx->SR & FlagName) // if the flag is 1
  {
    return FLAG_SET;
  }

  return FLAG_RESET;       // else if the flag is 0
}

/***************************************************************
 * @fn       - SPI_SendData                       
 * @brief    - This function sends data  
 * @param1   - ptr peripheral register definition structure for SPIx                                 
 * @param2   - ptr to buffer                                 
 * @param3   - length of data                                        
 * @return   - none                                          
 * @Note     - This is a blocking call | Polling based function                                    
 * *************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len)
{
  // Loop until len becomes 0
  while(len > 0)
  {
    // 1. Wait until TXE is empty (FLAG_SET)
    while(SPI_GetFlagStatus(pSPIHandle, SPI_TXE_FLAG) == FLAG_RESET); // TXE = Transmit buffer empty flag 0 = not empty 1 = empty
    // 2. Check the DFF bit in CR1
    if((pSPIHandle->CR1 & (1 << SPI_CR1_DFF)) == 0)
    {
      // 8 bit DFF
      // 3. Load the data into the DR
      pSPIHandle->DR = *pTxBuffer; // DR = Data register for SPI, load data into DR
      len--; // decrement the length
      pTxBuffer++; // increment the buffer address
    }
    else
    {
      // 16 bit DFF
      // 3. Load the data into the DR
      pSPIHandle->DR = *((uint16_t*)pTxBuffer); // static cast to 16 bit and load the data into DR
      len--; // Decrement the length by 2
      len--;
      (uint16_t*)pTxBuffer++; // Increment the buffer address
    }
  }
}

/***************************************************************
 * @fn       - SPI_ReceiveData                       
 * @brief    - This function receives data  
 * @param1   - ptr peripheral register definition structure for SPIx                              
 * @param2   - ptr to buffer                                 
 * @param3   - length of data                                             
 * @return   - none                                        
 * @Note     - This is a blocking call | Polling based function                                     
 * *************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
  // Loop until len becomes 0
  while(len > 0)
  {
    // 1. Wait until RXNE is empty (FLAG_RESET)
    while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == (uint8_t)FLAG_RESET); // RXNE = Receive buffer not empty flag 0 = empty 1 = not empty
    // 2. Check the DFF bit in CR1
    if((pSPIx->CR1 & (1 << SPI_CR1_DFF)) == 0)
    {
      // 8 bit DFF
      // 3. Load the data from the DR
      *pRxBuffer = pSPIx->DR; // DR = Data register for SPI, load data from DR
      len--; // decrement the length
      pRxBuffer++; // increment the buffer address
      // Steps: 1. Wait until Rx Buffer is not empty
      //        2. Check the DFF bit in CR1 to decide 8 bit or 16 bit
      //        3. If 8 bit, load the data from DR to Rx buffer
      //        3.1 Read DR for 1 byte of data, decrease the length by 1 and increase the buffer address by 1
      //        4. If 16 bit, load the data from DR to Rx buffer
      //        4.1 Read DR for 2 bytes of data, decrease the length by 2 and increase the buffer address by 2
      //        repeat until length becomes 0
    }
    else
    {
      // 16 bit DFF
      // 3. Load the data from the DR
      *((uint16_t*)pRxBuffer) = pSPIx->DR; // static cast to 16 bit and load the data from DR
      len--; // Decrement the length by 2
      len--;
      (uint16_t*)pRxBuffer++; // Increment the buffer address
    }
  }
}

/***************************************************************
 * @fn       - SPI_IRQInterruptConfig                       
 * @brief    - This function configures the IRQ interrupt  
 * @param1   - IRQ number                                    
 * @param2   - Enable or Disable                                       
 * @return   - none                                        
 * @Note     - none                                          
 * *************************************************************/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
  // Configure the FIRQENx register
}

/***************************************************************
 * @fn       - SPI_IRQPriorityConfig                      
 * @brief    - This function configures the IRQ priority  
 * @param1   - IRQ number                                    
 * @param2   - IRQ priority                                    
 * @return   - none                                        
 * @Note     - none                                          
 * *************************************************************/
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
  // Find out the IPR register
  // Find out the iprx
  // Set the priority
}

/***************************************************************
 * @fn       - SPI_IRQHandling                     
 * @brief    - This function handles the IRQ  
 * @param1   - ptr to SPI handle                                   
 * @return   - none                                          
 * @Note     - none                                          
 * *************************************************************/
void SPI_IRQHandling(stm32f407xx_spi_handle_t *pHandle)
{
  // Check for TXE
  // Check for RXNE
  // Check for OVR flag
}