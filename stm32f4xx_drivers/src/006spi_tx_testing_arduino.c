#include "stm32f407xx.h"
#include "stm32f407xx_gpio.h"
#include "stm32f407xx_spi.h"
#include <string.h>

void delay(void)
{
    for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{
  // exercise test the SPI_SendData API to send the string Hello World and use the below configurations
  // 1. SPI-2 Master Mode
  // 2. SCLK = Max Possible
  // 3. DFF = 8 bits (default) and 16 bits


  // configure the GPIO pins for SPI2
  // 1. enable the clock for GPIOB peripheral
  // GPIO_PeriClockControl(GPIOB, ENABLE);

  // 2. configure the GPIO pins for SPI2
  GPIO_Handle_t gpioB, gpioA;
  memset(&gpioB, 0, sizeof(gpioB));

  gpioB.pGPIOx = GPIOB;
  gpioB.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
  gpioB.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
  gpioB.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
  gpioB.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
  gpioB.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  
  // NSS
  gpioB.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
  GPIO_Init(&gpioB);

  // SCLK
  gpioB.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
  GPIO_Init(&gpioB);

  // // MISO
  // gpioB.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
  // GPIO_Init(&gpioB);

  // MOSI
  gpioB.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
  GPIO_Init(&gpioB);

  // maybe add button too
  memset(&gpioA, 0, sizeof(gpioA));
  gpioA.pGPIOx = GPIOA;
  gpioA.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
  gpioA.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  gpioA.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  gpioA.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
  GPIO_Init(&gpioA);

  stm32f407xx_spi_handle_t spi2Handle;
  char user_data[] = "Hello World";

  // initialize the SPI2 peripheral handle
  memset(&spi2Handle, 0, sizeof(spi2Handle));

  // // 1. enable the clock for SPI2 peripheral
  // SPI_PeriClockControl(SPI2, ENABLE);

  // 2. configure the SPI2 peripheral
  spi2Handle.pSPIx = SPI2;
  spi2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
  spi2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
  spi2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
  spi2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BIT;
  spi2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
  spi2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
  spi2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // hardware slave management enabled for NSS pin
  // spi2Handle.SPIConfig.SPI_SSOE = SPI_SSOE_EN;
  // spi2Handle.SPIConfig.SPI_CRCL = SPI_CRCL_8BIT;
  // spi2Handle.SPIConfig.SPI_FirstBit = SPI_FIRSTBIT_MSB;
  // spi2Handle.SPIConfig.SPI_TxOnly = SPI_TXONLY_EN;
  // spi2Handle.SPIConfig.SPI_RxOnly = SPI_RXONLY_DIS;

  // 3. initialize the SPI2 peripheral
  SPI_Init(&spi2Handle);

  // 4. enable the SPI2 peripheral
  SPI_SSOEConfig(SPI2, ENABLE);

  // // this makes NSS signal internally high and avoids MODF error
  // SPI_SSIConfig(SPI2, ENABLE); // Only needed when using SSM

  while(1)
  {
    // wait for button press
    while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

    // delay to avoid button de-bouncing 200ms of delay
    delay();

    // 4. enable the SPI2 peripheral
    SPI_PeripheralControl(SPI2, ENABLE);
    
    // 5.1 send length information
    uint8_t data_len = strlen(user_data);
    SPI_SendData(SPI2, &data_len, 1);

    // 5.2 send data
    SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

    // 6. confirm SPI is not busy
    while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) == 1); // if flag is 1 its busy, if flag is 0 its not busy and exit the loop

    // 7. disable the SPI2 peripheral only when the last bit is sent. Which is why the flag status is used.
    SPI_PeripheralControl(SPI2, DISABLE);
  }

  return 0;
}