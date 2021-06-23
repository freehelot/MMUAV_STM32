/**
  ******************************************************************************
  * @file    bsp_spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
  ******************************************************************************
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_spi.h"

// Private defines
#define SPI_INSTANCE    SPI1

// Data Types
SPI_HandleTypeDef hspi1;

// Public functions

void bsp_spi_init(void)
{
	  hspi1.Instance = SPI_INSTANCE;
	  hspi1.Init.Mode = SPI_MODE_MASTER;
	  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	  hspi1.Init.NSS = SPI_NSS_SOFT;
	  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	  hspi1.Init.CRCPolynomial = 10;
	  if (HAL_SPI_Init(&hspi1) != HAL_OK)
	  {
	    // Error handler - not implemented
	  }

}

/**
 * @brief Low level implementation of SPI and SPI gpio init
 * needed as callback function implemented in HAL
 *
 * @param spiHandle
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI_INSTANCE)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();
    // GPIO enable and init for SPI periph.
    bsp_spi_gpio_init();
  }

}

void bsp_spi_tmc2130_send(uint8_t cmd, uint32_t data)
{
	// Variable, 5x8bits=40bits for communication via SPI
	uint8_t spi[5]={0};
	// First byte is command (write/read + register)
	spi[0]=cmd;
	// Other bytes set
	spi[1]=((data>>24)&0xFF);
	spi[2]=((data>>16)&0xFF);
	spi[3]=((data>>8)&0xFF);
	spi[4]=((data>>0)&0xFF);
	//Transmit via SPI
	HAL_SPI_Transmit(&hspi1,spi,5,50);
}


