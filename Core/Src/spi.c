/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "spi.h"

/* USER CODE BEGIN 0 */
typedef enum
{
	mode_0 =0, //256U
	mode_1 =1, //128U
	mode_2 =2, //64U
	mode_3 =3, //32U
	mode_4 =4, //16U
	mode_5 =5, //8U
	mode_6 =6, //4U
	mode_7 =7, //2U
	mode_8 =8, //1U
	mode_default = mode_0
}chop_mode_t;

/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
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
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
 *
 */
void tmc_write( uint8_t cmd,uint32_t data){

	uint8_t spi[5]={0};

	HAL_GPIO_WritePin(CS_1_GPIO_Port,CS_1_Pin,GPIO_PIN_RESET); //CS pin low
	spi[0]=cmd;
	//HAL_SPI_Transmit(&hspi1,spi,5,50);
	//0x00001010
	//0x0000101000
	spi[1]=((data>>24)&0xFF);
	spi[2]=((data>>16)&0xFF);
	spi[3]=((data>>8)&0xFF);
	spi[4]=((data>>0)&0xFF);
	HAL_SPI_Transmit(&hspi1,spi,5,50);
	HAL_GPIO_WritePin(CS_1_GPIO_Port,CS_1_Pin,GPIO_PIN_SET); //CS pin low

}
/**
 *
 */
void tmc_init(void)
{
	tmc_write(REG_GCONF|TMC2130_WRITE, TMC_INIT_DATA);
	tmc_write(REG_GCONF|TMC2130_WRITE, TMC_INIT_DATA);
	tmc_write(REG_CHOPCONF|TMC2130_WRITE, TMC_INIT_DATA);
	tmc_write(REG_COOLCONF|TMC2130_WRITE, TMC_INIT_DATA);
	tmc_write(REG_IHOLD_IRUN|TMC2130_WRITE, TMC_INIT_DATA);

}

void tmc_set_config(uint8_t mode)
{
	uint32_t data;
	switch(mode)
	{
		case mode_0:
			data=0x00008008UL;
		break;
		case mode_1:
			data=0x02008008UL;
		break;
		case mode_2:
			data=0x03008008UL;
		break;
		case mode_3:
			data=0x04008008UL;
		break;
		case mode_4:
			data=0x05008008UL;
		break;
		case mode_5:
			data=0x06008008UL;
		break;
		case mode_6:
			data=0x07008008UL;
		break;
		case mode_7:
			data=0x08008008UL;
		break;
		case mode_8:
			data=0x00008008UL;
		break;
		default:
			data = 0x00008008UL;
		break;
	}
	tmc_write(TMC2130_WRITE|REG_GCONF, 0x00000001UL);
	tmc_write(TMC2130_WRITE|REG_IHOLD_IRUN, 0x00001010UL);
	tmc_write(TMC2130_WRITE|REG_CHOPCONF, data);

	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET); //low EN for enabling of driver
}

void tmc_get_status(void)
{
	tmc_write(TMC2130_READ|REG_GCONF, 0x00000000UL);
	tmc_write(TMC2130_READ|REG_GCONF, 0x00000000UL);
	tmc_write(TMC2130_READ|REG_IHOLD_IRUN, 0x00000000UL);
	tmc_write(TMC2130_READ|REG_IHOLD_IRUN, 0x00000000UL);
	tmc_write(TMC2130_READ|REG_CHOPCONF, 0x00000000UL);
	tmc_write(TMC2130_READ|REG_CHOPCONF, 0x00000000UL);

}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
