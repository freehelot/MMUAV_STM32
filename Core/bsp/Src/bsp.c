/**
  ******************************************************************************
  * @file    bsp.c
  * @brief   BSP initialization source file
  ******************************************************************************
  *
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp.h"

/* Private data types ------------------------------------------------------------------*/

/* Private defines ------------------------------------------------------------------*/

/* Private function prototypes ------------------------------------------------------------------*/

/* Public functions ------------------------------------------------------------------*/

void bsp_init(void)
{
	// Inits
    // Sys clock init
    bsp_sys_clock_config();
	// stepper pins, leds, bjt and other GPIO init
	bsp_gpio_init();
	// SPI peripheral init
    bsp_spi_init();
    // USART peripheral init
    bsp_usart_init();
    // TIM2 init
    bsp_tim_init();
    HAL_TIM_Base_Start_IT(&htim2);



}
/* Private functions ------------------------------------------------------------------*/


/****END OF FILE****/
