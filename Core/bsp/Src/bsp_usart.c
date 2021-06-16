/**
  ******************************************************************************
  * @file    bsp_usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_usart.h"

// Private defines

// Private data types
UART_HandleTypeDef huart2;
// Private function prototypes

// Public functions

void bsp_usart_init(void)
{
	  huart2.Instance = USART2;
	  huart2.Init.BaudRate = 115200 ;
	  huart2.Init.WordLength = UART_WORDLENGTH_8B;
	  huart2.Init.StopBits = UART_STOPBITS_1;
	  huart2.Init.Parity = UART_PARITY_NONE;
	  huart2.Init.Mode = UART_MODE_TX_RX;
	  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	  if (HAL_UART_Init(&huart2) != HAL_OK)
	  {
	    // Error handler - not implemented
	  }
      // Enables UART interrupt
	  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

}

/**
 * @brief HAL specific initialization function called via HAL_UART_Init(&huart2)
 *
 * @param uartHandle
 */
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
	 if(uartHandle->Instance==USART2)
	 {
		 // USART2 clock enabled
		 __HAL_RCC_USART2_CLK_ENABLE();
		 // USART2 GPIO enable
		 bsp_usart2_gpio_init();

		 // USART2 Interrupt init and NVIC priority
		 HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
		 HAL_NVIC_EnableIRQ(USART2_IRQn);

	 }
}

// User specific function for IRQ handling
// Called in stm32f4xx_it.c
/**
 * @brief User specific function for IRQ handling called in stm32f4xx_it.c
 *
 * @param huart uart handle
 */
void USER_UART_IRQHandler( UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		rx_data = __HAL_UART_FLUSH_DRREGISTER(huart);
		static char rx_head;
		rx_head = RX_BUFFER_HEAD + 1;
		if(rx_head == BUFSIZE)
		{
			rx_head = 0;
		}
		if(rx_head != RX_BUFFER_TAIL)
		{
			RX_BUFFER[RX_BUFFER_HEAD] = rx_data;
			RX_BUFFER_HEAD = rx_head;
		}
	}
}

// Private functions


/****END OF FILE****/
