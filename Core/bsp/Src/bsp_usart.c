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
	  RX_BUFFER_HEAD = 0;
	  RX_BUFFER_TAIL = 0;

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
		 __HAL_RCC_GPIOA_CLK_ENABLE();
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
void bsp_usart_irq_handler( UART_HandleTypeDef *huart)
{
	static char rx_head;
	static char rx_data;
	if(huart->Instance == USART2)
	{	// macro used to store the content of UART data register
		// automaticly clears the  USART interrupt pending bit
		// When a character is received it is placed into global received char queue
		rx_data = __HAL_UART_FLUSH_DRREGISTER(huart);
		//static char rx_head;
		rx_head = RX_BUFFER_HEAD + 1;
		// rest of the code checks if the newly received character will cause the buffer overrun
		// and put it into the buffer only if there is space left
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

void bsp_usart_send_char(uint8_t c)
{
	// this is needed to transmit data in blocking mode, which means that UASRT1 interface
	// is ready for a new transmission
	HAL_UART_Transmit(&huart2, &c, sizeof(c), 10);
}

int bsp_usart_dequeue(char* c)
{
	int ret;
	ret = 0;
	*c = 0;
	HAL_NVIC_DisableIRQ(USART2_IRQn);
	// check whether the buffer is not empty

	if(RX_BUFFER_HEAD != RX_BUFFER_TAIL)
	{
		//iff not empty -> fetch oldest value to byref param (char *c)
		*c = RX_BUFFER[RX_BUFFER_TAIL];
		// adjust the value of the buffer tail
		RX_BUFFER_TAIL++;

		if(RX_BUFFER_TAIL == BUFSIZE)
		{
			RX_BUFFER_TAIL = 0;
		}
		// set ret flag to 1
		// indicates that receiver buffer was not empty
		ret = 1;
	}
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	return ret;
}

// Private functions


/****END OF FILE****/
