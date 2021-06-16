/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

#include <bsp_gpio.h>
/* Public defines ------------------------------------------------------------------*/

// Size of FIFO buffer
#define BUFSIZE    (16U)

/* Public data types ------------------------------------------------------------------*/

extern UART_HandleTypeDef huart2;

extern char RX_BUFFER[BUFSIZE];
extern int  RX_BUFFER_HEAD, RX_BUFFER_TAIL;
uint8_t rx_data;
/* Public function prototypes ------------------------------------------------------------------*/


/**
 * @brief User implemented function to call when IRQ handler
 * is called by usart2
 *
 * @param huart USART2
 */
void bsp_usart_irq_handler( UART_HandleTypeDef *huart);

/**
 * @brief Initialization of desired USART
 */
void bsp_usart_init(void);

/**
 * @brief Send a single character through USART interface
 * Function guarantees that the character will be sent
 * but does not provide any guarantees about timing
 *
 * @param single character
 */
void bsp_usart_send(uint8_t c);



/**
 * @brief Fetch a single character from queue of received characters
 * in case of multiple characters waiting in queue
 * the one first arrived will be fetched
 * Function is non-blocking
 *
 * @param c character queue
 * @return 0 if no character to be processed, otherwise 1
 */
int bsp_usart_dequeue(char* c);


#ifdef __cplusplus
}
#endif

#endif /* __BSP_USART_H__ */

/*****************************END OF FILE****/
