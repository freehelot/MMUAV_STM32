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
#define BUFSIZE    (32U)

/* Public data types ------------------------------------------------------------------*/

extern UART_HandleTypeDef huart2;
// RX_BUFFER holds all unprocessed incoming characters. Buffer is organized as FIFO queue,
// using static C character array.
// Buffering of incoming characters enables less strict timings in the main control loop,
// which can be even more relaxed by increasing the FIFO depth (BUFSIZE constant).
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
 * A high level function that sends a single character through the USART1 interface;
 * the function guarantees that character will be sent, but does not provide any guarantees about the timing
 *
 * @param single character
 */
void bsp_usart_send_char(uint8_t c);



/**
 * @brief Fetch a single character from queue of received characters
 * in case of multiple characters waiting in queue
 * the one first arrived will be fetched
 * Function is non-blocking
 *
 *Function fetches a single character from a queue of received characters. In case that there are more
 *then one character waiting to be processed in a queue, the one that first arrived
 *will be fetched. Function is non-blocking, meaning that it will not block on empty queue
 *(it will return 0 if there are no characters to be processed);
 *otherwise, the function will return 1 and fetched character will be returned through
 *the bzref parameter (suplied as a pointer to char).
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
