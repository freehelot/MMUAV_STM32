/**
  ******************************************************************************
  * @file    bsp_tim.h
  * @brief   This file contains all the function prototypes for
  *          the bsp_tim.c file
  ******************************************************************************
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_TIM_H__
#define __BSP_TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"

#include <stdint.h>

/* Public defines ------------------------------------------------------------------*/

/* Public data types ------------------------------------------------------------------*/

// handle of timer2
extern TIM_HandleTypeDef htim2;
// timer value in microseconds
extern uint32_t timer2_ticks_usec;

/* Public function prototypes ------------------------------------------------------------------*/

/**
 * @brief Initializes timer peripheral
 *
 */
void bsp_tim_init(void);

/**
 * @brief Wait for period of time in non-blocking mode
 *
 * @param us microseconds
 */
void bsp_tim_wait_usec(uint32_t us);


#ifdef __cplusplus
}
#endif

#endif /* __BSP_TIM_H__ */

/*****************************END OF FILE****/
