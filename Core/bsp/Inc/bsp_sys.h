/**
  ******************************************************************************
  * @file    bsp_sys.h
  * @brief   This file contains all the function prototypes for
  *          the bsp_sys.c file
  ******************************************************************************
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_SYS_H__
#define __BSP_SYS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Public defines ------------------------------------------------------------------*/

/* Public data types ------------------------------------------------------------------*/

/* Public function prototypes ------------------------------------------------------------------*/


/**
  * @brief System Clock Configuration
  * @retval None
  */
void bsp_sys_clock_config(void);

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void);


#ifdef __cplusplus
}
#endif

#endif /* __BSP_SYS_H__ */

/*****************************END OF FILE****/
