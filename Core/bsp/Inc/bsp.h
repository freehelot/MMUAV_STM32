/**
  ******************************************************************************
  * @file    bsp.h
  * @brief   This file contains all the function prototypes for
  *          the bsp.c file
  ******************************************************************************
  *
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_H__
#define __BSP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

#include <bsp_gpio.h>
#include <bsp_usart.h>
#include <bsp_spi.h>




/* Private function prototypes ------------------------------------------------------------------*/

/**
 * Initializes all BSP
 */
void bsp_init(void);

#ifdef __cplusplus
}
#endif
#endif /*__ BSP_H__ */

/*****************************END OF FILE****/
