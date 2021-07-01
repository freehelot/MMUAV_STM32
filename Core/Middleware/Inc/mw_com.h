/**
  ******************************************************************************
  * @file    mw_com.h
  * @brief   This file contains all the function prototypes for
  *          the mw_com.c file
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MW_COM_H__
#define __MW_COM_H__


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include <stdlib.h>

#include <bsp_gpio.h>
#include <bsp_usart.h>
/* Public defines ------------------------------------------------------------------*/

/* Public data types ------------------------------------------------------------------*/


/* Public function prototypes ------------------------------------------------------------------*/


/**
 * @brief
 *
 * @param dir
 * @param movement
 * @return
 */
void mw_com_command(bool *dir, uint32_t *movement, bool *confirm);


/**
 *
 */
void mw_com_command_all(bool *dirx, bool *diry, uint32_t *movx, uint32_t *movy, bool *confirm);


#ifdef __cplusplus
}
#endif

#endif /*__MW_COM_H__ */

/*****************************END OF FILE****/
