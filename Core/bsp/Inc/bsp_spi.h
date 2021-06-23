/**
  ******************************************************************************
  * @file    bsp_spi.h
  * @brief   This file contains all the function prototypes for
  *          the bsp_spi.c file
  ******************************************************************************
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_SPI_H__
#define __BSP_SPI_H__


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include <bsp_gpio.h>

// Public and external data types
extern SPI_HandleTypeDef hspi1;

// Public function prototypes
/**
 * @brief Initializes SPI peripheral
 *
 */
void bsp_spi_init(void);


/**
 * Simple function to write to TMC2130 via SPI.
 *
 * @param cmd Register address
 * @param data Data to be written to register
 */
void bsp_spi_tmc2130_send(uint8_t cmd,uint32_t data);



#ifdef __cplusplus
}
#endif

#endif /* __BSP_SPI_H__ */

/****END OF FILE****/
