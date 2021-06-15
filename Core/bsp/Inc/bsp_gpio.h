/**
  ******************************************************************************
  * @file    bsp_gpio.h
  * @brief   This file contains all the function prototypes for
  *          the bsp_gpio.c file
  ******************************************************************************
  *
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_GPIO_H__
#define __BSP_GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
//#include "main.h"

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

/* Public defines -----------------------------------------------------------*/

// Stepper driver defines
// Chip select GPIO defines
// X axis motors
#define CS_1_PIN            GPIO_PIN_4
#define CS_1_GPIO_PORT      GPIOC
#define CS_2_PIN            GPIO_PIN_5
#define CS_2_GPIO_PORT      GPIOC
// Y axis motors
#define CS_3_PIN            GPIO_PIN_0
#define CS_3_GPIO_PORT      GPIOB
#define CS_4_PIN            GPIO_PIN_1
#define CS_4_GPIO_PORT      GPIOB

// SPIO GPIO defines
/**
 * SPI1 GPIO Configuration
PA5     ------> SPI1_SCK
PA6     ------> SPI1_MISO
PA7     ------> SPI1_MOSI
*/
#define SPI_SCK_PIN         GPIO_PIN_5
#define SPI_SCK_GPIO_PORT   GPIOA
#define SPI_MISO_PIN        GPIO_PIN_6
#define SPI_MISO_GPIO_PORT  GPIOA
#define SPI_MOSI_PIN        GPIO_PIN_7
#define SPI_MOSI_GPIO_PORT  GPIOA

// Step and direction defines for both axes
#define STEP_X_PIN          GPIO_PIN_8
#define STEP_X_GPIO_PORT    GPIOA
#define STEP_Y_PIN          GPIO_PIN_9
#define STEP_Y_GPIO_PORT    GPIOC
#define DIR_X_PIN           GPIO_PIN_9
#define DIR_X_GPIO_PORT     GPIOA
#define DIR_Y_PIN           GPIO_PIN_8
#define DIR_Y_GPIO_PORT     GPIOC

// Enable pin define for stepper driver
#define EN_PIN              GPIO_PIN_10
#define EN_GPIO_PORT        GPIOA

// Define for BJT/mosfet voltage control to drivers
#define BJT_PIN             GPIO_PIN_13
#define BJT_GPIO_PORT       GPIOD

// Usart2 pin defines
#define USART2_TX_PIN       GPIO_PIN_2
#define USART2_TX_GPIO_PORT GPIOA
#define USART2_RX_PIN       GPIO_PIN_3
#define USART2_RX_GPIO_PORT GPIOA

// Define for PCB specific LEDs
#define LED1_PIN            GPIO_PIN_12
#define LED1_GPIO_PORT      GPIOD
#define LED2_PIN            GPIO_PIN_14
#define LED2_GPIO_PORT      GPIOD

// Defines for unused components
// ADC not initialized and unused currently
#define ADC_PIN             GPIO_PIN_1
#define ADC_GPIO_PORT       GPIOA
// DAC not initialized and unused currently
#define DAC_PIN             GPIO_PIN_4
#define DAC_GPIO_PORT       GPIOA
// Free GPIO pin for further use
#define IO_PIN              GPIO_PIN_15
#define IO_GPIO_PORT        GPIOD

/*Public function prototypes*****************************/

/**
 * Initialization of all GPIO peripherals
 */
void bsp_gpio_init(void);

/**
 * USART2 GPIO pins initialization
 */
void bsp_usart2_gpio_init(void);

/**
 * SPI GPIO pins initialization
 */
void bsp_spi_gpio_init(void);



void tmc_init_gpio(void);
void tmc_step(uint32_t micros, bool dir);

#ifdef __cplusplus
}
#endif
#endif /*__ BSP_GPIO_H__ */

/*****************************END OF FILE****/
