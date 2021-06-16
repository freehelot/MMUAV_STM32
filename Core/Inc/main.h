/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include <bsp.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/


/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
// Stepper driver defines
// Chip select GPIO defines
// X axis motors
#define CS_1_Pin            GPIO_PIN_4
#define CS_1_GPIO_Port      GPIOC
#define CS_2_Pin            GPIO_PIN_5
#define CS_2_GPIO_Port      GPIOC
// Y axis motors
#define CS_3_Pin            GPIO_PIN_0
#define CS_3_GPIO_Port      GPIOB
#define CS_4_Pin            GPIO_PIN_1
#define CS_4_GPIO_Port      GPIOB

// SPIO GPIO defines
/**
 * SPI1 GPIO Configuration
PA5     ------> SPI1_SCK
PA6     ------> SPI1_MISO
PA7     ------> SPI1_MOSI
*/
#define SPI_SCK_PIN         GPIO_PIN_3
#define SPI_SCK_GPIO_PORT   GPIOB
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


// unused, replace later
#define STEP_Pin            GPIO_PIN_8
#define STEP_GPIO_Port      GPIOA
#define DIR_Pin             GPIO_PIN_9
#define DIR_GPIO_Port       GPIOA
#define EN_Pin              GPIO_PIN_10
#define EN_GPIO_Port        GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
