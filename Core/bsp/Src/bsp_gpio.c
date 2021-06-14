/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  *
  * 
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <bsp_gpio.h>

/* Private function prototypes-------------------------------------------*/

static bool bsp_bjt_gpio_init(void);

static bool bsp_led_gpio_init(void);

static bool bsp_stepper_gpio_init(void);

/* Public functions-------------------------------------------*/

void bsp_gpio_init(void)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};

//  GPIO port clock enable
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    // Chip select gpio output level init
    HAL_GPIO_WritePin(CS_1_GPIO_Port, CS_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CS_2_GPIO_Port, CS_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CS_3_GPIO_Port, CS_3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CS_4_GPIO_Port, CS_4_Pin, GPIO_PIN_RESET);

}

/* Private functions-------------------------------------------*/
