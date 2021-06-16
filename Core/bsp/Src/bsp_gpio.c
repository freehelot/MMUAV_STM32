/**
  ******************************************************************************
  * @file    bsp_gpio.c
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

/**
 * @brief Initialization of all stepper driver GPIO pins except
 * SPI related pins
 *
 */
static void bsp_stepper_gpio_init(void);

/**
 * @brief Initialization of bjt mosfet control pin
 *
 */
static void bsp_bjt_gpio_init(void);

/**
 * @brief Initialization of LED1 and LED2 on PCB
 *
 */
static void bsp_led_gpio_init(void);

/* Public functions-------------------------------------------*/

void bsp_gpio_init(void)
{


    // GPIO port clock enable
    __HAL_RCC_GPIOH_CLK_ENABLE();

    // Initialization calls for GPIO peripherals
    bsp_stepper_gpio_init();
    bsp_bjt_gpio_init();
    bsp_led_gpio_init();
    //bsp_usart2_gpio_init();
    //bsp_spi_gpio_init();

}

void bsp_usart2_gpio_init(void)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};
	// Clock enable
    __HAL_RCC_GPIOA_CLK_ENABLE();
    // Pin configuration
    GPIO_InitStruct.Pin = USART2_RX_PIN|USART2_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

void bsp_spi_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
	// Clock enable
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = SPI_SCK_PIN|SPI_MISO_PIN|SPI_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
/* Private functions-------------------------------------------*/

static void bsp_stepper_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
	// Clock enable
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // Chip select gpio output level init
    HAL_GPIO_WritePin(CS_1_GPIO_PORT, CS_1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CS_2_GPIO_PORT, CS_2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CS_3_GPIO_PORT, CS_3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CS_4_GPIO_PORT, CS_4_PIN, GPIO_PIN_RESET);
    // Dir, step and enable gpio output level init
    HAL_GPIO_WritePin(GPIOA, STEP_X_PIN|DIR_X_PIN|EN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, STEP_Y_PIN|DIR_Y_PIN, GPIO_PIN_RESET);

    // Configuration of Pins
    // GPIOC
    GPIO_InitStruct.Pin = CS_1_PIN|CS_2_PIN|STEP_Y_PIN|DIR_Y_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    // GPIOB
    GPIO_InitStruct.Pin = CS_3_PIN|CS_4_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    // GPIOA
    GPIO_InitStruct.Pin = STEP_X_PIN|DIR_X_PIN|EN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

static void bsp_bjt_gpio_init(void)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};
	// Clock enable
    __HAL_RCC_GPIOD_CLK_ENABLE();
    // GPIO output level init
    HAL_GPIO_WritePin(BJT_GPIO_PORT, BJT_PIN, GPIO_PIN_RESET);
    // Pin configuration
    GPIO_InitStruct.Pin = BJT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BJT_GPIO_PORT, &GPIO_InitStruct);

}

static void bsp_led_gpio_init(void)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};
	// Clock enable
    __HAL_RCC_GPIOD_CLK_ENABLE();
    // GPIO output level init
    HAL_GPIO_WritePin(GPIOD, LED1_PIN|LED2_PIN, GPIO_PIN_RESET);
    // Pin configuration
    GPIO_InitStruct.Pin = LED1_PIN|LED2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}


