/**
  ******************************************************************************
  * @file    bsp_tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
  ******************************************************************************
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <bsp_tim.h>


/* Private defines ------------------------------------------------------------------*/

/* Private data types ------------------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* Private function prototypes ------------------------------------------------------------------*/

/**
 * @brief Function fetches current value of ticks
 * in microseconds
 *
 * @return current value in usec
 */
static uint32_t bsp_tim_get_usec(void);


/* Public functions ------------------------------------------------------------------*/

void bsp_tim_init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    // TIM2 instance settings
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 1-1;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 720-1;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
      // Error handler - not implemented
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        // Error handler - not implemented
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        // Error handler - not implemented
    }

    /* TIM2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

}

/**
 * @brief HAL defined callback function
 *
 * @param htim timer instance
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim -> Instance == TIM2)
	{
		timer2_ticks_usec++;
	}
}

/**
 * @brief HAL defined function for clock initialization
 *
 * @param tim_baseHandle timer handle
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
  }
}

void bsp_tim_wait_usec(uint32_t usec)
{
	uint32_t t1, t2;
	t1 = timer2_get_usec();
	for (; ;)
	{
		t2 = timer2_get_usec();
		if ((t2 - t1) >= usec)
			break;
		if (t2 < t1)
			break;
	}

}
/* Private functions ------------------------------------------------------------------*/

static uint32_t bsp_tim_get_usec(void)
{
	uint32_t value;
	NVIC_DisableIRQ(TIM2_IRQn);
	value = timer2_ticks_usec;
	NVIC_EnableIRQ(TIM2_IRQn);
	return value;
}
