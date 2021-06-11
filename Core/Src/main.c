/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tmc_io.h"
#include <bits.h>
#include <register.h>
#include <stdbool.h>

#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STEP_ANGLE				(18U)
#define STEP_ANGLE_DIV			(10U)
#define PITCH_DIAM				(312U)
#define PITCH_DIV				(10U)
#define FULL_ANGLE				(360U)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DELAY_MS 10U
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t drv[5] = {0};
uint8_t crv[5];
uint8_t vrv[5];
uint32_t timer2_Ticks_usec;

//usart related PV
char RX_BUFFER[BUFSIZE];
int RX_BUFFER_HEAD, RX_BUFFER_TAIL;
uint8_t rx_data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
/* __GNUC__ */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	timer2_Ticks_usec = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  uint8_t size = 3;
  uint8_t counter = 0;
  char command[size];
  char c;
  bool check = false;
  uint32_t movement = 0;
  bool valid = true;
  bool direction = true;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_NVIC_Init();
  MX_USART2_UART_Init();
  HAL_TIM_Base_Start_IT(&htim2);
  //MX_USART6_UART_Init();
  MX_SPI1_Init();

  /* Initialize interrupts */
  //MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  //HAL_Delay(2000);
  tmc_io_gpio();
  tmc_io_init();//set all regs to zero
  tmc_io_config(5);

  //HAL_Delay(2000);
  //tmc_get_status();
  ///////////////////////////////////
  //timer2 init
  //timer2_Ticks_usec=0;
  //init gpio for tmc, DIR, STEP, EN to high (EN = 1, disabled, EN = 0, enabled)
  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
  int32_t pulses = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
	  timer2_wait_usec(5);
	  // Expect XXX numbers
	  // Checks if the command in mm has been given
	  if(!check)
	  {
		  // Waits for command via uart
		  if(USART2_Dequeue(&c)!=0)
		  	  {
			  	  	  // Check first symbol

		  			  if(valid)
		  			  {
		  				 USART2_SendChar(c);
		  				 if(c == '-')
		  				 {
		  					 direction = false;
		  				 }
		  				 else
		  				 {
		  					 direction = true;
		  				 }
		  				 valid = false;
		  			  }
		  			  else
		  			  {
		  				  USART2_SendChar(c);
		  				  command[counter]=c;
		  				  counter++;
		  				  if(counter > (size - 1))
		  				  {
		  					  counter = 0; // Resets counter
		  					  check = true; // Command received
		  					  valid = true;
		  					  movement = (uint32_t)atoi(command);
		  					  /*if(movement > 255)
								{
									movement = 240;
		  						}
		  						else if(movement < 0)
		  						{
		  						  	movement = 0;
		  						}*/
		  				  }

		  			  }

		  	  }
	  }
	  if(check)
	  {
		  //pulses = (FULL_ANGLE * movement * PITCH_DIV * STEP_ANGLE_DIV) / (STEP_ANGLE * PITCH_DIAM);
		  //pulses = ( movement  * STEP_ANGLE_DIV) / (STEP_ANGLE);
		  //pulses = (2*2*movement * 641)/ 314 ;
		  pulses = (2*2*movement * 65)/ 30 ;
		  for(uint32_t i = 0; i<pulses; i++)
		  {
			  tmc_step(20,direction);
		  }
		  check = false; // command fullfilled
	  }


	  //tmc_step(100);
			//HAL_SPI_Transmit(&hspi1, drv, 5, 20);
			//HAL_GPIO_WritePin(CS_1_GPIO_Port,CS_1_Pin,GPIO_PIN_SET);

		  	//HAL_Delay(100);
		  	//HAL_GPIO_WritePin(CS_1_GPIO_Port,CS_1_Pin,GPIO_PIN_RESET);
		  	//tmc_io_write(REG_CHOPCONF,set_chopconf(0,0 , 0, 0, 0, 0, 0));
			//HAL_SPI_Transmit(&hspi1, drv, 5, 20);
			//HAL_GPIO_WritePin(CS_1_GPIO_Port,CS_1_Pin,GPIO_PIN_SET);

		  	//HAL_Delay(100);
		  	//tmc_io_write(TMC2130_WRITE, set_chopconf_struct(choppy));



  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
