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
//#include "spi.h"
//#include "tim.h"
//#include "usart.h"
//#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <bits.h>
#include <register.h>
#include <stdbool.h>

#include <stdlib.h>
#include <stm32f4xx_hal_uart.h>

#include <string.h>
/* USER CODE END Includes */
UART_HandleTypeDef huart2;
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Constants for stepper gear calculations
#define STEP_ANGLE                (18U)
#define STEP_ANGLE_DIV            (10U)
#define PITCH_DIAM                (312U)
#define PITCH_DIV                 (10U)
#define FULL_ANGLE                (360U)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DELAY_MS                  (10U)
#define MODE_SELECT               (6U)
/* USER CODE END PM */
#define MAX_LENGTH				(162U) //mm
#define MIDDLE_LENGTH           (81U)  // 0mm center of arm
#define LED_RED					  (0U)
#define LED_GREEN    			  (0U)

//#define USEC_STEP				(20U)
#define USEC_STEP				(100U)
#define USEC_NONE				(0U)

#define TOOTH_COUNT				(25U)
#define MICRO_STEP				(4U)
/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t drv[5] = {0};
uint8_t crv[5];
uint8_t vrv[5];
//uint32_t timer2_Ticks_usec;
uint32_t timer2_ticks_usec;
//usart related PV
//char RX_BUFFER[BUFSIZE];
//int RX_BUFFER_HEAD, RX_BUFFER_TAIL;
//uint8_t rx_data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

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
	timer2_ticks_usec = 0;

	uint32_t pulses_x = 0;
	uint32_t pulses_y = 0;
	uint32_t pulses = 0;
	bool dir_x = true;
	bool dir_y = true;
	uint32_t mov_x = 0;
	uint32_t mov_y = 0;
	bool check_x = false;

	uint32_t mov_des_x = 0;
	uint32_t mov_des_y = 0;
	//char Message[] = "\n Confirmation received\r\n";

	uint32_t pos_x = MIDDLE_LENGTH;
	uint32_t pos_y = MIDDLE_LENGTH;

	bool x_on = true;
	bool y_on = true;


  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();



  /* Initialize all configured peripherals */
  bsp_init();

  mw_tmc2130_io_init();

  bsp_gpio_led_toggle(1);

  // CALIBRATION
  mw_tmc2130_io_config_all(MODE_SELECT);
  mw_tmc2130_io_calib(X_AXIS, 300);
  //mw_tmc2130_io_calib(Y_AXIS, 300);
  //mw_tmc2130_io_deinit();
  HAL_Delay(1000);
  // MIcrostepping select
  //mw_tmc2130_io_init();
  //mw_tmc2130_io_config_all(MODE_SELECT);
  bsp_gpio_led_toggle(1);



  /* Infinite loop */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //HAL_UART_Transmit(&huart2,(uint8_t *)Message, strlen(Message), 100);
	  //HAL_Delay(500);
	  //bsp_usart_send_char(c);
	  //HAL_Delay(500);
//    /* USER CODE END WHILE */
//    /* USER CODE BEGIN 3 */
//	  //timer2_wait_usec(5);
//	  // Expect XXX numbers
//	  // Checks if the command in mm has been given
//	  if(!check)
//	  {
//		  // Waits for command via uart
	  	  mw_com_command_all(&dir_x, &dir_y, &mov_x, &mov_y, &check_x);
		  bsp_gpio_led_toggle(1);


		  //mov_x = mw_fun_pos(mov_x, &dir_x, &pos_x);
		  //mov_y = mw_fun_pos(mov_y, &dir_y, &pos_y);
	  	  //if(check_x && check_y)
		  if(check_x )
	  	  {
			  mov_des_x = mw_fun_pos(mov_x, &dir_x, &pos_x);
			  mov_des_y = mw_fun_pos(mov_y, &dir_y, &pos_y);
			  if(pos_x == 0)
			  {
				  pos_x = 81;
			  }
			  if(dir_x == false)
			  {
				  pos_x = 81;
			  }

	  		  x_on = true;
	  		  y_on = true;
	  		  //HAL_UART_Transmit(&huart2,(uint8_t *)Message, strlen(Message), 100);
	  		  //pulses = (FULL_ANGLE * mov_x * PITCH_DIV * STEP_ANGLE_DIV) / (STEP_ANGLE * PITCH_DIAM);
	  		  //pulses = ( mov_x  * STEP_ANGLE_DIV) / (STEP_ANGLE);
	  		  //pulses = (2*2*mov_x * 641)/ 314 ;
	  		  //pulses = (2*2*mov_x * 65)/ 30 ;
	  		  pulses_x = mw_fun_pulses(mov_des_x, 4);
	  		  pulses_y = mw_fun_pulses(mov_des_y, 4);
	  		  if(pulses_x >= pulses_y)
	  		  {
	  			  pulses = pulses_x;
	  		  }
	  		  else
	  		  {
	  			  pulses = pulses_y;
	  		  }
	  		  for(uint32_t i = 0; i < pulses; i++)
	  		  {
	  			  if(i >= pulses_x)
	  			  {
	  				  x_on = 0;
	  			  }
	  			  if(i >= pulses_y)
	  			  {
	  				  y_on = 0;
	  			  }
	  			  mw_tmc2130_io_step_all(x_on, y_on, dir_x, dir_y, USEC_STEP);


	  		  }
	  		  //HAL_UART_Transmit(&huart2,(uint8_t *)Message, strlen(Message), 100);

/*
	  		  for(uint32_t i = 0; i<pulses; i++)
	  				  {

	  			  	  	  if(pulses_x < i)
	  			  	  	  {
		  					  mw_tmc2130_io_step(X_AXIS, dir_x, USEC_STEP);
	  			  	  	  }
	  			  	  	  else
	  			  	  	  {

	  			  	  	  }


	  				  }*/
	  		  //pulses = (2*2*mov_y * 65)/ 30 ;
	  		  //pulses = mw_fun_pulses(mov_y);
	  		  		 // for(uint32_t i = 0; i<pulses; i++)
	  		  		  //{
	  		  			//  mw_tmc2130_io_step(Y_AXIS, dir_y, 20);
	  		  		 // }
	  		check_x = false;
	  	  }
	  	  bsp_gpio_led_toggle(1);
	  	  check_x = false;
	  	  //check_y = false;
//		  if(USART2_Dequeue(&c)!=0)
//		  	  {
//			  	  	  // Check first symbol
//
//		  			  if(valid)
//		  			  {
//		  				 USART2_SendChar(c);
//		  				 if(c == '-')
//		  				 {
//		  					 direction = false;
//		  				 }
//		  				 else
//		  				 {
//		  					 direction = true;
//		  				 }
//		  				 valid = false;
//		  			  }
//		  			  else
//		  			  {
//		  				  USART2_SendChar(c);
//		  				  command[counter]=c;
//		  				  counter++;
//		  				  if(counter > (size - 1))
//		  				  {
//		  					  counter = 0; // Resets counter
//		  					  check = true; // Command received
//		  					  valid = true;
//		  					  movement = (uint32_t)atoi(command);
//		  					  /*if(movement > 255)
//								{
//									movement = 240;
//		  						}
//		  						else if(movement < 0)
//		  						{
//		  						  	movement = 0;
//		  						}*/
//		  				  }
//
//		  			  }
//
//		  	  }
//	  }
//	  if(check)
//	  {
//		  //pulses = (FULL_ANGLE * movement * PITCH_DIV * STEP_ANGLE_DIV) / (STEP_ANGLE * PITCH_DIAM);
//		  //pulses = ( movement  * STEP_ANGLE_DIV) / (STEP_ANGLE);
//		  //pulses = (2*2*movement * 641)/ 314 ;
//		  pulses = (2*2*movement * 65)/ 30 ;
//		  for(uint32_t i = 0; i<pulses; i++)
//		  {
//			  tmc_step(20,direction);
//		  }
//		  check = false; // command fullfilled
//	  }
//
//
//	  //tmc_step(100);
//			//HAL_SPI_Transmit(&hspi1, drv, 5, 20);
//			//HAL_GPIO_WritePin(CS_1_GPIO_Port,CS_1_Pin,GPIO_PIN_SET);
//
//		  	//HAL_Delay(100);
//		  	//HAL_GPIO_WritePin(CS_1_GPIO_Port,CS_1_Pin,GPIO_PIN_RESET);
//		  	//tmc_io_write(REG_CHOPCONF,set_chopconf(0,0 , 0, 0, 0, 0, 0));
//			//HAL_SPI_Transmit(&hspi1, drv, 5, 20);
//			//HAL_GPIO_WritePin(CS_1_GPIO_Port,CS_1_Pin,GPIO_PIN_SET);
//
//		  	//HAL_Delay(100);
//		  	//tmc_io_write(TMC2130_WRITE, set_chopconf_struct(choppy));
//
//
//
//  }
  }
  /* USER CODE END 3 */
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
