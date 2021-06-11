/*
 * tmc_io.h
 *
 *  Created on: 6 Apr 2021
 *      Author: hrvoje
 */

#ifndef INC_TMC_IO_H_
#define INC_TMC_IO_H_

/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "register.h"
#include <gpio.h>
#include <spi.h>
#include <TMC2130/TMC2130_Mask_Shift.h>
#include <bits.h>
#include <tim.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define CS_1_Pin GPIO_PIN_4
#define CS_1_GPIO_Port GPIOC
#define STEP_Pin GPIO_PIN_8
#define STEP_GPIO_Port GPIOA
#define DIR_Pin GPIO_PIN_9
#define DIR_GPIO_Port GPIOA
#define EN_Pin GPIO_PIN_10
#define EN_GPIO_Port GPIOA

// enum for drv status flags
typedef enum
{
	START = 0,
	STALL = 1,
	OTEMP = 2,
	OTEMPW = 3,
	S2GA = 4,
	S2GB = 5,
	OLA = 6,
	OLB = 7,
	STAND = 8
} drv_status_flags_t;

typedef enum
{
	SPI_STAND = 0,
	SPI_STALL = 1,
	SPI_DR_ERR = 2,
	SPI_RESET = 3,
	DEFAULT = 4
}spi_status_t;
/* USER CODE END Private defines */


extern SPI_HandleTypeDef hspi1;




/* USER CODE BEGIN Prototypes */


/**
 * Simple function to write to TMC2130 via SPI.
 *
 * @param cmd Register address
 * @param data Data to be written to register
 */
void tmc_io_write( uint8_t cmd,uint32_t data);

/**
 * Set all data to 0 in registers. Must do during reset.
 */
void tmc_io_init(void);

/**
 * Get status of registers on logic analyzer
 */
void tmc_io_status(void);

/**
 * Setting microsteps depending on mode chosen
 * Initial function for testing chopconf
 * @param mode Mode selection for microstep setting
 */
void tmc_io_config(uint8_t mode);

/**
 * Set inital state of GPIO pins CS_1, DIR, STEP and EN
 */
void tmc_io_gpio(void);

/**
 * Function for doing STEP for defined time in milliseconds
 * @param millis Time of delay in milliseconds
 */
void tmc_io_step_millis(uint32_t millis);


/**
 * Function for doing step for defined time in usecs
 * @param usec Time of delay in usec
 */
void tmc_io_step(uint32_t usec);

/**
 * Enable TMC stepper driver
 */
void tmc_enable(void);

/**
 * Disable TMC stepper driver
 */
void tmc_disable(void);
/**
 * Chopconf configuration via input arguments for some register value
 */
uint32_t set_chopconf(uint8_t toff, uint8_t hstrt, uint8_t hend, uint8_t tbl, uint8_t vsense, uint8_t sync, uint8_t mres);

/**
 * Chopconf configuration via chopconf structure that was initialized or changed beforehand
 *
 * return 32bit data for SPI write/read
 */
uint32_t set_chopconf_struct(chopconf_t chop);

/**
 * Gconf configuration via gconf structure that was initialized or changed beforehand
 *
 * return 32bit data for SPI write/read
 */
uint32_t set_gconf_struct(gconf_t gconf);

/**
 * Coolconf configuration via coolconf structure that was initialized or changed beforehand
 *
 * return 32bit data for SPI write/read
 */
uint32_t set_coolconf_struct(coolconf_t cool);

/**
 * IHOLDRUN configuration via iholdrun structure that was initialized or changed beforehand
 *
 * return 32bit data for SPI write/read
 */
uint32_t set_iholdrun_struct(ihold_irun_t holdrun);

/**
 * PWMCONF configuration via pwmconf structure that was initialized or changed beforehand
 *
 * return 32bit data for SPI write/read
 */
uint32_t set_pwmconf_struct(pwmconf_t pwmconf);

/**
 *  Filling drv_status struct with data read from SPI
 *
 *  return struct with filled data
 */
drv_status_t get_drv_status(uint32_t data);


/**
 * Gets stallguard value from drv_status
 *
 * return stallguard value in uint32_t data value
 */
uint32_t get_sg_result(uint32_t data);


/**
 * Gets actual motor current
 *
 * return motor current in uint32_t value
 */
uint32_t get_cs_actual(uint32_t data);

/**
 * Gets SPI status bits
 *
 * return enum spi_status flag
 */
spi_status_t get_spi_status(uint8_t cmd);
/* USER CODE END Prototypes */


void ramp_stepper(uint32_t step, uint32_t repeat);

void accelerate(uint32_t start, uint32_t end);

void decelerate(uint32_t start, uint32_t end);

void const_speed(uint32_t step, uint32_t repeat);

#endif /* INC_TMC_IO_H_ */
