/*
 * tmc_io.h
 *
 *  Created on: 6 Apr 2021
 *      Author: hrvoje
 */

#ifndef INC_TMC_IO_H_
#define INC_TMC_IO_H_

/* USER CODE BEGIN Includes */
// Library includes
#include <stdint.h>
#include <stdbool.h>
// BSP includes
//#include <gpio.h>
//#include <tim.h>
//#include <spi.h>
#include <bsp_gpio.h>
#include <bsp_spi.h>
#include <bsp_tim.h>
#include <mw_fun.h>

// TMC specific includes
#include <TMC2130/TMC2130_Mask_Shift.h>
#include <bits.h>
#include "register.h"


/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/*
#define CS_1_Pin GPIO_PIN_4
#define CS_1_GPIO_Port GPIOC
#define STEP_Pin GPIO_PIN_8
#define STEP_GPIO_Port GPIOA
#define DIR_Pin GPIO_PIN_9
#define DIR_GPIO_Port GPIOA
#define EN_Pin GPIO_PIN_10
#define EN_GPIO_Port GPIOA
*/
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

//CS1 and CS2 are X axes motors
#define X_MOT_1  (1U)
#define X_MOT_2  (2U)
#define X_AXIS   (1U)
//CS3 and CS4 are X axes motors
#define Y_MOT_1  (3U)
#define Y_MOT_2  (4U)
#define Y_AXIS   (2U)
/* USER CODE END Private defines */


extern SPI_HandleTypeDef hspi1;




/* USER CODE BEGIN Prototypes */


/**
 * @brief Middleware that uses SPI for writing to stepper driver
 * Should probably be static -> TODO: implement that
 *
 * @param cs Chip select
 * @param cmd Command for R/W + register where to R/W
 * @param data Data to be sent to register
 */
void mw_tmc2130_io_write(uint8_t cs, uint8_t cmd, uint32_t data);


/**
 * @brief Middleware that uses SPI for writing to all stepper driver
 *
 * @param cmd Command for R/W + register where to R/W
 * @param data Data to be sent to register
 */
void mw_tmc2130_io_write_all(uint8_t cmd, uint32_t data);




/**
 * @brief Middleware that uses SPI for writing to selected axis stepper drivers
 *
 * @param axis desired duo of stepper drivers
 * @param cmd Command for R/W + register where to R/W
 * @param data Data to be sent to register
 */

void mw_tmc2130_io_write_axis(uint8_t axis, uint8_t cmd, uint32_t data);






/**
 * @brief Initializes all stepper drivers
 *
 */
void mw_tmc2130_io_init(void);


/**
 * @brief Setting microsteps depending on mode chosen
 * Initial function for testing chopconf
 * @param mode Mode selection for microstep setting
 */
void mw_tmc2130_io_config_all(uint8_t mode);


/**
 * @brief Function for doing step on axis in usecs
 *
 * @param axis Desired axis of movement
 * @param dir  Desired direction of movement
 * @param usec Step time in microseconds
 */
void mw_tmc2130_io_step(uint8_t axis, bool dir, uint32_t usec);



/**
 *
 */
void mw_tmc2130_io_step_all(bool x, bool y, bool dir_x, bool dir_y, uint32_t usec);


/**
 * Get status of registers on logic analyzer
 */
void mw_tmc2130_io_status(void);


/**
 *
 */
void mw_tmc2130_io_calib(uint8_t axis, uint32_t usec);

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
