/*
 * mw_tmc_io.c
 *
 *  Created on: 6 Apr 2021
 *      Author: hrvoje
 */
/* Includes ------------------------------------------------------------------*/
#include <mw_tmc2130_io.h>

/* Private define ------------------------------------------------------------*/

#define TMC_INIT_DATA (0x00000000)
#define PWM_INIT_DATA (0x00050480)

// unused
//#define INIT_REGISTER(REG) REG##_t REG##_register = REG##_t

// Step limits
#define MAX_STEP (2000U)
#define MIN_STEP (100U)

#define CALIB_PULSE (800U)
#define CENTER_MOV  (81U)
#define USEC_MOV	(20U)
/* Private typedef -----------------------------------------------------------*/

// Microstepping mode
typedef enum
{
	mode_0 =0, //256U
	mode_1 =1, //128U
	mode_2 =2, //64U
	mode_3 =3, //32U
	mode_4 =4, //16U
	mode_5 =5, //8U
	mode_6 =6, //4U
	mode_7 =7, //2U
	mode_8 =8, //1U
	mode_default = mode_0
}chop_mode_t;

/* Private user code ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;




// Reworked functions to remove all instances of hal_gpio or hal_spi calls and instead call bsp functions
// Which are designed to handles such calls to hal_gpio/spi

void mw_tmc2130_io_write(uint8_t cs, uint8_t cmd, uint32_t data)
{
	// Variable, 5x8bits=40bits for communication via SPI
	bsp_gpio_chipselect(cs);
	bsp_spi_tmc2130_send(cmd, data);
	bsp_gpio_chipselect_reset();

}

void mw_tmc2130_io_write_all(uint8_t cmd, uint32_t data){
	bsp_gpio_chipselect_all();
	bsp_spi_tmc2130_send(cmd, data);
	bsp_gpio_chipselect_reset();
}


void mw_tmc2130_io_write_axis(uint8_t axis, uint8_t cmd, uint32_t data)
{
	switch(axis)
	{
	    case X_AXIS:
	    	bsp_gpio_chipselect(X_MOT_1);
	    	bsp_gpio_chipselect(X_MOT_2);
	    break;
	    case Y_AXIS:
	    	bsp_gpio_chipselect(Y_MOT_1);
	    	bsp_gpio_chipselect(Y_MOT_2);
	    break;
	    default:
	    	// Wrong input, do nothing or call error handler
	    break;
	}
	bsp_spi_tmc2130_send(cmd, data);
	bsp_gpio_chipselect_reset();

}

void mw_tmc2130_io_init(void)
{
	// Power reset to drivers
	//bsp_gpio_bjt_off();
	bsp_gpio_led_toggle(0);
	HAL_Delay(1000);
	//bsp_gpio_bjt_on();
	mw_tmc2130_io_write_all(REG_GCONF|TMC2130_WRITE, TMC_INIT_DATA);
	mw_tmc2130_io_write_all(REG_GCONF|TMC2130_WRITE, TMC_INIT_DATA);
	mw_tmc2130_io_write_all(REG_CHOPCONF|TMC2130_WRITE, TMC_INIT_DATA);
	mw_tmc2130_io_write_all(REG_CHOPCONF|TMC2130_WRITE, TMC_INIT_DATA);
	mw_tmc2130_io_write_all(REG_COOLCONF|TMC2130_WRITE, TMC_INIT_DATA);
	mw_tmc2130_io_write_all(REG_IHOLD_IRUN|TMC2130_WRITE, TMC_INIT_DATA);
	HAL_Delay(250);
}

void mw_tmc2130_io_deinit(void)
{
	bsp_gpio_bjt_off();
	bsp_gpio_tmc2130_disable();
}

void mw_tmc2130_io_config_all(uint8_t mode)
{

	// 32bit data for storing data for registers
	uint32_t data;
	switch(mode)
	{
		case mode_0:
			//native 256 microsteps, MRES=0, TBL=1=24, TOFF=8
			data=0x00008008UL;
		break;
		case mode_1:
			//128 microsteps, MRES=0, TBL=1=24, TOFF=8
			data=0x01008008UL;
		break;
		case mode_2:
			// 64 microsteps, MRES=0, TBL=1=24, TOFF=8
			data=0x02008008UL;
		break;
		case mode_3:
			// 32 microsteps, MRES=0, TBL=1=24, TOFF=8
			data=0x03008008UL;
		break;
		case mode_4:
			// 16 microsteps, MRES=0, TBL=1=24, TOFF=8
			data=0x04008008UL;
		break;
		case mode_5:
			//  8 microsteps, MRES=0, TBL=1=24, TOFF=8
			data=0x05008008UL;
		break;
		case mode_6:
			//  4 microsteps, MRES=0, TBL=1=24, TOFF=8
			data=0x06008008UL;
		break;
		case mode_7:
			//  2 microsteps, MRES=0, TBL=1=24, TOFF=8
			data=0x07008008UL;
		break;
		case mode_8:
			//  1 microsteps, MRES=0, TBL=1=24, TOFF=8
			data=0x08008008UL;
		break;
		default:
			//native 256 microsteps, MRES=0, TBL=1=24, TOFF=8
			data = 0x00008008UL;
		break;
	}


	// Voltage on AIN is current reference
	mw_tmc2130_io_write_all(TMC2130_WRITE|REG_GCONF, 0x00000001UL);
	// IHOLD=0x10, IRUN=0x10
	//tmc_io_write(TMC2130_WRITE|REG_IHOLD_IRUN, 0x00001010UL);
	//mw_tmc2130_io_write_all(TMC2130_WRITE|REG_IHOLD_IRUN,0x00001E12UL);
	mw_tmc2130_io_write_all(TMC2130_WRITE|REG_IHOLD_IRUN,0x00001C12UL);
	//write mode of operation to chopconf register
	mw_tmc2130_io_write_all(TMC2130_WRITE|REG_CHOPCONF, data);
	//low EN for enabling of driver
	bsp_gpio_tmc2130_enable();

	HAL_Delay(100);

}



/*  ---------------------------------------------------------*/


void mw_tmc2130_io_status(void)
{
	/*
	tmc_io_write(TMC2130_READ|REG_GCONF, 0x00000000UL);
	tmc_io_write(TMC2130_READ|REG_GCONF, 0x00000000UL);
	tmc_io_write(TMC2130_READ|REG_IHOLD_IRUN, 0x00000000UL);
	tmc_io_write(TMC2130_READ|REG_IHOLD_IRUN, 0x00000000UL);
	tmc_io_write(TMC2130_READ|REG_CHOPCONF, 0x00000000UL);
	tmc_io_write(TMC2130_READ|REG_CHOPCONF, 0x00000000UL);
*/
}


void mw_tmc2130_io_step(uint8_t axis, bool dir, uint32_t usec)
{
	bsp_gpio_step_dir(axis, dir);
	bsp_gpio_step_axis_on(axis);
	bsp_tim_wait_usec(usec);
	bsp_gpio_step_axis_off(axis);
	bsp_tim_wait_usec(usec);

}

void mw_tmc2130_io_step_all(bool x, bool y, bool dir_x, bool dir_y, uint32_t usec)
{
	bsp_gpio_step_dir(X_AXIS, dir_x);
	bsp_gpio_step_dir(Y_AXIS, dir_y);
	if(x)
	{
		bsp_gpio_step_axis_on(X_AXIS);
	}
	if(y)
	{
		bsp_gpio_step_axis_on(Y_AXIS);
	}
	bsp_tim_wait_usec(usec);
	if(x)
	{
		bsp_gpio_step_axis_off(X_AXIS);
	}
	if(y)
	{
		bsp_gpio_step_axis_off(Y_AXIS);
	}
	bsp_tim_wait_usec(usec);

}
// private functions


void mw_tmc2130_io_calib(uint8_t axis, uint32_t usec)
{
	uint32_t pulses = CALIB_PULSE;
	//mw_tmc2130_io_write_all(TMC2130_WRITE|REG_CHOPCONF, 0x05008008UL);
	//bsp_gpio_tmc2130_enable();
	for(uint32_t i=0; i<pulses; i++){
		mw_tmc2130_io_step(axis, 1, usec);
	}
	pulses = mw_fun_pulses(2 * CENTER_MOV, 8);

	for(uint32_t i=0; i<pulses; i++)
		{
			mw_tmc2130_io_step(axis, 0, USEC_MOV);
		}

}




////////////////////////////////////////////////////////////////
/// Currently unused functions for specific stepper driver calibration


uint32_t set_chopconf(uint8_t toff, uint8_t hstrt, uint8_t hend, uint8_t tbl, uint8_t vsense, uint8_t sync, uint8_t mres)
{
	uint32_t data;
	data = ((uint32_t)((uint32_t)toff << TMC2130_TOFF_SHIFT) & TMC2130_TOFF_MASK)
		| ((uint32_t)((uint32_t)hstrt << TMC2130_HSTRT_SHIFT) & TMC2130_HSTRT_MASK)
		| ((uint32_t)((uint32_t)hend << TMC2130_HEND_SHIFT) & TMC2130_HEND_MASK)
		| ((uint32_t)((uint32_t)tbl << TMC2130_TBL_SHIFT) & TMC2130_TBL_MASK)
		| ((uint32_t)((uint32_t)vsense << TMC2130_VSENSE_SHIFT) & TMC2130_VSENSE_MASK)
		| ((uint32_t)((uint32_t)sync << TMC2130_SYNC_SHIFT) & TMC2130_SYNC_MASK)
		| ((uint32_t)((uint32_t)mres << TMC2130_MRES_SHIFT) & TMC2130_MRES_MASK);
	return data;
}

uint32_t set_chopconf_struct(chopconf_t chop)
{
	uint32_t data;
	data = ((uint32_t)((uint32_t)chop.toff << TMC2130_TOFF_SHIFT) & TMC2130_TOFF_MASK)
		| ((uint32_t)((uint32_t)chop.hstrt << TMC2130_HSTRT_SHIFT) & TMC2130_HSTRT_MASK)
		| ((uint32_t)((uint32_t)chop.hend << TMC2130_HEND_SHIFT) & TMC2130_HEND_MASK)
		| ((uint32_t)((uint32_t)chop.tbl << TMC2130_TBL_SHIFT) & TMC2130_TBL_MASK)
		| ((uint32_t)((uint32_t)chop.vsense << TMC2130_VSENSE_SHIFT) & TMC2130_VSENSE_MASK)
		| ((uint32_t)((uint32_t)chop.sync << TMC2130_SYNC_SHIFT) & TMC2130_SYNC_MASK)
		| ((uint32_t)((uint32_t)chop.mres << TMC2130_MRES_SHIFT) & TMC2130_MRES_MASK);
	return data;

}

uint32_t set_gconf_struct(gconf_t gconf)
{
	uint32_t data;
	data =
			   ((uint32_t)((uint32_t)gconf.i_scale_analog << TMC2130_I_SCALE_ANALOG_SHIFT) & TMC2130_I_SCALE_ANALOG_MASK)
			|  ((uint32_t)((uint32_t)gconf.internal_rsense << TMC2130_INTERNAL_RSENSE_SHIFT) & TMC2130_INTERNAL_RSENSE_MASK)
			|  ((uint32_t)((uint32_t)gconf.en_pwm_mode << TMC2130_EN_PWM_MODE_SHIFT) & TMC2130_EN_PWM_MODE_MASK)
			|  ((uint32_t)((uint32_t)gconf.enc_commutation << TMC2130_ENC_COMMUTATION_SHIFT) & TMC2130_ENC_COMMUTATION_MASK)
			|  ((uint32_t)((uint32_t)gconf.shaft << TMC2130_SHAFT_SHIFT) & TMC2130_SHAFT_MASK)
			|  ((uint32_t)((uint32_t)gconf.diag0_error << TMC2130_DIAG0_ERROR_ONLY_WITH_SD_MODE1_SHIFT) & TMC2130_DIAG0_ERROR_ONLY_WITH_SD_MODE1_MASK)
			|  ((uint32_t)((uint32_t)gconf.diag0_otpw << TMC2130_DIAG0_OTPW_ONLY_WITH_SD_MODE1_SHIFT) & TMC2130_DIAG0_OTPW_ONLY_WITH_SD_MODE1_MASK)
			|  ((uint32_t)((uint32_t)gconf.diag0_stall << TMC2130_DIAG0_STALL_SHIFT) & TMC2130_DIAG0_STALL_MASK)
			|  ((uint32_t)((uint32_t)gconf.diag1_stall << TMC2130_DIAG1_STALL_SHIFT) & TMC2130_DIAG1_STALL_MASK)
			|  ((uint32_t)((uint32_t)gconf.diag1_index << TMC2130_DIAG1_INDEX_SHIFT) & TMC2130_DIAG1_INDEX_MASK)
			|  ((uint32_t)((uint32_t)gconf.diag1_onstate << TMC2130_DIAG1_ONSTATE_SHIFT) & TMC2130_DIAG1_ONSTATE_MASK)
			|  ((uint32_t)((uint32_t)gconf.diag1_steps_skipped << TMC2130_DIAG1_STEPS_SKIPPED_SHIFT) & TMC2130_DIAG1_STEPS_SKIPPED_MASK)
			|  ((uint32_t)((uint32_t)gconf.diag0_int_pushpull << TMC2130_DIAG0_INT_PUSHPULL_SHIFT) & TMC2130_DIAG0_INT_PUSHPULL_MASK)
			|  ((uint32_t)((uint32_t)gconf.diag1_pushpull << TMC2130_DIAG1_POSCOMP_PUSHPULL_SHIFT) & TMC2130_DIAG1_POSCOMP_PUSHPULL_MASK)
			|  ((uint32_t)((uint32_t)gconf.small_hysteresis << TMC2130_SMALL_HYSTERESIS_SHIFT) & TMC2130_SMALL_HYSTERESIS_MASK)
			|  ((uint32_t)((uint32_t)gconf.stop_enable << TMC2130_STOP_ENABLE_SHIFT) & TMC2130_STOP_ENABLE_MASK)
			|  ((uint32_t)((uint32_t)gconf.direct_mode << TMC2130_DIRECT_MODE_SHIFT) & TMC2130_DIRECT_MODE_MASK)
			|  ((uint32_t)((uint32_t)gconf.test_mode << TMC2130_TEST_MODE_SHIFT) & TMC2130_TEST_MODE_MASK)
///////////////////


			;
	return data;
}

uint32_t set_coolconf_struct(coolconf_t cool)
{
	uint32_t data;
	data = ((uint32_t)((uint32_t)cool.semin << TMC2130_SEMIN_SHIFT) & TMC2130_SEMIN_MASK)
		|  ((uint32_t)((uint32_t)cool.seup << TMC2130_SEUP_SHIFT) & TMC2130_SEUP_MASK)
		|  ((uint32_t)((uint32_t)cool.semax << TMC2130_SEMAX_SHIFT) & TMC2130_SEMAX_MASK)
		|  ((uint32_t)((uint32_t)cool.sedn << TMC2130_SEDN_SHIFT) & TMC2130_SEDN_MASK)
		|  ((uint32_t)((uint32_t)cool.seimin << TMC2130_SEIMIN_SHIFT) & TMC2130_SEIMIN_MASK)
		|  ((uint32_t)((uint32_t)cool.sgt << TMC2130_SGT_SHIFT) & TMC2130_SGT_MASK)
		|  ((uint32_t)((uint32_t)cool.sfilt << TMC2130_SFILT_SHIFT) & TMC2130_SFILT_MASK);
	return data;
}

uint32_t set_iholdrun_struct(ihold_irun_t holdrun)
{
	uint32_t data;
	data = ((uint32_t)((uint32_t)holdrun.ihold << TMC2130_IHOLD_SHIFT) & TMC2130_IHOLDDELAY_MASK)
		|  ((uint32_t)((uint32_t)holdrun.irun << TMC2130_IRUN_SHIFT) & TMC2130_IRUN_MASK)
		|  ((uint32_t)((uint32_t)holdrun.iholddelay <<TMC2130_IHOLDDELAY_SHIFT) & TMC2130_IHOLDDELAY_MASK);
	return data;
}

uint32_t set_pwmconf_struct(pwmconf_t pwmconf)
{
	uint32_t data;
	data = ((uint32_t)((uint32_t)pwmconf.pwm_ampl << TMC2130_PWM_AMPL_SHIFT) & TMC2130_PWM_AMPL_MASK)
		|  ((uint32_t)((uint32_t)pwmconf.pwm_grad << TMC2130_PWM_GRAD_SHIFT) & TMC2130_PWM_GRAD_MASK)
		|  ((uint32_t)((uint32_t)pwmconf.pwm_freq << TMC2130_PWM_FREQ_SHIFT) & TMC2130_PWM_FREQ_MASK)
		|  ((uint32_t)((uint32_t)pwmconf.pwm_autoscale << TMC2130_PWM_AUTOSCALE_SHIFT) & TMC2130_PWM_AUTOSCALE_MASK)
		|  ((uint32_t)((uint32_t)pwmconf.pwm_symmetric << TMC2130_PWM_SYMMETRIC_SHIFT) & TMC2130_PWM_SYMMETRIC_MASK)
		|  ((uint32_t)((uint32_t)pwmconf.freewheel << TMC2130_FREEWHEEL_SHIFT) & TMC2130_FREEWHEEL_MASK)
		;



	return data;
}

drv_status_t get_drv_status(uint32_t data)
{
	drv_status_t get;
	get.sr = data;
	return get;
}


uint32_t get_sg_result(uint32_t data)
{
	uint32_t sg;
	sg = ((data & TMC2130_SG_RESULT_MASK) >> TMC2130_SG_RESULT_SHIFT)
			& (MASK_32_DOWN);
	return sg;
}

uint32_t get_cs_actual(uint32_t data)
{
	uint32_t cs;
	cs = ((data & TMC2130_CS_ACTUAL_MASK) >> TMC2130_CS_ACTUAL_SHIFT)
					& (MASK_32_DOWN);
	return cs;
}

spi_status_t get_spi_status(uint8_t cmd)
{
	spi_status_t status = DEFAULT;

	if ( (cmd & SPI_STATUS_STAND_MASK)>> SPI_STATUS_STAND_SHIFT)
	{
		status = SPI_STAND;
	}

	if ( (cmd & SPI_STATUS_SG2_MASK)>> SPI_STATUS_SG2_SHIFT)
	{
		status = SPI_STALL;
	}

	if( (cmd & SPI_STATUS_DRV_ERR_MASK)>> SPI_STATUS_DRV_ERR_SHIFT)
	{
		status = SPI_DR_ERR;
	}

	if( (cmd & SPI_STATUS_RESET_MASK)>> SPI_STATUS_RESET_SHIFT)
	{
		status = SPI_RESET;
	}


	return status;
}


///////////////////////////////////////////////////////////////////////////////
// Functions for ramping, but unused and unnecessary
void accelerate(uint32_t start, uint32_t end)
{
	for(uint32_t i = start; i > end; (i=i-20))
	{
		//tmc_io_step(i);
	}
}

void decelerate(uint32_t start, uint32_t end)
{
	for(uint32_t i = start; i < end; (i=i+20))
	{
		//tmc_io_step(i);
	}
}

void const_speed(uint32_t step, uint32_t repeat)
{
	for(uint32_t i = repeat; i>0;i--)
	{
		//tmc_io_step(step);
	}
}

void ramp_stepper(uint32_t max_speed, uint32_t repeat)
{
	HAL_GPIO_WritePin(EN_GPIO_PORT, EN_PIN, GPIO_PIN_RESET);
	accelerate(200, max_speed);
	const_speed(max_speed, 10);
	decelerate(max_speed, 200);
	HAL_GPIO_WritePin(EN_GPIO_PORT, EN_PIN, GPIO_PIN_SET);
}
/* CODE END*/
