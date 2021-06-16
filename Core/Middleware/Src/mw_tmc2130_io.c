/*
 * mw_tmc_io.c
 *
 *  Created on: 6 Apr 2021
 *      Author: hrvoje
 */
/* Includes ------------------------------------------------------------------*/
#include <mw_tmc2130_io.h>
/* USER CODE BEGIN 0 */
#include <bits.h>
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TMC_INIT_DATA (0x00000000)
#define PWM_INIT_DATA (0x00050480)
/* USER CODE END PD */
//#define INIT_REGISTER(REG) REG##_t REG##_register = REG##_t


/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
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

#define MAX_STEP 2000
#define MIN_STEP 100
/* USER CODE END PTD */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
SPI_HandleTypeDef hspi1;
/* USER CODE END 0 */

/* USER CODE BEGIN 1 */


void tmc_io_write( uint8_t cmd,uint32_t data)
{

		// Variable, 5x8bits=40bits for communication via SPI
		uint8_t spi[5]={0};
		//CS pin low before sending
		HAL_GPIO_WritePin(CS_1_GPIO_Port,CS_1_Pin,GPIO_PIN_RESET);
		// First byte is command (write/read + register)
		spi[0]=cmd;
		// Other bytes set
		spi[1]=((data>>24)&0xFF);
		spi[2]=((data>>16)&0xFF);
		spi[3]=((data>>8)&0xFF);
		spi[4]=((data>>0)&0xFF);
		//Transmit via SPI
		HAL_SPI_Transmit(&hspi1,spi,5,50);
		//CS pin high after sending
		HAL_GPIO_WritePin(CS_1_GPIO_Port,CS_1_Pin,GPIO_PIN_SET);



}

void tmc_io_init(void)
{
	HAL_Delay(200);
	tmc_io_write(REG_GCONF|TMC2130_WRITE, TMC_INIT_DATA);
	tmc_io_write(REG_GCONF|TMC2130_WRITE, TMC_INIT_DATA);
	tmc_io_write(REG_CHOPCONF|TMC2130_WRITE, TMC_INIT_DATA);
	tmc_io_write(REG_CHOPCONF|TMC2130_WRITE, TMC_INIT_DATA);
	tmc_io_write(REG_COOLCONF|TMC2130_WRITE, TMC_INIT_DATA);
	tmc_io_write(REG_IHOLD_IRUN|TMC2130_WRITE, TMC_INIT_DATA);

}

void tmc_io_config(uint8_t mode)
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
			data=0x02008008UL;
		break;
		case mode_2:
			// 64 microsteps, MRES=0, TBL=1=24, TOFF=8
			data=0x03008008UL;
		break;
		case mode_3:
			// 32 microsteps, MRES=0, TBL=1=24, TOFF=8
			data=0x04008008UL;
		break;
		case mode_4:
			// 16 microsteps, MRES=0, TBL=1=24, TOFF=8
			data=0x05008008UL;
		break;
		case mode_5:
			//  8 microsteps, MRES=0, TBL=1=24, TOFF=8
			data=0x06008008UL;
		break;
		case mode_6:
			//  4 microsteps, MRES=0, TBL=1=24, TOFF=8
			data=0x07008008UL;
		break;
		case mode_7:
			//  2 microsteps, MRES=0, TBL=1=24, TOFF=8
			data=0x08008008UL;
		break;
		case mode_8:
			//  1 microsteps, MRES=0, TBL=1=24, TOFF=8
			data=0x00008008UL;
		break;
		default:
			//native 256 microsteps, MRES=0, TBL=1=24, TOFF=8
			data = 0x00008008UL;
		break;
	}
	// Voltage on AIN is current reference
	tmc_io_write(TMC2130_WRITE|REG_GCONF, 0x00000001UL);
	// IHOLD=0x10, IRUN=0x10
	//tmc_io_write(TMC2130_WRITE|REG_IHOLD_IRUN, 0x00001010UL);
	tmc_io_write(TMC2130_WRITE|REG_IHOLD_IRUN,0x00001C12UL);
	//write mode of operation to chopconf register
	tmc_io_write(TMC2130_WRITE|REG_CHOPCONF, data);
	//low EN for enabling of driver
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
}

void tmc_io_status(void)
{
	tmc_io_write(TMC2130_READ|REG_GCONF, 0x00000000UL);
	tmc_io_write(TMC2130_READ|REG_GCONF, 0x00000000UL);
	tmc_io_write(TMC2130_READ|REG_IHOLD_IRUN, 0x00000000UL);
	tmc_io_write(TMC2130_READ|REG_IHOLD_IRUN, 0x00000000UL);
	tmc_io_write(TMC2130_READ|REG_CHOPCONF, 0x00000000UL);
	tmc_io_write(TMC2130_READ|REG_CHOPCONF, 0x00000000UL);

}

void tmc_io_gpio(void)
{
	  HAL_GPIO_WritePin(CS_1_GPIO_Port,CS_1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET); // disable driver on startup, pin_set for disable
}


void tmc_io_step_millis(uint32_t millis)
{
	  HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin,GPIO_PIN_SET);
	  HAL_Delay(millis);
	  HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin,GPIO_PIN_RESET);
	  HAL_Delay(millis);
}


void tmc_io_step(uint32_t usec)
{
	  HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin,GPIO_PIN_SET);
      timer2_wait_usec(usec);
	  HAL_GPIO_WritePin(STEP_GPIO_Port,STEP_Pin,GPIO_PIN_RESET);
      timer2_wait_usec(usec);
}

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

void accelerate(uint32_t start, uint32_t end)
{
	for(uint32_t i = start; i > end; (i=i-20))
	{
		tmc_io_step(i);
	}
}

void decelerate(uint32_t start, uint32_t end)
{
	for(uint32_t i = start; i < end; (i=i+20))
	{
		tmc_io_step(i);
	}
}

void const_speed(uint32_t step, uint32_t repeat)
{
	for(uint32_t i = repeat; i>0;i--)
	{
		tmc_io_step(step);
	}
}

void ramp_stepper(uint32_t max_speed, uint32_t repeat)
{
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
	accelerate(200, max_speed);
	const_speed(max_speed, 10);
	decelerate(max_speed, 200);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
}
/* USER CODE END 1 */

