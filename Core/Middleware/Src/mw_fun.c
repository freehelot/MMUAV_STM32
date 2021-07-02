/*
 * mw_fun.c
 *
 *  Created on: Jun 29, 2021
 *      Author: hrvoje
 */
#include <mw_fun.h>

#define HALF_RANGE  (81U)

uint32_t mw_fun_pulses(uint32_t movement)
{
	//pulses = (FULL_ANGLE * mov_x * PITCH_DIV * STEP_ANGLE_DIV) / (STEP_ANGLE * PITCH_DIAM);
	uint32_t pulses = 0;
	//pulses = (movement * 10) - ((movement * 10) % 15);
	pulses = (movement * 10) - (movement * 10) % 3;
	pulses = (  pulses *2 * 2*642) / 3120;
	/*
	uint32_t check_pulses = (movement *2 * 2*642) / 312;
	if (pulses < check_pulses)
	{
		pulses = check_pulses;
	}
	*/
	return pulses;
}

uint32_t mw_fun_pos(uint32_t mov_c, bool *dir, uint32_t *pos_c)
{
	bool dir_des;
	dir_des = *dir;
	uint32_t pos_des = 0;
	uint32_t mov_des = 0;
	uint32_t pos_current = 0;
	pos_current = *pos_c;
	// saturation
	if(mov_c > 78)
	{
		mov_des = 78;
	}
	else
	{
		mov_des = mov_c;
	}

	// DIR decides direction, plus or minus
	if(dir_des)
	{
		pos_des = HALF_RANGE + mov_des;
	}
	else
	{
		pos_des = HALF_RANGE - mov_des;
	}

	// Check the value of current position with desired to
	// calculate desired movement
	if(pos_des >= pos_current)
	{
		mov_des = pos_des - pos_current;
		*dir = true;
	}
	else
	{
		mov_des = pos_current - pos_des;
		*dir = false;
	}
	*pos_c = pos_des;

	return mov_des;
}
