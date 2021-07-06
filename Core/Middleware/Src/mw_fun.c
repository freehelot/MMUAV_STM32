/*
 * mw_fun.c
 *
 *  Created on: Jun 29, 2021
 *      Author: hrvoje
 */
#include <mw_fun.h>

#define TEETH_COUNT		(25U)
#define MIN_ANGLE		(18U) // 1.8deg
#define TEETH_ANGLE		(144)	// 14.4deg
#define MICRO_STEP		(4U)
#define MICRO_STEP_HALF (8U)
#define HALF_RANGE  	(81U)
#define MAX_RANGE		(80U)
#define MODULE			(125U)	// 1.25mm
#define PI				(314U)  // 3.14
#define FULL_ANGLE		(360U)  // 360 deg
#define SCALING_POINT	(100000U) // Scale factor for removing float points
#define PRECISION		(1U)	//1 -> min movement precision, 0 -> per teeth movement precision
#define SCALING_FULL 	(2U)	// scaling for full precision, 2*1.8 deg = 1mm lin movement
#define TEETH_PRECISION (8U)
uint32_t mw_fun_pulses(uint32_t movement, uint32_t ms)
{
	// we receive movement in mm * 10
	// MIN -81, MAX 81
	// CALCULATIONS
	// lin movement = module * teeth number * PI * angle_rotated / 360deg
	// degs between teeth -> 14.4 deg -> 8 steps of 1.8deg (360/25 = 14.4)
	// 8 microstep -> 1.8deg -> 0.225deg per step
	//    ->  teeth_number * 8  = 45deg
	// 4 microstep -> 1.8deg -> 0.45deg per step
	//    ->  teeth_number * 4  = 45deg
	// Per these calculations -> teeth to teeth movement -> 3.925mm
	// minimum movement of 1.8deg rotation -> 0.490625mm

	uint32_t pulses = 0;
	uint32_t mov = 0;
	// pulse * 0.490625mm
	mov = movement;
	// pulse * 0.490625mm
	if(PRECISION != 1)
	{
		// pulse * 0.490625mm
		mov =  movement / 4;
		mov = mov * TEETH_PRECISION;
	}

	pulses = (mov * FULL_ANGLE * SCALING_POINT) / (MODULE * PI * TEETH_COUNT * MIN_ANGLE);
	pulses = pulses * ms;
	//pulses = (movement * 10) - ((movement * 10) % 15);
	//pulses = (movement * 10) - (movement * 10) % 3;
	//pulses = (  pulses *2 * 2*642) / 3120;
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
