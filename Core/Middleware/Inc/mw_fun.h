/*
 * mw_fun.h
 *
 *  Created on: Jun 29, 2021
 *      Author: hrvoje
 */

#ifndef MIDDLEWARE_INC_MW_FUN_H_
#define MIDDLEWARE_INC_MW_FUN_H_

#include <stdint.h>
#include <stdbool.h>

#define STEP_ANGLE                (18U)
#define STEP_ANGLE_DIV            (10U)
#define PITCH_DIAM                (312U)
#define PITCH_DIV                 (10U)
#define FULL_ANGLE                (360U)

/**
 *
 */
uint32_t mw_fun_pulses(uint32_t movement, uint32_t ms);

/**
 *
 */
uint32_t mw_fun_pos(uint32_t mov_c, bool *dir, uint32_t *pos_c);

/**
 * @brief Functions for doing a ramp-up, ramp-down
 *
 * @param steps Number of steps needed to complete the ramp
 * @return true if completed, false if still moving
 */
bool mw_fun_ramp(uin32_t steps);

#endif /* MIDDLEWARE_INC_MW_FUN_H_ */
