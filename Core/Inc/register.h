/*
 * register.h
 *
 *  Created on: Apr 5, 2021
 *      Author: hrvoje
 */

#ifndef INC_REGISTER_H_
#define INC_REGISTER_H_

/* USER CODE BEGIN PV */
#define TMC2130_READ (0x00)
#define TMC2130_WRITE (0x80)

// Register memory positions
#define REG_GCONF (0x00)
#define REG_IHOLD_IRUN (0x10)
#define REG_CHOPCONF (0x6C)
#define REG_COOLCONF (0x6D)

#endif /* INC_REGISTER_H_ */
