/*
 * ICU_Functions.h
 *
 *  Created on: Mar 15, 2024
 *      Author: sherief alglaly
 */
#include "Position_Control.h"

#ifndef INC_ICU_FUNCTIONS_H_
#define INC_ICU_FUNCTIONS_H_

 // read pulses of motors
void MotorFS_ICU(Motor_t* Motor);
void MotorBS_ICU(Motor_t* Motor);
void MotorR_ICU(Motor_t* Motor);
void MotorL_ICU(Motor_t* Motor);

#endif /* INC_ICU_FUNCTIONS_H_ */
