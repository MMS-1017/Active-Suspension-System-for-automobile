/*
 * Motor_Direction_Bluetooth.h
 *
 *  Created on: Mar 27, 2024
 *      Author: sherief alglaly
 */

#include "stm32f4xx_hal.h"
#include "../Inc/Position_Control.h"

#ifndef INC_MOTOR_DIRECTION_BLUETOOTH_H_
#define INC_MOTOR_DIRECTION_BLUETOOTH_H_

void Bluetooth_controlDirection(unsigned char rxdata);

#endif /* INC_MOTOR_DIRECTION_BLUETOOTH_H_ */
