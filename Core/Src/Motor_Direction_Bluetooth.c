/*
 * Motor_Direction_Bluetooth.c
 *
 *  Created on: Mar 27, 2024
 *      Author: sherief alglaly
 */
#include "../Inc/Motor_Direction_Bluetooth.h"
extern Motor_t MotorForwardSuspension,MotorBackwardSuspension,MotorRight,MotorLeft ;
int Global_s32ForwardPosition=0;
int Global_s32BackwardPosition=0;
int k;

int32_t counter_j = 0;
int32_t counter_I = 0;


#define  mod  1
void Bluetooth_controlDirection(unsigned char rxdata)
{
	k++;

	  if (rxdata == 'B')
	  {
		  MotorRight.Direction_suspension = up_move;
		  MotorLeft.Direction_suspension = up_move;
	  }
	  else if (rxdata == 'F')
	  {
		  MotorRight.Direction_suspension = down_mov;
		  MotorLeft.Direction_suspension = down_mov;
	  }
	  else if (rxdata == 'L')
	  {
		  MotorRight.Direction_suspension = up_move;
		  MotorLeft.Direction_suspension = down_mov;
	  }
	  else if (rxdata == 'R')
	  {
		  MotorRight.Direction_suspension = down_mov;
		  MotorLeft.Direction_suspension = up_move;
	  }

	  else if (rxdata == 'J')
	  {
		  counter_j +=20;
		  MotorForwardSuspension.Target_position= counter_j;
	  }
	  else if (rxdata == 'M')
	  {
		  counter_j-=20;
		  MotorForwardSuspension.Target_position=  counter_j;

	  }
	  else if (rxdata=='I')
	  {
		  counter_I+=20;
		  MotorBackwardSuspension.Target_position=  counter_I;
	  }
	  else if (rxdata=='K')
	  {
		  counter_I-=20;
		  MotorBackwardSuspension.Target_position= counter_I;
	  }
	  else
	  {
		  MotorRight.Direction_suspension = stop;
		  MotorLeft.Direction_suspension  = stop;
	  }
	  /* Adjusting the two following 4 lines with Speed control Function*/
	  __HAL_TIM_SET_COMPARE(MotorRight.PWM_Timer, MotorRight.PWM_channel, 75);
	  __HAL_TIM_SET_COMPARE(MotorLeft.PWM_Timer, MotorLeft.PWM_channel, 75);
	  HAL_TIM_PWM_Start(MotorRight.PWM_Timer, MotorRight.PWM_channel);
	  HAL_TIM_PWM_Start(MotorLeft.PWM_Timer, MotorLeft.PWM_channel);

	  DriveMotorDirection(&MotorRight);
	  DriveMotorDirection(&MotorLeft);
}
