/*
 * ICU_Functions.c
 *
 *  Created on: Mar 15, 2024
 *      Author: sherief alglaly
 */
#include "ICU_Functions.h"
#include "stm32f4xx_hal.h"
#include "Position_Control.h"


/********** Use Just two functions instead of four functions !!! S***********/


void MotorFS_ICU(Motor_t* Motor)
{
	 //HAL_TIM_IC_Stop_IT(Motor->IC_Timer,Motor->IC_channel);

	static uint8_t first = 0;
	uint8_t bitstatus1 ;
	bitstatus1 = HAL_GPIO_ReadPin(Motor->Encoder2_port, Motor->Encoder2_pin);


	if (bitstatus1 == GPIO_PIN_RESET)
	{
		UpdateEncoder(Motor,up_move);
	}
	else
	{
		UpdateEncoder(Motor,down_mov);
	}
}

void MotorBS_ICU(Motor_t* Motor)
{
    uint8_t bitstatus1 ;
	bitstatus1 = HAL_GPIO_ReadPin(Motor->Encoder2_port, Motor->Encoder2_pin);




	if (bitstatus1 == GPIO_PIN_RESET)
	{

		UpdateEncoder(Motor,up_move);
	}
	else
	{
		UpdateEncoder(Motor,down_mov);

	}
}

void MotorR_ICU(Motor_t* Motor)
{
    uint8_t bitstatus1 ;

    bitstatus1 = HAL_GPIO_ReadPin(Motor->Encoder2_port, Motor->Encoder2_pin);
    /*Get time between two ticks -- to determine the speed */
    /*  get interval time (hal_timer read capture)
     *  set counter to zero
     *  start ic it
     */
//    Motor->IntervalTime = HAL_TIM_ReadCapturedValue(Motor->IC_Timer, Motor->IC_channel);
//    __HAL_TIM_SET_COUNTER(Motor->IC_Timer,0);
//    HAL_TIM_IC_Start_IT(Motor->IC_Timer, Motor->IC_channel);

	if (bitstatus1 == GPIO_PIN_RESET)
	{
		UpdateEncoder(Motor,up_move);
	}
	else
	{
		UpdateEncoder(Motor,down_mov);
	}
}

void MotorL_ICU(Motor_t* Motor)
{
    uint8_t bitstatus1 ;
	bitstatus1 = HAL_GPIO_ReadPin(Motor->Encoder2_port, Motor->Encoder2_pin);
	/*Get time between two ticks -- to determine the speed */
//    Motor->IntervalTime = HAL_TIM_ReadCapturedValue(Motor->IC_Timer, Motor->IC_channel);
//    __HAL_TIM_SET_COUNTER(Motor->IC_Timer,0);
//    HAL_TIM_IC_Start_IT(Motor->IC_Timer, Motor->IC_channel);

	if (bitstatus1 == GPIO_PIN_RESET)
	{
		UpdateEncoder(Motor,up_move);
	}
	else
	{
		UpdateEncoder(Motor,down_mov);
	}
}


