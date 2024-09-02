/*
 * Position_Control.h
 *
 *  Created on: Feb 4, 2024
 *      Author: sherief alglaly
 */

#ifndef INC_POSITION_CONTROL_H_
#define INC_POSITION_CONTROL_H_

#include "stm32f4xx_hal.h"
typedef enum
{
	Backward_move,
	Forward_move
}Direction_MainMotor;

typedef enum
{
	up_move,
	down_mov,
	stop
}Dir_suspensionMoto;

typedef enum
{
	Right_motor,
	Left_motor,
	forwardSus_motor,
	BackwardSus_motor
}motor_no;

typedef struct
{
	 motor_no Motor_no;
     uint8_t max_dutyCycle;
	//Pins
 	uint16_t IC_channel;
 	TIM_HandleTypeDef * IC_Timer;
//	uint16_t Encoder_pin;
//	GPIO_TypeDef * Encoder_port;
	uint16_t Encoder2_pin;
	GPIO_TypeDef * Encoder2_port;

	uint16_t Encoder1_pin;
	GPIO_TypeDef * Encoder1_port;

	uint16_t Direction_pin1;
	GPIO_TypeDef * Direction_port1;

	uint16_t Direction_pin2;
	GPIO_TypeDef * Direction_port2;
	uint16_t PWM_channel;
	TIM_HandleTypeDef * PWM_Timer;

	uint32_t IntervalTime;



	uint16_t Motor_Enable;



	//Direction
	 Direction_MainMotor Direction_main;

	// Suspension motors
	 Dir_suspensionMoto Direction_suspension;

	/******************position*************************/
	int32_t CurrentPosition;
	int32_t Target_position;
    int32_t PrevPosition;
	//howskeeping values
	float prev_err;
	int32_t err;
	float prev_time;
	float errIntegral;
	float errdot;
	float deltaerror;
	//pid
	float proportional;
	float integral;
	float differential;
	//control
	float ControlSignal;
    int32_t Ref;

}Motor_t ;

void init_motors(Motor_t* array_of_motors);
/*************position ******************/
void Position_GetNewTarget(Motor_t* motor,int32_t Targt,uint8_t);
void Position_Calc_Signal(Motor_t* );
void UpdateEncoder(Motor_t* ,Dir_suspensionMoto dir  );
void DriveMotor(Motor_t* );
void DriveMotorDirection(Motor_t* );
void DriveMotorDirectionSPeed(Motor_t* motor );
void init_PIDVal(Motor_t* motor,float P,float I,float D);
void Print_IntervalTime(Motor_t* motor);
void Print_Position(Motor_t* motor);


void SpeedFB(Motor_t* right,Motor_t* left ,uint8_t speed,Direction_MainMotor dir) ;

// function to speed control motor
 // forward or back ward fuction gets struct to motor, desired speed, direction
   /*- gets error "time interval difference" relative to the slower, then control this error to make the higher speed motor get less speed
    * the function to get the difference in ICU_Functions" compare which one is the higher speed then get the difference.
    */
 // right and left function gets struct to motor, desired speed, direction

/*the last two functions call pwm functions to drive motors  */
//function to drive motor directions speeds !

#endif /* INC_POSITION_CONTROL_H_ */

/*Motor_t MotorRight={
  		.Motor_no =Right_motor,

  		.Encoder2_pin=mr_enc2_Pin,
  		.Encoder2_port=mr_enc2_GPIO_Port,


		//tim14 ch1

  		.Direction_pin1=mr_dir1_Pin,
  		.Direction_port1=mr_dir1_GPIO_Port,

  		.Direction_pin2=mr_dir2_Pin,
  		.Direction_port2=mr_dir2_GPIO_Port,

  		.PWM_channel=TIM_CHANNEL_1,
  		.PWM_Timer=&htim1

  };*/
