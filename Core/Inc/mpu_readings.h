//
///**
//  ******************************************************************************
//  * @file           : mpu_readings.h
//  * @brief          : this file contains all the definitions and FP
//  * @author 		: Mahmoud Sayed
//  ******************************************************************************
//**/
//
#ifndef MPU_READINGS_H_
#define MPU_READINGS_H_
//
///************************ all includes *****************/
//#include <stdio.h>
//#include <string.h>
//#include <stdlib.h>
//#include <math.h>
//#include <stdbool.h>
//#include "stm32f4xx_hal.h"
//#include "Position_Control.h"
///********************* MPU6050 registers ******************/
//#define MPU6050_ADDR 	0x68<<1
//#define PWR_MGMT_1_REG 	0x6B
//#define SMPLRT_DIV_REG 	0x19
//#define GYRO_CNFG_REG 	0x1B
//#define ACC_CNFG_REG 	0x1C
//
///********************* statu errs ******************/
//typedef enum
//{    no_err,
//	init_err,
//	loop_err,
//	run_err
//}mpu_errs;
//
///**************** MPU typedef *************/
//typedef struct {
//	uint8_t data;
//	uint8_t buffer[2],tuffer[6],cuffer[6];
//	int16_t gyro_raw[3],acc_raw[3];
//	float gyro_cal[3];
//	int16_t acc_total_vector;
//	float angle_pitch_gyro,angle_roll_gyro;
//	float angle_pitch_acc,angle_roll_acc;
//	float angle_pitch,angle_roll;
//	int16_t raw_temp;
//	float temp;
//	int i;
//	float prevtime,prevtime1,time1,elapsedtime1,prevtime2,time2,elapsedtime2;
//	I2C_HandleTypeDef* I2C_handle;
//	HAL_StatusTypeDef set_gyro;
//	mpu_errs status  ;
//}MPU_vars_t;
//
//
//
//
///************************ MPU FP ************************/
//mpu_errs MPU_init(MPU_vars_t*);
//mpu_errs MPU_run(MPU_vars_t*);
//void MPUControlMotor(Motor_t* motor, float Angle);
//
//#endif
//////
///////*
//////*************************
//////*                       *
//////* İbrahim Cahit Özdemir *
//////*                       *
//////*     October 2021      *
//////*                       *
//////*************************
//////*/
//////
//#ifndef INC_GY521_H_
//#define INC_GY521_H_
//
//#endif /* INC_GY521_H_ */
//
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "Position_Control.h"

#define RAD_TO_DEG 57.295779513082320876798154814105
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43

// Setup MPU6050
#define MPU6050_ADDR 0xd0

// MPU6050 structure
typedef struct {

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;

    double  LastAngle_X ;
    double  LastAngle_Y ;

    double Diff_X;
    double Diff_Y;
} MPU6050_t;


// Kalman structure
typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;


uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

void MPUControlMotor(Motor_t* motor, float Angle);

void control_mpu_incremental(Motor_t* motor, float Angle);

#endif /* INC_GY521_H_ */
