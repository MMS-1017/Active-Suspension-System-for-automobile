///**
//  ******************************************************************************
//  * @file           : mpu_readings.h
//  * @brief          : C file contains all the implementations and FP
//  * @author 		: Mahmoud Sayed
//  ******************************************************************************
//**/
//
//#include "mpu_readings.h"
//#include "Position_Control.h"
//#include "ParametersOfVehicle.h"
//
//mpu_errs  MPU_init(MPU_vars_t* mpu1)
//{
//	HAL_StatusTypeDef status = HAL_OK ;
//	mpu_errs err_state =no_err;
//
//	   mpu1->data = 0x00;
//	   status = HAL_I2C_Mem_Write(mpu1->I2C_handle, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &(mpu1->data), 1, HAL_MAX_DELAY);
//	   mpu1->data = 0x08;
//	   status = HAL_I2C_Mem_Write(mpu1->I2C_handle, MPU6050_ADDR, GYRO_CNFG_REG, 1, &(mpu1->data), 1, HAL_MAX_DELAY);
//	   mpu1->data = 0x10;
//	   status = HAL_I2C_Mem_Write(mpu1->I2C_handle, MPU6050_ADDR, ACC_CNFG_REG, 1, &(mpu1->data), 1, HAL_MAX_DELAY);
//
//	   if (status != HAL_OK)
//		   {
//		   err_state = init_err;
//		   return err_state ;
//		   }
//
//	   for(mpu1->i = 0; mpu1->i<2000; mpu1->i++)
//	   {
//	 	  mpu1->prevtime2 = mpu1->time2;
//	 	  mpu1->time2 = HAL_GetTick();
//	 	  mpu1->elapsedtime2 = (mpu1->time2 - mpu1->prevtime2)*1000;
//
//	 	  mpu1->cuffer[0] = 0x43;
//	 	  status = HAL_I2C_Master_Transmit(mpu1->I2C_handle, MPU6050_ADDR, mpu1->cuffer, 1, HAL_MAX_DELAY);
//	 	  status = HAL_I2C_Master_Receive(mpu1->I2C_handle, MPU6050_ADDR, mpu1->cuffer, 6, HAL_MAX_DELAY);
//
//	 	  mpu1->gyro_raw[0] = (mpu1->cuffer[0] << 8 | mpu1->cuffer[1]);
//	 	  mpu1->gyro_raw[1] = (mpu1->cuffer[2] << 8 | mpu1->cuffer[3]);
//	 	  mpu1->gyro_raw[2] = (mpu1->cuffer[4] << 8 | mpu1->cuffer[5]);
//
//	 	  mpu1->gyro_cal[0] += mpu1->gyro_raw[0];
//	 	  mpu1->gyro_cal[1] += mpu1->gyro_raw[1];
//	 	  mpu1->gyro_cal[2] += mpu1->gyro_raw[2];
//
//	 	  HAL_Delay(3);
//	 	  if (status != HAL_OK) break;
//	   }
//
//	   mpu1->gyro_cal[0] /= 2000;
//	   mpu1->gyro_cal[1] /= 2000;
//	   mpu1->gyro_cal[2] /= 2000;
//
//	   if (status != HAL_OK)
//	   {
//		   err_state = loop_err;
//		   return err_state ;
//	   }
//	   return err_state ;
//
//	//   HAL_Delay(1000);
//}
//
//mpu_errs MPU_run(MPU_vars_t* mpu1)
//{
//			HAL_StatusTypeDef status = HAL_OK ;
//			mpu_errs err_state =no_err;
//
//			mpu1->prevtime1 = mpu1->time1;
//			mpu1->time1 = HAL_GetTick();
//			mpu1->elapsedtime1 = (mpu1->time1 - mpu1->prevtime1)*1000;
//
//			mpu1->tuffer[0] = 0x3B;
//			status = HAL_I2C_Master_Transmit(mpu1->I2C_handle, MPU6050_ADDR, mpu1->tuffer, 1, HAL_MAX_DELAY);
//			status = HAL_I2C_Master_Receive(mpu1->I2C_handle, MPU6050_ADDR, mpu1->tuffer, 6, HAL_MAX_DELAY);
//			// check errors
//			if (status != HAL_OK)
//			{
//				err_state = run_err;
//				return err_state ;
//			}
//
//
//			 // ACC RAW VALUES
//			 mpu1->acc_raw[0] = (mpu1->tuffer[0] << 8 | mpu1->tuffer[1]);
//			 mpu1->acc_raw[1] = (mpu1->tuffer[2] << 8 | mpu1->tuffer[3]);
//			 mpu1->acc_raw[2] = (mpu1->tuffer[4] << 8 | mpu1->tuffer[5]);
//
//			 mpu1->buffer[0] = 0x41;
//			 HAL_I2C_Master_Transmit(mpu1->I2C_handle, MPU6050_ADDR, mpu1->buffer, 1, HAL_MAX_DELAY);
//			 HAL_I2C_Master_Receive(mpu1->I2C_handle, MPU6050_ADDR, mpu1->buffer, 2, HAL_MAX_DELAY);
//
//
//			 //temp values
//			 mpu1->raw_temp = (mpu1->buffer[0] << 8 | mpu1->buffer[1]);
//			 mpu1->temp = (mpu1->raw_temp / 340.0) + 36.53;
//
//
//			 mpu1->cuffer[0] = 0x43;
//			 HAL_I2C_Master_Transmit(mpu1->I2C_handle, MPU6050_ADDR, mpu1->cuffer, 1, HAL_MAX_DELAY);
//			 HAL_I2C_Master_Receive(mpu1->I2C_handle, MPU6050_ADDR, mpu1->cuffer, 6, HAL_MAX_DELAY);
//
//			 // GYRO RAW VALUES
//			 mpu1->gyro_raw[0] = (mpu1->cuffer[0] << 8 | mpu1->cuffer[1]);
//			 mpu1->gyro_raw[1] = (mpu1->cuffer[2] << 8 | mpu1->cuffer[3]);
//			 mpu1->gyro_raw[2] = (mpu1->cuffer[4] << 8 | mpu1->cuffer[5]);
//
//			 mpu1->gyro_raw[0] -= mpu1->gyro_cal[0];
//			 mpu1->gyro_raw[1] -= mpu1->gyro_cal[1];
//			 mpu1->gyro_raw[2] -= mpu1->gyro_cal[2];
//
//			 mpu1->angle_pitch_gyro += mpu1->gyro_raw[0] * 0.0000611;
//			 mpu1->angle_roll_gyro += mpu1->gyro_raw[1] * 0.0000611;
//
//			 mpu1->angle_pitch_gyro += mpu1->angle_roll_gyro * sin(mpu1->gyro_raw[2] * 0.000001066);
//			 mpu1->angle_roll_gyro -= mpu1->angle_pitch_gyro * sin(mpu1->gyro_raw[2] * 0.000001066);
//
//			 mpu1->acc_total_vector = sqrt((mpu1->acc_raw[0]*mpu1->acc_raw[0])+(mpu1->acc_raw[1]*mpu1->acc_raw[1])+(mpu1->acc_raw[2]*mpu1->acc_raw[2]));
//
//			 //57.296 = 1 / (3.412/180)
//			 mpu1->angle_pitch_acc = asin((float)mpu1->acc_raw[1]/mpu1->acc_total_vector)* 57.296;
//			 mpu1->angle_roll_acc = asin((float)mpu1->acc_raw[0]/mpu1->acc_total_vector)* -57.296;
//
//			 mpu1->angle_pitch_acc -= 0.00;
//			 mpu1->angle_roll_acc -= 0.00;
//
//			 //adding low pass filter & high pass filter
//			 if(mpu1->set_gyro){
//				 mpu1->angle_pitch = mpu1->angle_pitch_gyro * 0.9996 + mpu1->angle_pitch_acc * 0.0004;
//				 mpu1->angle_roll  = mpu1->angle_roll_gyro * 0.9996 + mpu1->angle_roll_acc * 0.0004;
//			 }
//			 else{
//				 mpu1->angle_pitch = mpu1->angle_pitch_acc;
//				 mpu1->set_gyro = true;
//			 }
//
//			 while((HAL_GetTick() - mpu1->prevtime)*1000 < 4000);
//			 mpu1->prevtime = HAL_GetTick();
//
//			 return err_state ;
//}
//
//void MPUControlMotor(Motor_t* motor, float Angle)
//{
//	        uint16_t Ticks = 0;
//	        float Length_bump = 0;
//			Length_bump = sin(Angle* (float)M_PI/(float)180) * LengthHalfOfCar ;
//		    /*   0  <---- 0   ticks */
//			/*   42 <---- 385 ticks*/
//			Ticks = Length_bump * 9.2;
//			if(Ticks <= 385){
//				Position_GetNewTarget(motor,Ticks );
//			}
//
//}
//////
///////*
//////*************************
//////*                       *
//////*     Abulseed's House  *
//////*                       *
//////*     june 2024         *
//////*     8 Dhu al-Hijja    *
//////*************************
//////*/
//////
#include <math.h>
#include "mpu_readings.h"
#include "Position_Control.h"
#include "ParametersOfVehicle.h"



const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0;
extern  uint8_t Rec_Data[14];
uint32_t timer;

Kalman_t KalmanX = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f
};

Kalman_t KalmanY = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f,
};

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx) {
    uint8_t check;
    uint8_t Data;

    // check device ID WHO_AM_I



    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

    if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);
        return 0;
    }
    return 1;
}


void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into acceleration in 'g'
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 16384.0
         for more details check ACCEL_CONFIG Register              ****/

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
}


void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into dps (�/s)
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 131.0
         for more details check GYRO_CONFIG Register              ****/

    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
}

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[2];
    int16_t temp;

    // Read 2 BYTES of data starting from TEMP_OUT_H_REG register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, i2c_timeout);

    temp = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
}

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {

    int16_t temp;

    // Read 14 BYTES of data starting from ACCEL_XOUT_H register
//    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, i2c_timeout);
//    HAL_I2C_Mem_Read_DMA(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14);

    DataStruct->Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t) (Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
    DataStruct->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;

    // Kalman angle solve
    double dt = (double) (HAL_GetTick() - timer) / 1000;
    timer = HAL_GetTick();
    double roll;
    double roll_sqrt = sqrt(
            DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0) {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    } else {
        roll = 0.0;
    }
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90)) {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    } else {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gy, dt);

}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt) {
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};


void MPUControlMotor(Motor_t* motor, float Angle)
{


    int16_t Ticks = 0;
//    static int16_t Last_Ticks=0;
    float Length_bump = 0;



	        Length_bump = sin(Angle * 3.14 / 180) * 160 ;
	        /*   0  <---- 0   ticks */
	        /*   42 <---- 190 ticks*/
	        Ticks = (Length_bump * 3.9 );
//	        Last_Ticks = Ticks;

//	        Ticks /=10;
//	        Ticks *=10;


	        if (motor->Motor_no == forwardSus_motor)
	        {
	           if (Ticks < 150  && Ticks > -350)
	           {
	        		Position_GetNewTarget(motor,Ticks,80 );
	        		//HAL_Delay(10);
	           }
	           else if (Ticks >150 )
	           {
	        	   Position_GetNewTarget(motor,150,80 );
	           }
	           else if (Ticks < -350 )
	           {
	        	   Position_GetNewTarget(motor,-350,80 );
	           }
	        }



	        else if (motor->Motor_no == BackwardSus_motor)
	        {
	           if (Ticks >= -100  && Ticks < 150)
	           {
	        		Position_GetNewTarget(motor,Ticks,100 );
	           }
	           else if (Ticks < -100 )
	           {
	        	   Position_GetNewTarget(motor,-100 ,100);
	           }
	           else if (Ticks > 150 )
	           {
	        	   Position_GetNewTarget(motor,150,100);
	           }
	        }

//	        else if (Ticks >385)
//	        {
//	        	Position_GetNewTarget(motor,385 );
//	        }
//	        else if (Ticks<-385)
//	        {
//	        	Position_GetNewTarget(motor,-385 );
//	        }




}
//
//void control_mpu_incremental(Motor_t* motor, float Angle)
//{
////	int cur_position = 0;
//	if(Angle < 0){
//			Position_GetNewTarget(motor,motor->CurrentPosition++);
//	}
//	else if(Angle > 0){
//
//			Position_GetNewTarget(motor,motor->CurrentPosition++);
//
//	}
//}
