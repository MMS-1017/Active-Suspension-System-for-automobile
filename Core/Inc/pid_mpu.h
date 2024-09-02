/*
 * pid_mpu.h
 *
 *  Created on: Jul 9, 2024
 *      Author: sherief alglaly
 */

#ifndef INC_PID_MPU_H_
#define INC_PID_MPU_H_

typedef struct {
    double kp;    // Proportional gain
    double ki;    // Integral gain
    double kd;    // Derivative gain
    double prev_error; // Previous error
    double integral;   // Integral of the error
    double setpoint;   // Desired value
} PIDController;


void pid_init(PIDController *pid, double kp, double ki, double kd, double setpoint);
double pid_compute(PIDController *pid, double measured_value, double dt);

#endif /* INC_PID_MPU_H_ */
