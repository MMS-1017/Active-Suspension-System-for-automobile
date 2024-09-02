/*
 * pid_mpu.c
 *
 *  Created on: Jul 9, 2024
 *      Author: sherief alglaly
 */
#include "../Inc/pid_mpu.h"

void pid_init(PIDController *pid, double kp, double ki, double kd, double setpoint) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->prev_error = 0;
    pid->integral = 0;
    pid->setpoint = setpoint;
}

double pid_compute(PIDController *pid, double measured_value, double dt) {
    double error = pid->setpoint - measured_value;
    pid->integral += error * dt;
    double derivative = (error - pid->prev_error) / dt;

    double output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    pid->prev_error = error;

    return -output;
}
