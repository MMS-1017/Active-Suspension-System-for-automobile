/*
 * pid.c
 *
 *  Created on: Jul 9, 2024
 *      Author: Eng.M
 */

#include "pid.h"

void PID_Init(PID_TypeDef *pid, double Kp, double Ki, double Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0;
    pid->previous_error = 0;
}

void PID_Setpoint(PID_TypeDef *pid, double setpoint)
{
    pid->setpoint = setpoint;
}

void PID_Input(PID_TypeDef *pid, double input)
{
    pid->input = input;
}

double PID_Compute(PID_TypeDef *pid)
{
    double error = pid->setpoint - pid->input;
    pid->integral += error;
    double derivative = error - pid->previous_error;

    pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    pid->previous_error = error;

    return pid->output;
}


