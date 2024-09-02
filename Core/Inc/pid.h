/*
 * pid.h
 *
 *  Created on: Jul 9, 2024
 *      Author: Eng.M
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct {
    double Kp;
    double Ki;
    double Kd;
    double setpoint;
    double input;
    double output;
    double integral;
    double previous_error;
} PID_TypeDef;


void PID_Init(PID_TypeDef *pid, double Kp, double Ki, double Kd);
void PID_Setpoint(PID_TypeDef *pid, double setpoint);
void PID_Input(PID_TypeDef *pid, double input);
double PID_Compute(PID_TypeDef *pid);



#endif /* INC_PID_H_ */
