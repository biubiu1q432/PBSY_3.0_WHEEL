#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "tim.h"



typedef struct
{
	int EncodeCount;
	float Distance;
	float Motorspeed;
} Motor_Stat;


typedef struct{
	 Motor_Stat LEFT_MOTOR;
	 Motor_Stat RIG_MOTOR;
	 float Car_Alpha;
}CAR_STAT;


//PWM
void Motor_Set(float Left_PWM,float Right_PWM);

//EC
void MotorSpeed_Get(void);

#endif


