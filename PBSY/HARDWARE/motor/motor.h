#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "tim.h"



typedef struct
{
	float Distance;
	float Motorspeed;
} Motor_Stat;


typedef struct{
	 float Car_omiga;
	 float Car_Speed;
	 float Car_Alpha;
	 float Car_Dis;
}CAR_STAT;


//PWM
void Motor_Set(float Left_PWM,float Right_PWM);

//EC
void CarStat_Get(void);

//car
void Car_Stat_init(void);


#endif


