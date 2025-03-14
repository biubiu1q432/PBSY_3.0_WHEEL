#ifndef __PID_H
#define __PID_H

#include "main.h"


//pid计算器
typedef struct 
{
	float target_val;
	float actual_val;//实际值
	float output_pwm;//输出值
	
	float target_dis;
	float actual_dis;
	float output_val;
	
	float target_sita;
	float actual_sita;
	float output_sita_val;
	
	float err;//当前偏差
	float err_last;//上次偏差
	float err_pre;//上上次偏差
	float err_sum;//误差累计值
	float Kp,Ki,Kd;//比例，积分，微分系数
		 
} Pid;


//参数初始化
void PID_init(void);
//位置式
float PI_realize(Pid * pid,float actual_val);
float PID_realize_dis(Pid * pid,float actual_val);
float PID_realize_mpu(Pid * pid,float sita);
//增量式
float Incremental_PID_val(Pid * pid,float actual_val);
//速度环
void Motor_Set_Val(float left_val,float right_val);
//位置环
uint8_t Motor_Set_Dis(float target_dis,float range_val);
uint8_t Motor_Set_Sita(float target_sita,float actual_sita,float range_val);

#endif


