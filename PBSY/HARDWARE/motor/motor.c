#include "motor.h"

extern CAR_STAT Car_stat;

Motor_Stat LEFT_MOTOR;
Motor_Stat RIG_MOTOR;

/**************************************************************************
@bref: 限幅环(pwm)：采用慢衰减，反比
@para	：float PWM
@return: void
**************************************************************************/
void Motor_Set(float Left_PWM,float Right_PWM)
{
	/*LEFT*/
	
	//T_V<0
	if(Left_PWM <= 0){
		//方向
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,MOTOR_ARR);
		//限幅
		if(Left_PWM<-MAX_PWM)Left_PWM = -MAX_PWM;
		if(Left_PWM>-MIN_PWM)Left_PWM=0;
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,MOTOR_ARR+Left_PWM);		
	} 
	
	//T_V>0
	if(Left_PWM >0){
		//方向
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,MOTOR_ARR);
		//限幅
		if(Left_PWM>MAX_PWM)Left_PWM=MAX_PWM;
		if(Left_PWM<=MIN_PWM)Left_PWM=0;
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,MOTOR_ARR-Left_PWM);
	} 

	/*RIGHT*/
	
	//T_V<0
	if(Right_PWM <= 0){
		//方向
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,MOTOR_ARR);
		//限
		if(Right_PWM<-MAX_PWM)Right_PWM=-MAX_PWM;
		if(Right_PWM>-MIN_PWM)Right_PWM=0;
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,Right_PWM+MOTOR_ARR);		
	} 
	
	//T_V>0
	if(Right_PWM >0){
		//方向
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,MOTOR_ARR);
		//限幅
		if(Right_PWM>MAX_PWM)Right_PWM=MAX_PWM;
		if(Right_PWM<MIN_PWM)Right_PWM=0;
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,MOTOR_ARR-Right_PWM);
	} 
	

}


/**************************************************************************
@bref: 左右轮参数获取
@para	：void
@return: void
**************************************************************************/
void CarStat_Get(void){
		
	int left_ec = __HAL_TIM_GET_COUNTER(&htim1);
	__HAL_TIM_SET_COUNTER(&htim1,0);
	if(left_ec > 60000)	left_ec = left_ec - EC_ARR;
	float left_distance = (left_ec/(float)LEFT_EC_1)*C_1;
	LEFT_MOTOR.Distance += left_distance;
	LEFT_MOTOR.Motorspeed = left_distance/ENCODER_TIME;

	int right_ec = __HAL_TIM_GET_COUNTER(&htim2);
	__HAL_TIM_SET_COUNTER(&htim2,0);		
	if(right_ec > 60000)	right_ec = right_ec - EC_ARR;
	float right_distance = (right_ec/(float)RIGHT_EC_1)*C_1;
	RIG_MOTOR.Distance += right_distance;
	RIG_MOTOR.Motorspeed = right_distance/ENCODER_TIME;
	
	
	//Car_Stat
	Car_stat.Car_Speed = (RIG_MOTOR.Motorspeed + LEFT_MOTOR.Motorspeed)/2.0f;
	Car_stat.Car_Dis = (LEFT_MOTOR.Distance + RIG_MOTOR.Distance)/2.0f;
	Car_stat.Car_omiga =( Car_stat.Car_Speed * SINANGLE ) / RADIUS;

}


/*
	相对角度
	距离重置
*/
void Car_Stat_ReFresh(void)
{
	Car_stat.Car_LastAlpha = Car_stat.Car_Alpha;
	Car_stat.Car_Dis = 0;
}





