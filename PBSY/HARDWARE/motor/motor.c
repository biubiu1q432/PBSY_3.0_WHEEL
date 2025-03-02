#include "motor.h"

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
void MotorSpeed_Get(void){
		
	int left_ec = __HAL_TIM_GET_COUNTER(&htim1);
	__HAL_TIM_SET_COUNTER(&htim1,0);
	if(left_ec > 60000)	left_ec = left_ec - EC_ARR;
	LEFT_MOTOR.EncodeCount = left_ec;
	float left_distance = (float)(LEFT_MOTOR.EncodeCount/LEFT_EC_1)*C_1;
	LEFT_MOTOR.Distance += left_distance;
	LEFT_MOTOR.Motorspeed = left_distance/ENCODER_TIME;

	int right_ec = __HAL_TIM_GET_COUNTER(&htim2);
	__HAL_TIM_SET_COUNTER(&htim2,0);		
	if(right_ec > 60000)	right_ec = right_ec - EC_ARR;
	RIG_MOTOR.EncodeCount = -right_ec;	
	float right_distance = ((float)RIG_MOTOR.EncodeCount/(float)RIGHT_EC_1)*C_1;
	RIG_MOTOR.Distance += right_distance;
	RIG_MOTOR.Motorspeed = right_distance/ENCODER_TIME;

}



