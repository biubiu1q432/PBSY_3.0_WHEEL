#include "pid.h"
#include "tim.h"
#include "motor.h"
#include "math.h"
//位置
Pid local_pid;
//增量
Pid left_incremental_pid;
Pid right_incremental_pid;
//mpu
Pid mpu_pid;
//激光
Pid Lidar_pid;


extern Motor_Stat LEFT_MOTOR;	/*左轮数据*/
extern Motor_Stat RIG_MOTOR;	/*右轮数据*/
extern CAR_STAT Car_stat;
extern Lidar lidar;
extern int Queue_lidar[5]; 
extern int q_size ;

double test = 0;

//给结构体类型变量赋初值
void PID_init()
{
	//位置式
	local_pid.target_dis=0.000;
	local_pid.actual_dis=0.000;
	local_pid.output_val=0.000;
	local_pid.err=0.000;
	local_pid.err_last=0.000;
	local_pid.err_sum=0.000;
	local_pid.Kp=3;
	local_pid.Ki=0.01;
	local_pid.Kd=-2;

	
	//增量式
	left_incremental_pid.actual_val=0.000;
	left_incremental_pid.target_val=0.000;
	left_incremental_pid.output_pwm=0.000;	
	left_incremental_pid.err=0.000;
	left_incremental_pid.err_last=0.000;
	left_incremental_pid.err_pre=0.000;
	left_incremental_pid.err_sum=0.000;
	left_incremental_pid.Kp=6;
	left_incremental_pid.Ki=0.005;
	left_incremental_pid.Kd=5;

	right_incremental_pid.actual_val=0.000;
	right_incremental_pid.target_val=0.000;
	right_incremental_pid.output_pwm=0.000;	
	right_incremental_pid.err=0.000;
	right_incremental_pid.err_last=0.000;
	right_incremental_pid.err_pre=0.000;
	right_incremental_pid.err_sum=0.000;
	right_incremental_pid.Kp=6;
	right_incremental_pid.Ki=0.005;
	right_incremental_pid.Kd=5;
	
	//mpu
	mpu_pid.target_sita=0.000;
	mpu_pid.actual_sita=0.000;
	mpu_pid.output_sita_val=0.000;
	mpu_pid.err=0.000;
	mpu_pid.err_last=0.000;
	mpu_pid.err_pre=0.000;
	mpu_pid.err_sum=0.000;
	mpu_pid.Kp=0.42;
	mpu_pid.Ki=0.0015;
	mpu_pid.Kd=1;


	//lidar
	Lidar_pid.target_dis=0.000;
	Lidar_pid.actual_dis=0.000;
	Lidar_pid.output_val=0.000;
	Lidar_pid.err=0.000;
	Lidar_pid.err_last=0.000;
	Lidar_pid.err_sum=0.000;
	Lidar_pid.Kp=0.45;
	Lidar_pid.Ki=0;
	Lidar_pid.Kd=0.8;


}


/**************************************************************************
@bref    走直线（mpu + lidar + ec）
@para	 恒定速度： float val
@return 
**************************************************************************/
uint8_t CarGoAhead(float val)
{
	float val_mpu;
	float val_lidar;
	float l_val = 0;
	float r_val = 0;
	
//	//over--》激光突变
//	if(GoToEnd_judge(Queue_lidar,q_size)){
//		PID_init();
//		return 0;
//	}
	
	//角度环--》维持当前角度
	mpu_pid.target_sita = 0;
	val_mpu=PID_realize_mpu(&mpu_pid,Car_stat.Car_Alpha);
	
	//激光环--》维持当前角度
	val_lidar = PID_realize_lidar(&Lidar_pid);
	
	
	//速度环--> VAL = 基础速度 + K1 * 陀螺仪修正 + k2 * 激光修正
	l_val = val - val_mpu * MPU_WEIGHT_FOR_VAL - val_lidar *  LIDAR_WEIGHT_FOR_VAL;
	r_val = val + val_mpu * MPU_WEIGHT_FOR_VAL + val_lidar *  LIDAR_WEIGHT_FOR_VAL;
	
	
	
	Motor_Set_Val(l_val,r_val);

	return 0;

}

/**************************************************************************
@bref    DEBUG：lidar走直线（lidar + ec）  
@para	 恒定速度： float val
@return 
**************************************************************************/
uint8_t CarGoAhead_Lidar(float val)
{
	float val_lidar;
	float l_val;
	float r_val;
	
	//over--》激光突变
	
	
	//激光环--》维持当前角度
	val_lidar = PID_realize_lidar(&Lidar_pid);

	l_val = val - val_lidar  ;
	r_val = val + val_lidar  ;
	
	printf("err:	%d	 l_val:  %f    r_val:    %f\r\n",((int)lidar.LefLidar - (int)lidar.RigLidar),l_val,r_val);

	Motor_Set_Val(l_val,r_val);
	
	return 1;

}



/**************************************************************************
@bref    DEBUG：mpu走直线（mpu + ec）
@para	 恒定速度： float val
@return 
**************************************************************************/
uint8_t CarGoAhead_MPU(float val)
{
	float val_mpu;
	float l_val;
	float r_val;
	
	//over--》激光突变
	
	//角度环--》维持当前角度
	mpu_pid.target_sita = 0;
	val_mpu=PID_realize_mpu(&mpu_pid,Car_stat.Car_Alpha);
	
	l_val = val - val_mpu  ;
	r_val = val + val_mpu  ;
	
	//printf("l_val:		%f   r_val:		%f   sita:		%f\r\n",l_val,r_val,Car_stat.Car_Alpha);
	Motor_Set_Val(l_val,r_val);

	return 0;

}

/**************************************************************************
@bref: 	 走指定距离 (ec + MPU)	sita + dis  ---> val  ---> pwm
@para	：目标距离target_dis，限速（恒正）range_val
@return :  void
**************************************************************************/
uint8_t CarSetDis(float target_dis,float range_val)
{

	local_pid.target_dis  = target_dis;
	float acual_dis	= Car_stat.Car_Dis;
	
	//over
	float judge = (target_dis - acual_dis);
	if(judge> 0 && judge <= ALLOW_ERR_DIS){
		Motor_Set(0,0);
		PID_init();
		return 1;
	}
	if(judge<0 &&  judge>=-ALLOW_ERR_DIS){
		Motor_Set(0,0);
		PID_init();
		return 1;
	}
	
	//位置环
	float val = PID_realize_dis(&local_pid,acual_dis);
	//角度环
	mpu_pid.target_sita = 0;
	float val_mpu  = PID_realize_mpu(&mpu_pid,Car_stat.Car_Alpha);
	
	
	//限幅
	if(target_dis >= 0 ){
		if(val > range_val) val = range_val;
	}
	if(target_dis < 0 ){
		if(val < -range_val) val = -range_val;
	}
	
	
	Motor_Set_Val(val-val_mpu,val+val_mpu);
	return 0;

}


/**************************************************************************
@bref   ：转向 (mpu + ec)  sita --> val
@para	：目标距离target_dis，限速（恒正）range_val
@return :  void
**************************************************************************/
uint8_t CarTurn(float target_sita,float actual_sita,float max_val,float min_val)
{

	mpu_pid.target_sita = target_sita;
	float val=PID_realize_mpu(&mpu_pid,actual_sita);
	test = val;
	
	//over
	if(mpu_pid.err>=0 && mpu_pid.err < ALLOW_ERR_SITA){
		Motor_Set(0,0);
		PID_init();
		return 1;
	}
	if(mpu_pid.err<0 && mpu_pid.err > -ALLOW_ERR_SITA){
		Motor_Set(0,0);
		PID_init();
		return 1;
	}
	
	//限幅
	if( (val>0) && (val > max_val) )val=max_val;
	if( (val<0) && (val < -max_val) )val=-max_val;
	
	if( (val>0) && (val < min_val) )val=min_val;
	if( (val<0) && (val > -min_val) )val=-min_val;

		

	
	Motor_Set_Val(-val,val);
	
	return 0;

}

/**************************************************************************
@bref: 位置环 :dis --> val
@para	：目标距离target_dis，限速（恒正）range_val
@return: void
**************************************************************************/
uint8_t Motor_Set_Dis(float target_dis,float range_val)
{

	local_pid.target_dis  = target_dis;
	float acual_dis = Car_stat.Car_Dis;//当前位置
	
	//over
	float judge = (target_dis - acual_dis);
	if(judge> 0 && judge <= ALLOW_ERR_DIS){
			Motor_Set(0,0);
			PID_init();
			return 1;
	}
	if(judge<0 &&  judge>=-ALLOW_ERR_DIS){
		Motor_Set(0,0);
		PID_init();
		return 1;
	}
	
	float val = PID_realize_dis(&local_pid,acual_dis);
	
	//限幅
	if(target_dis >= 0 ){
		if(val > range_val) val = range_val;
	}
	if(target_dis < 0 ){
		if(val < -range_val) val = -range_val;
	}
		
	Motor_Set_Val(val,val);
	return 0;

}


 
/**************************************************************************
@bref: 速度环 :val ---> pwm
@para	：left_val,right_val
@return: void
**************************************************************************/
void Motor_Set_Val(float left_val,float right_val)
{
		float left_pwm,right_pwm;
		//目标赋值
		left_incremental_pid.target_val = left_val;
		right_incremental_pid.target_val = right_val;
		//速度环计算
		left_pwm = Incremental_PID_val(&left_incremental_pid,LEFT_MOTOR.Motorspeed);
		right_pwm = Incremental_PID_val(&right_incremental_pid,RIG_MOTOR.Motorspeed);
		
		
		//pwm + 限幅
		Motor_Set(left_pwm,right_pwm);
}


/**************************************************************************
@bref: 位置式PID控制器
@para	：当前位置	pid->actual_dis
@return: 指定速度	pid->output_val
**************************************************************************/

float PID_realize_dis(Pid * pid,float actual_dis)
{
	/*********************/
	pid->actual_dis = actual_dis;//传递真实值
	pid->err = pid->target_dis - pid->actual_dis;////当前误差=目标值-真实值		
	/*********************/
	
	pid->err_sum += pid->err;//误差累计值 = 当前误差累计和
	//使用PID控制 输出 = Kp*当前误差  +  Ki*误差累计值 + Kd*(当前误差-上次误差)
	pid->output_val = pid->Kp*pid->err + pid->Ki*pid->err_sum + pid->Kd*(pid->err - pid->err_last);	
	//保存上次误差: 这次误差赋值给上次误差
	pid->err_last = pid->err;
	
//	//限幅
//	if(pid->output_val < 0.5 && pid->output_val > 0) pid->output_val = 0;
//	if(pid->output_val > -0.5 && pid->output_val < 0) pid->output_val = 0;
	
	
	
	return pid->output_val;
}


float PID_realize_mpu(Pid * pid,float sita)
{
	/*********************/
	pid->actual_sita = sita;//传递真实值
	pid->err = pid->target_sita - pid->actual_sita;////当前误差=目标值-真实值		
	/*********************/
	
	pid->err_sum += pid->err;//误差累计值 = 当前误差累计和
	//使用PID控制 输出 = Kp*当前误差  +  Ki*误差累计值 + Kd*(当前误差-上次误差)
	pid->output_sita_val = pid->Kp*pid->err + pid->Ki*pid->err_sum + pid->Kd*(pid->err - pid->err_last);	
	//保存上次误差: 这次误差赋值给上次误差
	pid->err_last = pid->err;
		
	return pid->output_sita_val;
}


float PID_realize_lidar(Pid * pid)
{
	/*********************/
	pid->lidar_err = ((int)lidar.LefLidar - (int)lidar.RigLidar);
	/*********************/
	
	pid->err_sum += pid->lidar_err;//误差累计值 = 当前误差累计和
	//使用PID控制 输出 = Kp*当前误差  +  Ki*误差累计值 + Kd*(当前误差-上次误差)
	pid->output_val = pid->Kp*pid->lidar_err + pid->Ki*pid->err_sum + pid->Kd*(pid->lidar_err - pid->err_last);	
	//保存上次误差: 这次误差赋值给上次误差
	pid->err_last = pid->lidar_err;

	//printf("pid->actual_dis 		%f\r\n",pid->actual_dis);
		
	return pid->output_val;
}


/**************************************************************************
函数功能：增量PID控制器
入口参数：实际值，目标值
返回  值：电机PWM
根据增量式离散PID公式 
pwm=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]		对应关系（增量式-》位置式）kp->kd ki->kp kd->ki
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
**************************************************************************/
float Incremental_PID_val(Pid * pid,float actual_val)
{ 		
	//计算
	pid->actual_val = actual_val;//传递真实值 
	pid->err = pid->target_val - pid->actual_val;//当前误差=目标值-真实值   
	pid->output_pwm  += (pid->Kd*(pid->err - pid->err_last))               /* 比例环节 */
									 + (pid->Kp * pid->err)                           /* 积分环节 */
									 + (pid->Ki*(pid->err - 2*pid->err_last + pid->err_pre));  /* 微分环节 */ 
    
	pid->err_pre=pid->err_last;                                   /* 保存上上次偏差 */
	pid->err_last=pid->err;	                                    /* 保存上一次偏差 */

	return pid->output_pwm;                                            /* 输出结果 */
}

	
