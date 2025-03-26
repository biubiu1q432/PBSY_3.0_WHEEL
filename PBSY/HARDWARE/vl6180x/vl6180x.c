#include "vl6180x.h"



#if I2CHARDWARE == 1
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c1;
#endif



/*
@breif  距离标定
@parm	float Cailbration_dis 固定标定距离 uint8_t repit 重复次数
*/
void VL6180X_Range_Cailbration(Lidar *lidar,int Cailbration_dis,uint8_t repit){
	
	int sum=0;
	int dis=0;
	
	for(int i = 1;i<=repit;i++){
		sum += VL6180X_Read_Range(1);
	} 
	
	dis = sum/repit;
	lidar->Lef_Cali = dis - Cailbration_dis;
		
	sum= 0;
	
	for(int i = 1;i<=repit;i++){
		sum += VL6180X_Read_Range(2);
	} 
	dis = sum/repit;
	lidar->Rig_Cali = dis - Cailbration_dis;

}




uint8_t VL6180X_Init(uint8_t Turn)
{
	if(VL6180X_Read_ID(Turn) == VL6180X_DEFAULT_ID)
	{	
		VL6180X_WriteReg(0x0207, 0x01,Turn);
		VL6180X_WriteReg(0x0208, 0x01,Turn);
		VL6180X_WriteReg(0x0096, 0x00,Turn);
		VL6180X_WriteReg(0x0097, 0xfd,Turn);
		VL6180X_WriteReg(0x00e3, 0x00,Turn);
		VL6180X_WriteReg(0x00e4, 0x04,Turn);
		VL6180X_WriteReg(0x00e5, 0x02,Turn);
		VL6180X_WriteReg(0x00e6, 0x01,Turn);
		VL6180X_WriteReg(0x00e7, 0x03,Turn);
		VL6180X_WriteReg(0x00f5, 0x02,Turn);
		VL6180X_WriteReg(0x00d9, 0x05,Turn);
		VL6180X_WriteReg(0x00db, 0xce,Turn);
		VL6180X_WriteReg(0x00dc, 0x03,Turn);
		VL6180X_WriteReg(0x00dd, 0xf8,Turn);
		VL6180X_WriteReg(0x009f, 0x00,Turn);
		VL6180X_WriteReg(0x00a3, 0x3c,Turn);
		VL6180X_WriteReg(0x00b7, 0x00,Turn);
		VL6180X_WriteReg(0x00bb, 0x3c,Turn);
		VL6180X_WriteReg(0x00b2, 0x09,Turn);
		VL6180X_WriteReg(0x00ca, 0x09,Turn);
		VL6180X_WriteReg(0x0198, 0x01,Turn);
		VL6180X_WriteReg(0x01b0, 0x17,Turn);
		VL6180X_WriteReg(0x01ad, 0x00,Turn);
		VL6180X_WriteReg(0x00ff, 0x05,Turn);
		VL6180X_WriteReg(0x0100, 0x05,Turn);
		VL6180X_WriteReg(0x0199, 0x05,Turn);
		VL6180X_WriteReg(0x01a6, 0x1b,Turn);
		VL6180X_WriteReg(0x01ac, 0x3e,Turn);
		VL6180X_WriteReg(0x01a7, 0x1f,Turn);
		VL6180X_WriteReg(0x0030, 0x00,Turn);
		
		// Recommended : Public registers - See data sheet for more detail
		VL6180X_WriteReg(0x0011, 0x10,Turn);       // Enables polling for 'New Sample ready'
									// when measurement completes
		VL6180X_WriteReg(0x010a, 0x30,Turn);       // Set the averaging sample period
									// (compromise between lower noise and
									// increased execution time)
		VL6180X_WriteReg(0x003f, 0x46,Turn);       // Sets the light and dark gain (upper
									// nibble). Dark gain should not be
									// changed. !上半字节要写入0x4	默认增益是1.0
		VL6180X_WriteReg(0x0031, 0xFF,Turn);       // sets the # of range measurements after
									// which auto calibration of system is
									// performed
		VL6180X_WriteReg(0x0040, 0x63,Turn);       // Set ALS integration time to 100ms
		VL6180X_WriteReg(0x002e, 0x01,Turn);       // perform a single temperature calibration
									// of the ranging sensor

		// Optional: Public registers - See data sheet for more detail
		VL6180X_WriteReg(0x001b, 0x09,Turn);    //测量间隔	轮询模式
									// period to 100ms	每步10ms->0-10ms
		VL6180X_WriteReg(0x003e, 0x31,Turn);      //测量周期	ALS模式
									// to 500ms		
		VL6180X_WriteReg(0x0014, 0x24,Turn);       // Configures interrupt on 'New Sample
									// Ready threshold event'
		return 0;
	}
	else return 1;
}




uint8_t VL6180X_Read_ID(uint8_t Turn)
{
	return VL6180X_ReadReg(VL6180X_REG_IDENTIFICATION_MODEL_ID,Turn);
}




uint8_t VL6180X_Read_Range(uint8_t Turn)
{
	
	uint8_t range = 0;
	while(!(VL6180X_ReadReg(VL6180X_REG_RESULT_RANGE_STATUS,Turn) & 0x01));
	VL6180X_WriteReg(VL6180X_REG_SYSRANGE_START,0x01,Turn);
	while(!(VL6180X_ReadReg(VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO,Turn) & 0x04));
	range = VL6180X_ReadReg(VL6180X_REG_RESULT_RANGE_VAL,Turn);
	VL6180X_WriteReg(VL6180X_REG_SYSTEM_INTERRUPT_CLEAR,0x07,Turn);
	return range;
}


uint8_t VL6180X_WriteReg(uint16_t reg,uint8_t data,uint8_t Turn)
{

#if I2CHARDWARE == 1	

	if(Turn ==2)		HAL_I2C_Mem_Write(&hi2c2,(VL6180X_DEFAULT_I2C_ADDR<<1)|0,reg,I2C_MEMADD_SIZE_16BIT,&data,1,LIDAR_MAX_WAIT);
	else if(Turn == 1)HAL_I2C_Mem_Write(&hi2c1,(VL6180X_DEFAULT_I2C_ADDR<<1)|0,reg,I2C_MEMADD_SIZE_16BIT,&data,1,LIDAR_MAX_WAIT);
	return 0;

#elif	I2CHARDWARE == 2
	
	uint8_t Index_H = (uint8_t)(reg >> 8);//高8位
	uint8_t Index_L = (uint8_t)(reg & 0xFF);//低8位
	
	MyI2C_Start(Turn);
	MyI2C_SendByte((VL6180X_DEFAULT_I2C_ADDR<<1)|0,Turn);
	if(MyI2C_ReceiveAck(Turn))	//等待应答
	{
		MyI2C_Stop(Turn);	
		return 1;		
	}
	MyI2C_SendByte(Index_H,Turn);
	MyI2C_ReceiveAck(Turn);	//等待ACK
	MyI2C_SendByte(Index_L,Turn);
	MyI2C_ReceiveAck(Turn);	//等待ACK
	MyI2C_SendByte(data,Turn);
	if(MyI2C_ReceiveAck(Turn))	//等待ACK
	{
		MyI2C_Stop(Turn);	 
		return 1;		 
	}
	MyI2C_Stop(Turn);
	return 0;	
	
#endif	
}
uint8_t VL6180X_ReadReg(uint16_t reg,uint8_t Turn)
{ 
	
#if I2CHARDWARE == 1	
	
	uint8_t data;
	if(Turn ==2)		HAL_I2C_Mem_Read(&hi2c2,(VL6180X_DEFAULT_I2C_ADDR<<1)|1,reg,I2C_MEMADD_SIZE_16BIT,&data,1,LIDAR_MAX_WAIT);
	else if(Turn == 1)HAL_I2C_Mem_Read(&hi2c1,(VL6180X_DEFAULT_I2C_ADDR<<1)|1,reg,I2C_MEMADD_SIZE_16BIT,&data,1,LIDAR_MAX_WAIT);
	return data;
 
#elif	 I2CHARDWARE == 2

	uint8_t Data;
	uint8_t Index_H = (uint8_t)(reg >> 8);
	uint8_t Index_L = (uint8_t)(reg & 0xff);
	MyI2C_Start(Turn); 
	MyI2C_SendByte((VL6180X_DEFAULT_I2C_ADDR<<1)|0,Turn);//发送器件地址+写命令	
	MyI2C_ReceiveAck(Turn);		//等待应答 
	MyI2C_SendByte(Index_H,Turn);	//写寄存器地址
	MyI2C_ReceiveAck(Turn);		//等待应答
	MyI2C_SendByte(Index_L,Turn);	//写寄存器地址
	MyI2C_ReceiveAck(Turn);	
	
	MyI2C_Start(Turn);
	MyI2C_SendByte((VL6180X_DEFAULT_I2C_ADDR<<1)|1,Turn);//发送器件地址+读命令	
	MyI2C_ReceiveAck(Turn);		//等待应答 
	Data=MyI2C_ReceiveByte(Turn);//读取数据,发送nACK 
	MyI2C_Stop(Turn);			//产生一个停止条件 
	return Data;
	
#endif
}




