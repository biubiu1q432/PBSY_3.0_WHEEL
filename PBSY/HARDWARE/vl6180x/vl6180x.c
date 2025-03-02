
#include "vl6180x.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include "MyI2C.h"
#include "i2c.h"
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

/*
0:LEFT_LIDAR
1:RIGHT_LIDAR
*/

uint8_t VL6180X_WriteReg(uint16_t reg,uint8_t data,uint8_t TURN)
{
	
	if (TURN==0)
		HAL_I2C_Mem_Write(&hi2c2,(VL6180X_DEFAULT_I2C_ADDR<<1)|0,reg,I2C_MEMADD_SIZE_16BIT,&data,1, HAL_MAX_DELAY);
	else if(TURN == 1)
		HAL_I2C_Mem_Write(&hi2c1,(VL6180X_DEFAULT_I2C_ADDR<<1)|0,reg,I2C_MEMADD_SIZE_16BIT,&data,1, HAL_MAX_DELAY);

	return 0;
}
uint8_t VL6180X_ReadReg(uint16_t reg,uint8_t TURN)
{
	uint8_t data;
	
	if (TURN==0)
		HAL_I2C_Mem_Read(&hi2c2,(VL6180X_DEFAULT_I2C_ADDR<<1)|1,reg,I2C_MEMADD_SIZE_16BIT,&data,1,HAL_MAX_DELAY);
	else if(TURN == 1)
		HAL_I2C_Mem_Read(&hi2c1,(VL6180X_DEFAULT_I2C_ADDR<<1)|1,reg,I2C_MEMADD_SIZE_16BIT,&data,1,HAL_MAX_DELAY);

	
	return data;
}


uint8_t VL6180X_Init(uint8_t TURN)
{
	uint8_t ret=0;
	if(VL6180X_Read_ID(TURN) == VL6180X_DEFAULT_ID)
	{	
		ret+=VL6180X_WriteReg(0x0207, 0x01,TURN);
		ret+=VL6180X_WriteReg(0x0208, 0x01,TURN);
		ret+=VL6180X_WriteReg(0x0096, 0x00,TURN);
		ret+=VL6180X_WriteReg(0x0097, 0xfd,TURN);
		ret+=VL6180X_WriteReg(0x00e3, 0x00,TURN);
		ret+=VL6180X_WriteReg(0x00e4, 0x04,TURN);
		ret+=VL6180X_WriteReg(0x00e5, 0x02,TURN);
		ret+=VL6180X_WriteReg(0x00e6, 0x01,TURN);
		ret+=VL6180X_WriteReg(0x00e7, 0x03,TURN);
		ret+=VL6180X_WriteReg(0x00f5, 0x02,TURN);
		ret+=VL6180X_WriteReg(0x00d9, 0x05,TURN);
		ret+=VL6180X_WriteReg(0x00db, 0xce,TURN);
		ret+=VL6180X_WriteReg(0x00dc, 0x03,TURN);
		ret+=VL6180X_WriteReg(0x00dd, 0xf8,TURN);
		ret+=VL6180X_WriteReg(0x009f, 0x00,TURN);
		ret+=VL6180X_WriteReg(0x00a3, 0x3c,TURN);
		ret+=VL6180X_WriteReg(0x00b7, 0x00,TURN);
		ret+=VL6180X_WriteReg(0x00bb, 0x3c,TURN);
		ret+=VL6180X_WriteReg(0x00b2, 0x09,TURN);
		ret+=VL6180X_WriteReg(0x00ca, 0x09,TURN);
		ret+=VL6180X_WriteReg(0x0198, 0x01,TURN);
		ret+=VL6180X_WriteReg(0x01b0, 0x17,TURN);
		ret+=VL6180X_WriteReg(0x01ad, 0x00,TURN);
		ret+=VL6180X_WriteReg(0x00ff, 0x05,TURN);
		ret+=VL6180X_WriteReg(0x0100, 0x05,TURN);
		ret+=VL6180X_WriteReg(0x0199, 0x05,TURN);
		ret+=VL6180X_WriteReg(0x01a6, 0x1b,TURN);
		ret+=VL6180X_WriteReg(0x01ac, 0x3e,TURN);
		ret+=VL6180X_WriteReg(0x01a7, 0x1f,TURN);
		ret+=VL6180X_WriteReg(0x0030, 0x00,TURN);
		
		// Recommended : Public registers - See data sheet for more detail
		ret+=VL6180X_WriteReg(0x0011, 0x10,TURN);       // Enables polling for 'New Sample ready'
									// when measurement completes
		ret+=VL6180X_WriteReg(0x010a, 0x30,TURN);       // Set the averaging sample period
									// (compromise between lower noise and
									// increased execution time)
		ret+=VL6180X_WriteReg(0x003f, 0x46,TURN);       // Sets the light and dark gain (upper
									// nibble). Dark gain should not be
									// changed. !ʏѫؖޚҪдɫ0x4	ĬɏնӦˇ1.0
		ret+=VL6180X_WriteReg(0x0031, 0xFF,TURN);       // sets the # of range measurements after
									// which auto calibration of system is
									// performed
		ret+=VL6180X_WriteReg(0x0040, 0x63,TURN);       // Set ALS integration time to 100ms
		ret+=VL6180X_WriteReg(0x002e, 0x01,TURN);       // perform a single temperature calibration
									// of the ranging sensor

		// Optional: Public registers - See data sheet for more detail
		ret+=VL6180X_WriteReg(0x001b, 0x09,TURN);    //Ӣݤٴ	Öѯģʽ
									// period to 100ms	ÿҽ10ms->0-10ms
		ret+=VL6180X_WriteReg(0x003e, 0x31,TURN);      //Ӣלǚ	ALSģʽ
									// to 500ms		
		ret+=VL6180X_WriteReg(0x0014, 0x24,TURN);       // Configures interrupt on 'New Sample
									// Ready threshold event'
		
		
		
		ret+=VL6180X_WriteReg(0x024,100,TURN);
		
		
		
		
		if(ret == 0)return 0;
		else return 1;

	}
	else return 1;
}

uint8_t VL6180X_Read_ID(uint8_t TURN)
{
	return VL6180X_ReadReg(VL6180X_REG_IDENTIFICATION_MODEL_ID,TURN);
}

uint8_t VL6180X_Read_Range(uint8_t TURN)
{
	uint8_t range = 0;
	while(!(VL6180X_ReadReg(VL6180X_REG_RESULT_RANGE_STATUS,TURN) & 0x01));
	VL6180X_WriteReg(VL6180X_REG_SYSRANGE_START,0x01,TURN);
	while(!(VL6180X_ReadReg(VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO,TURN) & 0x04));
	range = VL6180X_ReadReg(VL6180X_REG_RESULT_RANGE_VAL,TURN);
	VL6180X_WriteReg(VL6180X_REG_SYSTEM_INTERRUPT_CLEAR,0x07,TURN);
	return range;
}




