#include "main.h"                  // Device header
#include "gpio.h"
#include "MyI2C.h"

/*================================================*/


void MyI2C_W_SCL(uint8_t BitValue,uint8_t Turn)
{
    if(Turn == 1)			HAL_GPIO_WritePin(L_SCL_GPIO_Port,L_SCL_Pin,(GPIO_PinState)BitValue);
	else if(Turn == 2) 	HAL_GPIO_WritePin(R_SCL_GPIO_Port,R_SCL_Pin,(GPIO_PinState)BitValue);
    Delayus(I2C_DELAY);
}

void MyI2C_W_SDA(uint8_t BitValue,uint8_t Turn)
{
    if(Turn == 1)			HAL_GPIO_WritePin(L_SDA_GPIO_Port,L_SDA_Pin,(GPIO_PinState)BitValue);
	else if(Turn == 2) 	HAL_GPIO_WritePin(R_SDA_GPIO_Port,R_SDA_Pin,(GPIO_PinState)BitValue);   
	Delayus(I2C_DELAY);
}

uint8_t MyI2C_R_SDA(uint8_t Turn)
{
    uint8_t BitValue;
    
	if(Turn == 1)			BitValue = (uint8_t)HAL_GPIO_ReadPin(L_SDA_GPIO_Port,L_SDA_Pin);
	else if(Turn == 2) 	BitValue = (uint8_t)HAL_GPIO_ReadPin(R_SDA_GPIO_Port,R_SDA_Pin);
    Delayus(I2C_DELAY);
    
	return BitValue;
}
/*================================================*/

void Delayus(uint32_t usdelay)
{
    __IO uint32_t Delay = usdelay * (SystemCoreClock /8U/1000U/1000);
    do
    {
        __NOP();
    }
    while (Delay --);
}


void MyI2C_Start(uint8_t Turn)
{
    MyI2C_W_SDA(1,Turn);
    MyI2C_W_SCL(1,Turn);
    MyI2C_W_SDA(0,Turn);
    MyI2C_W_SCL(0,Turn);
}

void MyI2C_Stop(uint8_t Turn)
{
    MyI2C_W_SDA(0,Turn);
    MyI2C_W_SCL(1,Turn);
    MyI2C_W_SDA(1,Turn);
}

void MyI2C_SendByte(uint8_t Byte,uint8_t Turn)
{
    uint8_t i;
    for (i = 0; i < 8; i ++)
    {
        MyI2C_W_SDA(Byte & (0x80 >> i),Turn);
        MyI2C_W_SCL(1,Turn);
        MyI2C_W_SCL(0,Turn);
    }
}

uint8_t MyI2C_ReceiveByte(uint8_t Turn)
{
    uint8_t i, Byte = 0x00;
    MyI2C_W_SDA(1,Turn);
    for (i = 0; i < 8; i ++)
    {
        MyI2C_W_SCL(1,Turn);
        if (MyI2C_R_SDA(Turn) == 1) {
            Byte |= (0x80 >> i);
        }
        MyI2C_W_SCL(0,Turn);
    }
    return Byte;
}

void MyI2C_SendAck(uint8_t AckBit,uint8_t Turn)
{
    MyI2C_W_SDA(AckBit,Turn);
    MyI2C_W_SCL(1,Turn);
    MyI2C_W_SCL(0,Turn);
}

uint8_t MyI2C_ReceiveAck(uint8_t Turn)
{
    uint8_t AckBit;
    MyI2C_W_SDA(1,Turn);
    MyI2C_W_SCL(1,Turn);
    AckBit = MyI2C_R_SDA(Turn);
    MyI2C_W_SCL(0,Turn);
    return AckBit;
}
