#include "main.h"                  // Device header
#include "gpio.h"
#include "MyI2C.h"




void Delayus(uint32_t usdelay)
{
  __IO uint32_t Delay = usdelay * (SystemCoreClock /8U/1000U/1000);//SystemCoreClock:ÏµÍ³ÆµÂÊ
  do
  {
    __NOP();
  }
  while (Delay --);
}	



void MyI2C_W_SCL(uint8_t BitValue)
{
	HAL_GPIO_WritePin(SCL_GPIO_Port,SCL_Pin,(GPIO_PinState)BitValue);
	Delayus(5);
}

void MyI2C_W_SDA(uint8_t BitValue)
{
	HAL_GPIO_WritePin(SDA_GPIO_Port,SDA_Pin,(GPIO_PinState)BitValue);
	Delayus(5);
}

uint8_t MyI2C_R_SDA(void)
{
	uint8_t BitValue;
	BitValue = (uint8_t)HAL_GPIO_ReadPin(SDA_GPIO_Port,SDA_Pin);
	Delayus(5);
	return BitValue;
}


void MyI2C_Start(void)
{
	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(0);
}

void MyI2C_Stop(void)
{
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(1);
}

void MyI2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i ++)
	{
		MyI2C_W_SDA(Byte & (0x80 >> i));
		MyI2C_W_SCL(1);
		MyI2C_W_SCL(0);
	}
}

uint8_t MyI2C_ReceiveByte(void)
{
	uint8_t i, Byte = 0x00;
	MyI2C_W_SDA(1);
	for (i = 0; i < 8; i ++)
	{
		MyI2C_W_SCL(1);
		if (MyI2C_R_SDA() == 1){Byte |= (0x80 >> i);}
		MyI2C_W_SCL(0);
	}
	return Byte;
}

void MyI2C_SendAck(uint8_t AckBit)
{
	MyI2C_W_SDA(AckBit);
	MyI2C_W_SCL(1);
	MyI2C_W_SCL(0);
}

uint8_t MyI2C_ReceiveAck(void)
{
	uint8_t AckBit;
	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);
	AckBit = MyI2C_R_SDA();
	MyI2C_W_SCL(0);
	return AckBit;
}
