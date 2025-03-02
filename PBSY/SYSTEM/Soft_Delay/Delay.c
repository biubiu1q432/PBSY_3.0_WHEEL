#include"main.h"

//微秒级的延时
void delay_us(uint32_t delay_us)
{    
	volatile unsigned int num;
	volatile unsigned int t;

	for (num = 0; num < delay_us; num++)
	{
			t = 11;
			while (t != 0)
			{
			t--;
			}
	}
}
//毫秒级的延时
void delay_ms(uint16_t delay_ms)
{    
	volatile unsigned int num;
	for (num = 0; num < delay_ms; num++)
	{
			delay_us(1000);
	}
}

