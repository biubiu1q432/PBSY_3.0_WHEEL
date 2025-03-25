#ifndef __MYI2C_H
#define __MYI2C_H

void Delayus(uint32_t usdelay);
void MyI2C_Init(uint8_t Turn);
void MyI2C_Start(uint8_t Turn);
void MyI2C_Stop(uint8_t Turn);
void MyI2C_SendByte(uint8_t Byte,uint8_t Turn);
uint8_t MyI2C_ReceiveByte(uint8_t Turn);
void MyI2C_SendAck(uint8_t AckBit,uint8_t Turn);
uint8_t MyI2C_ReceiveAck(uint8_t Turn);
void MyI2C_W_SCL(uint8_t BitValue,uint8_t Turn);
void MyI2C_W_SDA(uint8_t BitValue,uint8_t Turn);
uint8_t MyI2C_R_SDA(uint8_t Turn);

#endif
