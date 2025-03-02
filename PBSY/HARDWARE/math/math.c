#include "math.h"

float GildeAverageValueFilter_float(float NewValue,float *Data,unsigned short int windows)
{
  float max,min;
  float sum;
  unsigned char i;
  Data[0]=NewValue;
  max=Data[0];
  min=Data[0];
  sum=Data[0];
  for(i=windows-1;i!=0;i--)	//循环四次，从后往前  
  {
    if(Data[i]>max) max=Data[i];
    else if(Data[i]<min) min=Data[i];
    sum+=Data[i];
    Data[i]=Data[i-1];	//数据右移
  }
  i=windows-2;
  sum=sum-max-min;
  sum=sum/i;
  return(sum);
}

int GildeAverageValueFilter_int(int NewValue,int *Data,unsigned short int windows)
{
  int max,min;
  int sum;
  unsigned char i;
  Data[0]=NewValue;
  max=Data[0];
  min=Data[0];
  sum=Data[0];
  for(i=windows-1;i!=0;i--)	//循环四次，从后往前  
  {
    if(Data[i]>max) max=Data[i];
    else if(Data[i]<min) min=Data[i];
    sum+=Data[i];
    Data[i]=Data[i-1];	//数据左移
  }
  i=windows-2;
  sum=sum-max-min;
  sum=(int)(sum/i);
  return(sum);
}

