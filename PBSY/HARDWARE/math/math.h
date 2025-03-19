#ifndef __MATH_H
#define __MATH_H

#include "main.h"
#include "stdio.h"

float GildeAverageValueFilter_float(float NewValue,float *Data,unsigned short int windows);
int GildeAverageValueFilter_int(int NewValue,int *Data,unsigned short int windows);
void Sliding_Window_Algorithm(int* ptr,int size,Lidar* lidar);
bool GoToEnd_judge(int* ptr,int size);


#endif
