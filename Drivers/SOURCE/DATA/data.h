#ifndef __DATA_H
#define __DATA_H

//#include "stm32f10x.h"
#include "main.h"
extern  int str[8] ;
 int delta_calcForward(float theta1, float theta2, float theta3);
 int delta_calcAngleYZ(float x0, float y0, float z0);
 int delta_calcInverse(float x0, float y0, float z0);//运动学逆解
 void run(void);
  void map(void);
	void DATE(void);
	 struct  ZUOBIAO
{
    float x0;
    float y0;
    float z0;
    float rx;
    float ry;
    float rz;
} ;
 struct JIAODU
{
    float theta0;
    float theta1;
    float theta2;
    float theta3;
} ;
 extern struct ZUOBIAO PM;
//extern struct test c;
extern struct JIAODU PN;


#endif


