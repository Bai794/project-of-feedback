#ifndef _KEY_H
#define _KEY_H
//#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F103������
//KEY��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2019/11/15
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

//����ķ�ʽ��ͨ��λ��������ʽ��ȡIO
//#define KEY0        PCin(5) 	//KEY0����PC5
//#define KEY1        PAin(15) 	//KEY1����PA15
//#define WK_UP       PAin(0)	    //WKUP����PA0
/*

#define M_Z_1_Pin GPIO_PIN_11
#define M_Z_1_GPIO_Port GPIOE
#define M_Z_2_Pin GPIO_PIN_13
#define M_Z_2_GPIO_Port GPIOE
#define M_Z_3_Pin GPIO_PIN_15
#define M_Z_3_GPIO_Port GPIOE

*/
//����ķ�ʽ��ͨ��ֱ�Ӳ���HAL�⺯����ʽ��ȡIO
#define KEY0        HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)  //KEY0����PE4
#define KEY1        HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3) //KEY1����PE3
#define WK_UP       HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)  //WKUP����PA0

#define M_Z_1       HAL_GPIO_ReadPin(M_Z_1_GPIO_Port,M_Z_1_Pin)  //KEY0����PE3
#define M_Z_2       HAL_GPIO_ReadPin(M_Z_2_GPIO_Port,M_Z_2_Pin) //KEY1����PE4
#define M_Z_3       HAL_GPIO_ReadPin(M_Z_3_GPIO_Port,M_Z_3_Pin)  //WKUP����PA0

#define KEY0_PRES 	1
#define KEY1_PRES 	2
#define WKUP_PRES   3

void KEY_Init(void);
unsigned char KEY_Scan(unsigned char mode);
#endif

