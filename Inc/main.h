/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <cmath>
#include "CAN_DZ.h"
	#include "data.h"
#include <string.h>  //头文件
extern	uint16_t speed4,speed5,speed6;
extern uint8_t data[8];
extern struct ZUOBIAO PM;
extern struct test c;
extern struct JIAODU PN;
extern short Real_Current_Value[4];
extern short Real_Velocity_Value[4];
extern long Real_Position_Value[4];
extern char Real_Online[4];
extern char Real_Ctl1_Value[4];
extern char Real_Ctl2_Value[4];
extern long  dif_qc1 , dif_qc2 , dif_qc3 ;
extern int x, y, z, fl , gravity1 , gravity2 , gravity3 , rst , flag , status ;
extern  long Origin_qc1, Origin_qc2, Origin_qc3; //两次编码器差值计算
extern unsigned char len ;
extern long  dif_qc1 , dif_qc2 , dif_qc3 ;
extern long real_angle1 , real_angle2 , real_angle3 , dif_angle1 , dif_angle2 , dif_angle3 ; //推导真实编码器角度
extern long rst_qc1, rst_qc2, rst_qc3; //复位编码器qc
extern double  man[7];
extern uint8_t Rxd_Data[8];//can中断接收数组（全局变量）
extern uint16_t encoder_4,encoder_5,encoder_6;
extern uint16_t angle_4,angle_5,angle_6;
extern uint32_t ID;
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#include <stdio.h>
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY1_Pin GPIO_PIN_3
#define KEY1_GPIO_Port GPIOE
#define KEY0_Pin GPIO_PIN_4
#define KEY0_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_5
#define LED1_GPIO_Port GPIOE
#define WK_UP_Pin GPIO_PIN_0
#define WK_UP_GPIO_Port GPIOA
#define M_Z_1_Pin GPIO_PIN_11
#define M_Z_1_GPIO_Port GPIOE
#define M_Z_2_Pin GPIO_PIN_13
#define M_Z_2_GPIO_Port GPIOE
#define M_Z_3_Pin GPIO_PIN_15
#define M_Z_3_GPIO_Port GPIOE
#define LED0_Pin GPIO_PIN_5
#define LED0_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
