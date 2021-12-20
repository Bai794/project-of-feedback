#ifndef __IIC_H__
#define	__IIC_H__
/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include <inttypes.h>

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
#define I2C_OWN_ADDRESS                            0x0A
#define Kp 100.0f                     
#define Ki 0.002f              
#define halfT 0.001f 
#define I2C_WR	        0		/* 写控制bit */
#define I2C_RD	        1		/* 读控制bit */

#define I2C_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2C_GPIO_PORT                       GPIOB   
#define I2C_SCL_PIN                         GPIO_PIN_12
#define I2C_SDA_PIN                         GPIO_PIN_13

#define I2C_SCL_HIGH()                      HAL_GPIO_WritePin(I2C_GPIO_PORT,I2C_SCL_PIN,GPIO_PIN_SET)    // 输出高电平
#define I2C_SCL_LOW()                       HAL_GPIO_WritePin(I2C_GPIO_PORT,I2C_SCL_PIN,GPIO_PIN_RESET)  // 输出低电平
#define I2C_SDA_HIGH()                      HAL_GPIO_WritePin(I2C_GPIO_PORT,I2C_SDA_PIN,GPIO_PIN_SET)    // 输出高电平
#define I2C_SDA_LOW()                       HAL_GPIO_WritePin(I2C_GPIO_PORT,I2C_SDA_PIN,GPIO_PIN_RESET)  // 输出低电平
#define I2C_SDA_READ()                      HAL_GPIO_ReadPin(I2C_GPIO_PORT,I2C_SDA_PIN)
extern float angle;
extern  float g;
/* 扩展变量 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/
void    I2C_Start(void);
void    I2C_Stop(void);
void    I2C_SendByte(uint8_t _ucByte);
uint8_t I2C_ReadByte(uint8_t ack);
uint8_t I2C_WaitAck(void);
void    I2C_Ack(void);
void    I2C_NAck(void);
uint8_t I2C_CheckDevice(uint8_t _Address);
void I2C_InitGPIO(void);
float Kalman_Filter(float Accel,float Gyro);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);

#endif /* __I2C_EEPROM_H__ */

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
