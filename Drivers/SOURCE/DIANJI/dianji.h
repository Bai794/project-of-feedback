#ifndef _DIANJI_H
#define _DIANJI_H
#include "main.h"
#include "can.h"

void  Read_PID_Date(int32_t id);
void  Write_PID_Date_RAM(int32_t id,uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi);
void  Write_PID_Date_ROM(int32_t id,uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi);
void Read_BianMa_Date(int32_t id);
void Read_Angle_Cmd(int32_t id);
void Close_Clear_Cmd(int32_t id);
void Stop_Cmd(int32_t id);
void Run_Cmd(int32_t id);
void LiJu_Contrl(int32_t id ,int16_t iqControl);//ת�رջ���������
void Speed_Contrl(int32_t id,int32_t speed);
void WeiZhi_Contrl(int32_t id,uint8_t spinDirection, uint16_t maxSpeed, uint16_t angleControl );
void Read_Temp_Electricity_Speed_encoder(int32_t id);//�������ȡ��ǰ������¶ȡ���ѹ��ת�١�������λ�á�




#endif



