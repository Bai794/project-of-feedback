#include "dianji.h"

/**
*****************************************
*author:xiaobai
* tools:keil5
*ic:stm32f103c8t6
*function: 
*****************************************
*/
void  Read_PID_Date(int32_t id)
{
    data[0] = 0x30;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    Can_TxMessage(0, id+0x140, 8, (uint8_t *)data); //��׼֡����
   
}

void  Write_PID_Date_RAM(int32_t id,uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi)
{
    data[0] = 0x31;
    data[1] = 0x00;
    data[2] = anglePidKp;
    data[3] = anglePidKi ;
    data[4] = speedPidKp;
    data[5] = speedPidKi;
    data[6] = iqPidKp; //ת�ػ�
    data[7] = iqPidKi;
    Can_TxMessage(0, id+0x140, 8, (uint8_t *)data);
}

void  Write_PID_Date_ROM(int32_t id,uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi)
{
    data[0] = 0x32;
    data[1] = 0x00;
    data[2] = anglePidKp;
    data[3] = anglePidKi ;
    data[4] = speedPidKp;
    data[5] = speedPidKi;
    data[6] = iqPidKp; //ת�ػ�
    data[7] = iqPidKi;
    Can_TxMessage(0, id+0x140, 8, (uint8_t *)data);

}
/****************���ٶȲ�����*******************/
//void  Read_ACC_Date()//��ȡ���ٶ�����
//{




//}


//void  Write_ACC_Date()	//д����ٶȵ� RAM ������ٶ����� Accel Ϊ int32_t ���ͣ��� λ 1dps/s
//{




//}
/****************���ٶȲ�����*******************/
void Read_BianMa_Date(int32_t id)// 
{
    data[0] = 0x90;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    Can_TxMessage(0, id+0x140, 8, (uint8_t *)data);
}
void Read_Angle_Cmd(int32_t id)//��ȡ��Ȧ�Ƕ����Ա��������Ϊ��ʼ�㣬˳ʱ�����ӣ��ٴε� �����ʱ��ֵ�� 0����λ 0.01��/LSB����ֵ��Χ 0~35999��
{
    data[0] = 0x94;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    Can_TxMessage(0, id+0x140, 8, (uint8_t *)data);
}


/*
������յ������ظ���������֡�����а��������²�����
1.����¶�temperature (int8_t���ͣ�1��/LSB����
2.�����ת�ص���ֵiq (int16_t���ͣ���Χ-2048~2048����Ӧʵ��ת�ص�����Χ-33A~33A����
3.���ת��speed(int16_t���ͣ�1dps/LSB����
4.������λ��ֵencoder (uint16_t���ͣ�14bit����������ֵ��Χ0~16383)
*/
void Read_Temp_Electricity_Speed_encoder(int32_t id)//�������ȡ��ǰ������¶ȡ���ѹ��ת�١�������λ�á�
{
    data[0] = 0x9C;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    Can_TxMessage(0, id+0x140, 8, (uint8_t *)data);
}

void Close_Clear_Cmd(int32_t id)//�رյ����ͬʱ����������״̬��֮ǰ���յĿ���ָ��
{
    data[0] = 0x80;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    Can_TxMessage(0, id+0x140, 8, (uint8_t *)data);

}
void Stop_Cmd(int32_t id)//ֹͣ�������������������״̬��֮ǰ���յĿ���ָ��
{
    data[0] = 0x81;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    Can_TxMessage(0, id+0x140, 8, (uint8_t *)data);
}
void Run_Cmd(int32_t id)//�ӵ��ֹͣ�����лָ�������У��ָ�ֹͣǰ�Ŀ��Ʒ�ʽ
{
    data[0] = 0x88;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
     Can_TxMessage(0, id+0x140, 8, (uint8_t *)data);
}
void LiJu_Contrl(int32_t id,int16_t iqControl)//ת�رջ������������ֵ iqControl Ϊ int16_t ���ͣ���ֵ��Χ-2000~ 2000�� ��Ӧʵ��ת�ص�����Χ-32A~32A
{
    data[0] = 0xA1;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = iqControl & 0x00ff; //��8
    data[5] = iqControl >> 8;
    data[6] = 0x00;
    data[7] = 0x00;
    Can_TxMessage(0, id+0x140, 8, (uint8_t *)data);
}
void Speed_Contrl(int32_t id,int32_t speed)//�ٶȱջ���������ֵspeedControlΪint32_t���ͣ���Ӧʵ��ת��Ϊ0.01dps/LSB
{
    data[0] = 0xA2;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = speed & 0x00ff; //��8
    data[5] = (speed >> 8) & 0x00ff;
    data[6] = (speed >> 16) & 0x00ff;
    data[7] = speed >> 24;
    Can_TxMessage(0, id+0x140, 8, (uint8_t *)data);

}

void WeiZhi_Contrl(int32_t id,uint8_t spinDirection, uint16_t maxSpeed, uint16_t angleControl )//λ�ñջ��������� ��������ֵ angleControl Ϊ uint16_t ���ͣ���ֵ��Χ 0~35999����Ӧʵ��λ��Ϊ 0.01degree/LSB����ʵ�ʽǶȷ�Χ 0��~359.99�㡣
{
    data[0] = 0xA6;
    data[1] = spinDirection;//0x00 ˳ 0x01��
    data[2] = maxSpeed & 0x00ff;
    data[3] = maxSpeed >> 8;
    data[4] = angleControl & 0xff; //��8
    data[5] = angleControl >> 8;
    data[6] = 0x00;
    data[7] = 0x00;
   Can_TxMessage(0, id+0x140, 8, (uint8_t *)data);
}


