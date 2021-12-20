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
    Can_TxMessage(0, id+0x140, 8, (uint8_t *)data); //标准帧发送
   
}

void  Write_PID_Date_RAM(int32_t id,uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi)
{
    data[0] = 0x31;
    data[1] = 0x00;
    data[2] = anglePidKp;
    data[3] = anglePidKi ;
    data[4] = speedPidKp;
    data[5] = speedPidKi;
    data[6] = iqPidKp; //转矩环
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
    data[6] = iqPidKp; //转矩环
    data[7] = iqPidKi;
    Can_TxMessage(0, id+0x140, 8, (uint8_t *)data);

}
/****************加速度不常用*******************/
//void  Read_ACC_Date()//读取加速度命令
//{




//}


//void  Write_ACC_Date()	//写入加速度到 RAM 命令。加速度数据 Accel 为 int32_t 类型，单 位 1dps/s
//{




//}
/****************加速度不常用*******************/
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
void Read_Angle_Cmd(int32_t id)//读取单圈角度命以编码器零点为起始点，顺时针增加，再次到 达零点时数值回 0，单位 0.01°/LSB，数值范围 0~35999。
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
电机在收到命令后回复主机，该帧数据中包含了以下参数。
1.电机温度temperature (int8_t类型，1℃/LSB）。
2.电机的转矩电流值iq (int16_t类型，范围-2048~2048，对应实际转矩电流范围-33A~33A）。
3.电机转速speed(int16_t类型，1dps/LSB）。
4.编码器位置值encoder (uint16_t类型，14bit编码器的数值范围0~16383)
*/
void Read_Temp_Electricity_Speed_encoder(int32_t id)//该命令读取当前电机的温度、电压、转速、编码器位置。
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

void Close_Clear_Cmd(int32_t id)//关闭电机，同时清除电机运行状态和之前接收的控制指令
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
void Stop_Cmd(int32_t id)//停止电机，但不清除电机运行状态和之前接收的控制指令
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
void Run_Cmd(int32_t id)//从电机停止命令中恢复电机运行（恢复停止前的控制方式
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
void LiJu_Contrl(int32_t id,int16_t iqControl)//转矩闭环控制命令，控制值 iqControl 为 int16_t 类型，数值范围-2000~ 2000， 对应实际转矩电流范围-32A~32A
{
    data[0] = 0xA1;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = iqControl & 0x00ff; //低8
    data[5] = iqControl >> 8;
    data[6] = 0x00;
    data[7] = 0x00;
    Can_TxMessage(0, id+0x140, 8, (uint8_t *)data);
}
void Speed_Contrl(int32_t id,int32_t speed)//速度闭环控制命令值speedControl为int32_t类型，对应实际转速为0.01dps/LSB
{
    data[0] = 0xA2;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = speed & 0x00ff; //低8
    data[5] = (speed >> 8) & 0x00ff;
    data[6] = (speed >> 16) & 0x00ff;
    data[7] = speed >> 24;
    Can_TxMessage(0, id+0x140, 8, (uint8_t *)data);

}

void WeiZhi_Contrl(int32_t id,uint8_t spinDirection, uint16_t maxSpeed, uint16_t angleControl )//位置闭环控制命令 ，，控制值 angleControl 为 uint16_t 类型，数值范围 0~35999，对应实际位置为 0.01degree/LSB，即实际角度范围 0°~359.99°。
{
    data[0] = 0xA6;
    data[1] = spinDirection;//0x00 顺 0x01逆
    data[2] = maxSpeed & 0x00ff;
    data[3] = maxSpeed >> 8;
    data[4] = angleControl & 0xff; //低8
    data[5] = angleControl >> 8;
    data[6] = 0x00;
    data[7] = 0x00;
   Can_TxMessage(0, id+0x140, 8, (uint8_t *)data);
}


