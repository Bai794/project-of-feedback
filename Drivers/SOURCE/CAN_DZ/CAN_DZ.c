#include "CAN_DZ.h"
#include "can.h"
#include "main.h"


unsigned int CAN_Time_Out = 0;

static void CAN_Delay_Us(unsigned int t)
{
	int i;
	for(i=0;i<t;i++)
	{
		int a=9;
		while(a--);
	}
}
uint8_t Data[8];
unsigned char can_tx_success_flag=0;

/****************************************************************************************
                                       复位指令
Group   取值范围 0-7
Number  取值范围 0-15，其中Number==0时，为广播发送
*****************************************************************************************/
void CAN_RoboModule_DRV_Reset(unsigned char Group,unsigned char Number)
{
    unsigned short can_id = 0x000;
    CAN_TxHeaderTypeDef tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    Data[0] = 0x55;
    Data[1] = 0x55;
    Data[2] = 0x55;
    Data[3] = 0x55;
    Data[4] = 0x55;
    Data[5] = 0x55;
    Data[6] = 0x55;
    Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    Can_TxMessage(CAN_ID_STD,tx_message.StdId,8,Data);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                     模式选择指令
Group   取值范围 0-7
Number  取值范围 0-15，其中Number==0时，为广播发送

Mode    取值范围

OpenLoop_Mode                       0x01
Current_Mode                        0x02
Velocity_Mode                       0x03
Position_Mode                       0x04
Velocity_Position_Mode              0x05
Current_Velocity_Mode               0x06
Current_Position_Mode               0x07
Current_Velocity_Position_Mode      0x08
*****************************************************************************************/
void CAN_RoboModule_DRV_Mode_Choice(unsigned char Group,unsigned char Number,unsigned char Mode)
{
    unsigned short can_id = 0x001;
    CAN_TxHeaderTypeDef tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    Data[0] = Mode;
    Data[1] = 0x55;
    Data[2] = 0x55;
    Data[3] = 0x55;
    Data[4] = 0x55;
    Data[5] = 0x55;
    Data[6] = 0x55;
    Data[7] = 0x55;
    
    can_tx_success_flag = 0;
   Can_TxMessage(CAN_ID_STD,tx_message.StdId,8,Data);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                   开环模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
-5000 ~ +5000，满值5000，其中temp_pwm = ±5000时，最大输出电压为电源电压

*****************************************************************************************/
void CAN_RoboModule_DRV_OpenLoop_Mode(unsigned char Group,unsigned char Number,short Temp_PWM)
{
    unsigned short can_id = 0x002;
    CAN_TxHeaderTypeDef tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    Data[1] = (unsigned char)(Temp_PWM&0xff);
    Data[2] = 0x55;
    Data[3] = 0x55;
    Data[4] = 0x55;
    Data[5] = 0x55;
    Data[6] = 0x55;
    Data[7] = 0x55;
    
    can_tx_success_flag = 0;
   Can_TxMessage(CAN_ID_STD,tx_message.StdId,8,Data);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                   电流模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_current的取值范围如下：
-32768 ~ +32767，单位mA

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Current)
{
    unsigned short can_id = 0x003;
    CAN_TxHeaderTypeDef tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    Data[1] = (unsigned char)(Temp_PWM&0xff);
    Data[2] = (unsigned char)((Temp_Current>>8)&0xff);
    Data[3] = (unsigned char)(Temp_Current&0xff);
    Data[4] = 0x55;
    Data[5] = 0x55;
    Data[6] = 0x55;
    Data[7] = 0x55;
    
    can_tx_success_flag = 0;
   Can_TxMessage(CAN_ID_STD,tx_message.StdId,8,Data);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                   速度模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_velocity的取值范围如下：
-32768 ~ +32767，单位RPM

*****************************************************************************************/
void CAN_RoboModule_DRV_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity)
{
    unsigned short can_id = 0x004;
    CAN_TxHeaderTypeDef tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    Data[1] = (unsigned char)(Temp_PWM&0xff);
    Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    Data[3] = (unsigned char)(Temp_Velocity&0xff);
    Data[4] = 0x55;
    Data[5] = 0x55;
    Data[6] = 0x55;
    Data[7] = 0x55;
    
    can_tx_success_flag = 0;
   Can_TxMessage(CAN_ID_STD,tx_message.StdId,8,Data);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

///****************************************************************************************
//                                   位置模式下的数据指令
//Group   取值范围 0-7

//Number  取值范围 0-15，其中Number==0时，为广播发送

//temp_pwm的取值范围如下：
//0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

//temp_position的取值范围如下：
//-2147483648~+2147483647，单位qc

//*****************************************************************************************/
//void CAN_RoboModule_DRV_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,long Temp_Position)
//{
//    unsigned short can_id = 0x005;
//    CAN_TxHeaderTypeDef tx_message;
//    
//    tx_message.IDE = CAN_ID_STD;    //标准帧
//    tx_message.RTR = CAN_RTR_DATA;  //数据帧
//    tx_message.DLC = 0x08;          //帧长度为8
//    
//    if((Group<=7)&&(Number<=15))
//    {
//        can_id |= Group<<8;
//        can_id |= Number<<4;
//    }
//    else
//    {
//        return;
//    }
//    
//    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

//    if(Temp_PWM > 5000)
//    {
//        Temp_PWM = 5000;
//    }
//    else if(Temp_PWM < -5000)
//    {
//        Temp_PWM = -5000;
//    }
//    
//    if(Temp_PWM < 0)
//    {
//        Temp_PWM = abs(Temp_PWM);
//    }
//    
//    Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
//    Data[1] = (unsigned char)(Temp_PWM&0xff);
//    Data[2] = 0x55;
//    Data[3] = 0x55;
//    Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
//    Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
//    Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
//    Data[7] = (unsigned char)(Temp_Position&0xff);
//    
//    can_tx_success_flag = 0;
//   Can_TxMessage(CAN_ID_STD,tx_message.StdId,8,Data);
//    
//    CAN_Time_Out = 0;
//    while(can_tx_success_flag == 0)
//    {
//        CAN_Delay_Us(1);
//        CAN_Time_Out++;
//        if(CAN_Time_Out>100)
//        {
//            break;
//        }
//    }
//}
/****************************************************************************************
                                   位置模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc

*****************************************************************************************/
void CAN_RoboModule_DRV_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,long Temp_Position)
{
    unsigned short can_id = 0x005;
    CAN_TxHeaderTypeDef tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    Data[1] = (unsigned char)(Temp_PWM&0xff);
    Data[2] = 0x55;
    Data[3] = 0x55;
    Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    Data[7] = (unsigned char)(Temp_Position&0xff);
    
    can_tx_success_flag = 0;
   Can_TxMessage(CAN_ID_STD,tx_message.StdId,8,Data);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}
/****************************************************************************************
                                  速度位置模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_velocity的取值范围如下：
0 ~ +32767，单位RPM

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc
*****************************************************************************************/
void CAN_RoboModule_DRV_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x006;
    CAN_TxHeaderTypeDef tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    if(Temp_Velocity < 0)
    {
        Temp_Velocity = abs(Temp_Velocity);
    }
    
    Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    Data[1] = (unsigned char)(Temp_PWM&0xff);
    Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    Data[3] = (unsigned char)(Temp_Velocity&0xff);
    Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    Data[7] = (unsigned char)(Temp_Position&0xff);
    
    can_tx_success_flag = 0;
   Can_TxMessage(CAN_ID_STD,tx_message.StdId,8,Data);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}


/****************************************************************************************
                                  电流速度模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_current的取值范围如下：
0 ~ +32767，单位mA

temp_velocity的取值范围如下：
-32768 ~ +32767，单位RPM

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity)
{
    unsigned short can_id = 0x007;
    CAN_TxHeaderTypeDef tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }
    
    Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    Data[1] = (unsigned char)(Temp_Current&0xff);
    Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    Data[3] = (unsigned char)(Temp_Velocity&0xff);
    Data[4] = 0x55;
    Data[5] = 0x55;
    Data[6] = 0x55;
    Data[7] = 0x55;
    
    can_tx_success_flag = 0;
   Can_TxMessage(CAN_ID_STD,tx_message.StdId,8,Data);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}


/****************************************************************************************
                                  电流位置模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_current的取值范围如下：
0 ~ +32767，单位mA

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,long Temp_Position)
{
    unsigned short can_id = 0x008;
    CAN_TxHeaderTypeDef tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    
    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }
    
    Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    Data[1] = (unsigned char)(Temp_Current&0xff);
    Data[2] = 0x55;
    Data[3] = 0x55;
    Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    Data[7] = (unsigned char)(Temp_Position&0xff);
    
    can_tx_success_flag = 0;
   Can_TxMessage(CAN_ID_STD,tx_message.StdId,8,Data);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}


/****************************************************************************************
                                  电流速度位置模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_current的取值范围如下：
0 ~ +32767，单位mA

temp_velocity的取值范围如下：
0 ~ +32767，单位RPM

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x009;
    CAN_TxHeaderTypeDef tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }
    
    if(Temp_Velocity < 0)
    {
        Temp_Velocity = abs(Temp_Velocity);
    }
    
    Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    Data[1] = (unsigned char)(Temp_Current&0xff);
    Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    Data[3] = (unsigned char)(Temp_Velocity&0xff);
    Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    Data[7] = (unsigned char)(Temp_Position&0xff);
    
    can_tx_success_flag = 0;
   Can_TxMessage(CAN_ID_STD,tx_message.StdId,8,Data);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                      配置指令
Temp_Time1的取值范围: 0 ~ 255，为0时候，为关闭电流速度位置反馈功能
Temp_Time2的取值范围: 0 ~ 255，为0时候，为关闭限位信号反馈功能
*****************************************************************************************/
void CAN_RoboModule_DRV_Config(unsigned char Group,unsigned char Number,unsigned char Temp_Time1,unsigned char Temp_Time2)
{
    unsigned short can_id = 0x00A;
    CAN_TxHeaderTypeDef tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;
    
    Data[0] = Temp_Time1;
    Data[1] = Temp_Time2;
    Data[2] = 0x55;
    Data[3] = 0x55;
    Data[4] = 0x55;
    Data[5] = 0x55;
    Data[6] = 0x55;
    Data[7] = 0x55;
    
    can_tx_success_flag = 0;
   Can_TxMessage(CAN_ID_STD,tx_message.StdId,8,Data);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                      在线检测
*****************************************************************************************/
void CAN_RoboModule_DRV_Online_Check(unsigned char Group,unsigned char Number)
{
    unsigned short can_id = 0x00F;
    CAN_TxHeaderTypeDef tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    Data[0] = 0x55;
    Data[1] = 0x55;
    Data[2] = 0x55;
    Data[3] = 0x55;
    Data[4] = 0x55;
    Data[5] = 0x55;
    Data[6] = 0x55;
    Data[7] = 0x55;
    
    can_tx_success_flag = 0;
   Can_TxMessage(CAN_ID_STD,tx_message.StdId,8,Data);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

short Real_Current_Value[4] = {0};
short Real_Velocity_Value[4] = {0};
long Real_Position_Value[4] = {0};
char Real_Online[4] = {0};
char Real_Ctl1_Value[4] = {0};
char Real_Ctl2_Value[4] = {0};

////本接收数据的函数，默认为4个驱动器，都挂在0组，编号为1、2、3、4
///*************************************************************************
//                          USB_LP_CAN1_RX0_IRQHandler
//描述：CAN1的接收中断函数
//*************************************************************************/
//void USB_LP_CAN1_RX0_IRQHandler(void) //CAN RX
//{
//    CAN_RxHeaderTypeDef rx_message;
//    
//    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
//	{
//        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
//        CAN_Receive(CAN1, CAN_FILTER_FIFO0, &rx_message);
//        
//        if((rx_message.IDE == CAN_Id_Standard)&&(rx_message.IDE == CAN_RTR_Data)&&(rx_message.DLC == 8)) //标准帧、数据帧、数据长度为8
//        {
//            if(rx_message.StdId == 0x1B)
//            {
//                Real_Current_Value[0] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
//                Real_Velocity_Value[0] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
//                Real_Position_Value[0] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
//            }
//            else if(rx_message.StdId == 0x2B)
//            {
//                Real_Current_Value[1] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
//                Real_Velocity_Value[1] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
//                Real_Position_Value[1] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
//            }
//            else if(rx_message.StdId == 0x3B)
//            {
//                Real_Current_Value[2] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
//                Real_Velocity_Value[2] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
//                Real_Position_Value[2] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
//            }
//            else if(rx_message.StdId == 0x4B)
//            {
//                Real_Current_Value[3] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
//                Real_Velocity_Value[3] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
//                Real_Position_Value[3] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
//            }
//            else if(rx_message.StdId == 0x1F)
//            {
//                Real_Online[0] = 1;
//            }
//            else if(rx_message.StdId == 0x2F)
//            {
//                Real_Online[1] = 1;
//            }
//            else if(rx_message.StdId == 0x3F)
//            {
//                Real_Online[2] = 1;
//            }
//            else if(rx_message.StdId == 0x4F)
//            {
//                Real_Online[3] = 1;
//            }
//            else if(rx_message.StdId == 0x1C)
//            {
//                Real_Ctl1_Value[0] = rx_message.Data[0];
//                Real_Ctl2_Value[0] = rx_message.Data[1];
//            }
//            else if(rx_message.StdId == 0x2C)
//            {
//                Real_Ctl1_Value[1] = rx_message.Data[0];
//                Real_Ctl2_Value[1] = rx_message.Data[1];
//            }
//            else if(rx_message.StdId == 0x3C)
//            {
//                Real_Ctl1_Value[2] = rx_message.Data[0];
//                Real_Ctl2_Value[2] = rx_message.Data[1];
//            }
//            else if(rx_message.StdId == 0x4C)
//            {
//                Real_Ctl1_Value[3] = rx_message.Data[0];
//                Real_Ctl2_Value[3] = rx_message.Data[1];
//            }

//        }
//                
//    }
//}
