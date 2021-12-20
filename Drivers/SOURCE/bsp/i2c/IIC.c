#include "iic.h"
#include "main.h"
/**
  ******************************************************************************
  * �ļ�����: bsp_eeprom.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2015-10-04
  * ��    ��: ���ģ��I2Cͨ��ʱ��
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F1Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: I2C����λ�ӳ٣����400KHz
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
static void I2C_Delay(void)
{
	uint8_t i;

	/*��
	 	�����ʱ����ͨ���߼������ǲ��Եõ��ġ�
		CPU��Ƶ72MHzʱ�����ڲ�Flash����, MDK���̲��Ż�
		ѭ������Ϊ10ʱ��SCLƵ�� = 205KHz 
		ѭ������Ϊ7ʱ��SCLƵ�� = 347KHz�� SCL�ߵ�ƽʱ��1.5us��SCL�͵�ƽʱ��2.87us 
	 	ѭ������Ϊ5ʱ��SCLƵ�� = 421KHz�� SCL�ߵ�ƽʱ��1.25us��SCL�͵�ƽʱ��2.375us 
        
    IAR���̱���Ч�ʸߣ���������Ϊ7
	*/
	for (i = 0; i < 10; i++);
}

/**
  * ��������: CPU����I2C���������ź�
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void I2C_Start(void)
{
	/* ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C���������ź� */
	I2C_SDA_HIGH();
	I2C_SCL_HIGH();
	I2C_Delay();
	I2C_SDA_LOW();
	I2C_Delay();
	I2C_SCL_LOW();
	I2C_Delay();
}

/**
  * ��������: CPU����I2C����ֹͣ�ź�
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void I2C_Stop(void)
{
	/* ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C����ֹͣ�ź� */
	I2C_SDA_LOW();
	I2C_SCL_HIGH();
	I2C_Delay();
	I2C_SDA_HIGH();
}

/**
  * ��������: CPU��I2C�����豸����8bit����
  * �������: Byte �� �ȴ����͵��ֽ�
  * �� �� ֵ: ��
  * ˵    ������
  */
void I2C_SendByte(uint8_t Byte)
{
	uint8_t i;

	/* �ȷ����ֽڵĸ�λbit7 */
	for (i = 0; i < 8; i++)
	{		
		if (Byte & 0x80)
		{
			I2C_SDA_HIGH();
		}
		else
		{
			I2C_SDA_LOW();
		}
		I2C_Delay();
		I2C_SCL_HIGH();
		I2C_Delay();	
		I2C_SCL_LOW();
		if (i == 7)
		{
			I2C_SDA_HIGH(); // �ͷ�����
		}
		Byte <<= 1;	/* ����һ��bit */
		I2C_Delay();
	}
}


/**
  * ��������: CPU��I2C�����豸��ȡ8bit����
  * �������: ��
  * �� �� ֵ: ����������
  * ˵    ������
  */
uint8_t I2C_ReadByte(uint8_t ack)
{
	uint8_t i;
	uint8_t value;

	/* ������1��bitΪ���ݵ�bit7 */
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		I2C_SCL_HIGH();
		I2C_Delay();
		if (I2C_SDA_READ())
		{
			value++;
		}
		I2C_SCL_LOW();
		I2C_Delay();
	}
  if(ack==0)
		I2C_NAck();
	else
		I2C_Ack();
	return value;
}

/**
  * ��������: CPU����һ��ʱ�ӣ�����ȡ������ACKӦ���ź�
  * �������: ��
  * �� �� ֵ: ����0��ʾ��ȷӦ��1��ʾ��������Ӧ
  * ˵    ������
  */
uint8_t I2C_WaitAck(void)
{
	uint8_t re;

	I2C_SDA_HIGH();	/* CPU�ͷ�SDA���� */
	I2C_Delay();
	I2C_SCL_HIGH();	/* CPU����SCL = 1, ��ʱ�����᷵��ACKӦ�� */
	I2C_Delay();
	if (I2C_SDA_READ())	/* CPU��ȡSDA����״̬ */
	{
		re = 1;
	}
	else
	{
		re = 0;
	}
	I2C_SCL_LOW();
	I2C_Delay();
	return re;
}

/**
  * ��������: CPU����һ��ACK�ź�
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void I2C_Ack(void)
{
	I2C_SDA_LOW();	/* CPU����SDA = 0 */
	I2C_Delay();
	I2C_SCL_HIGH();	/* CPU����1��ʱ�� */
	I2C_Delay();
	I2C_SCL_LOW();
	I2C_Delay();
	I2C_SDA_HIGH();	/* CPU�ͷ�SDA���� */
}

/**
  * ��������: CPU����1��NACK�ź�
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void I2C_NAck(void)
{
	I2C_SDA_HIGH();	/* CPU����SDA = 1 */
	I2C_Delay();
	I2C_SCL_HIGH();	/* CPU����1��ʱ�� */
	I2C_Delay();
	I2C_SCL_LOW();
	I2C_Delay();	
}

/**
  * ��������: ����I2C���ߵ�GPIO������ģ��IO�ķ�ʽʵ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void I2C_InitGPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* ��GPIOʱ�� */
	I2C_GPIO_CLK_ENABLE();

  GPIO_InitStruct.Pin = I2C_SCL_PIN|I2C_SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);

  /* ��һ��ֹͣ�ź�, ��λI2C�����ϵ������豸������ģʽ */
  I2C_Stop();
}

/**
  * ��������: ���I2C�����豸��CPU�����豸��ַ��Ȼ���ȡ�豸Ӧ�����жϸ��豸�Ƿ����
  * �������: _Address���豸��I2C���ߵ�ַ
  * �� �� ֵ: ����ֵ 0 ��ʾ��ȷ�� ����1��ʾδ̽�⵽
  * ˵    �����ڷ���I2C�豸ǰ�����ȵ��� I2C_CheckDevice() ���I2C�豸�Ƿ��������ú���������GPIO
  */
uint8_t I2C_CheckDevice(uint8_t _Address)
{
	uint8_t ucAck;

	I2C_InitGPIO();		/* ����GPIO */	
	I2C_Start();		/* ���������ź� */
	/* �����豸��ַ+��д����bit��0 = w�� 1 = r) bit7 �ȴ� */
	I2C_SendByte(_Address | I2C_WR);
	ucAck = I2C_WaitAck();	/* ����豸��ACKӦ�� */
	I2C_Stop();			/* ����ֹͣ�ź� */
	return ucAck;
}


///******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/

//float K1 =0.02; 
//float  angle_dot, angle;	
//float Q_angle=0.001;// ����������Э����
//float Q_gyro=0.003;//0.003 ����������Э���� ����������Э����Ϊһ��һ�����о���
//float R_angle=0.5;// ����������Э���� �Ȳ���ƫ��
//float dt=0.005;//                 
//char  C_0 = 1;
//float Q_bias, Angle_err;
//float PCt_0, PCt_1, E;
//float K_0, K_1, t_0, t_1;
//float Pdot[4] ={0,0,0,0};
//float PP[2][2] = { { 1, 0 },{ 0, 1 } };

///**************************************************************************
//�������ܣ����׿������˲�
//��ڲ��������ٶȡ����ٶ�
//����  ֵ����
//**************************************************************************/
//float Kalman_Filter(float Accel,float Gyro)		
//{
//	angle+=(Gyro - Q_bias) * dt; //�������
//	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��

//	Pdot[1]=-PP[1][1];
//	Pdot[2]=-PP[1][1];
//	Pdot[3]=Q_gyro;
//	PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
//	PP[0][1] += Pdot[1] * dt;   // =����������Э����
//	PP[1][0] += Pdot[2] * dt;
//	PP[1][1] += Pdot[3] * dt;
//		
//	Angle_err = Accel - angle;	//zk-�������
//	
//	PCt_0 = C_0 * PP[0][0];
//	PCt_1 = C_0 * PP[1][0];
//	
//	E = R_angle + C_0 * PCt_0;
//	
//	K_0 = PCt_0 / E;
//	K_1 = PCt_1 / E;
//	
//	t_0 = PCt_0;
//	t_1 = C_0 * PP[0][1];

//	PP[0][0] -= K_0 * t_0;		 //����������Э����
//	PP[0][1] -= K_0 * t_1;
//	PP[1][0] -= K_1 * t_0;
//	PP[1][1] -= K_1 * t_1;
//		
//	angle	+= K_0 * Angle_err;	 //�������
//	Q_bias	+= K_1 * Angle_err;	 //�������
//	angle_dot   = Gyro - Q_bias;	 //���ֵ(�������)��΢��=���ٶ�
//	return angle;
//}

   float norm;
   float vx, vy, vz;
   float ex, ey, ez;  
	 float q0 = 1, q1 = 0, q2 = 0, q3 = 0;         
   float exInt = 0, eyInt = 0, ezInt = 0;        
   float g = 9.8;
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
  
	
    norm = sqrt(ax*ax + ay*ay + az*az);  
    ax = ax / norm;           
    ay = ay / norm;
    az = az / norm; 
	
	
    vx = 2*(q1*q3 - q0*q2);
    vy = 2*(q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
	
	
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);
    exInt = exInt + ex*Ki;
    eyInt = eyInt + ey*Ki;
    ezInt = ezInt + ez*Ki;
    gx = gx + Kp*ex + exInt;
    gy = gy + Kp*ey + eyInt;
    gz = gz + Kp*ez + ezInt;
	
	
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;
    pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; 
    roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;
    yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;
}

