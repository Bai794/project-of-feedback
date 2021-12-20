#include "data.h"
#include "stdio.h"
#include "main.h"
#include "math.h"
#include "CAN_DZ.h"
#include "ztjs.h"
struct ZUOBIAO PM;
struct JIAODU PN;
// int x, y, z, fl = 100, gravity1 = 400, gravity2 = 410, gravity3 = 300, rst = 0, flag = 0, status = 0;
// long Origin_qc1, Origin_qc2, Origin_qc3; //两次编码器差值计算
////unsigned char len = 0;
////long  dif_qc1 = 0, dif_qc2 = 0, dif_qc3 = 0;
// float real_angle1 = 0, real_angle2 = 0, real_angle3 = 0, dif_angle1 = 0, dif_angle2 = 0, dif_angle3 = 0; //推导真实编码器角度
// long rst_qc1, rst_qc2, rst_qc3; //复位编码器qc
// double  man[7];

// extern struct
//{
//	 float x0;
//	 float y0;
//	 float z0;
// }PM;
// extern struct
//{
//	 float theta0;
//	 float theta1;
//	 float theta2;
//	 float theta3;
// }PN;

// robot geometry
// (look at pics above for explanation)
const float e = 242.5; // end effector
const float f = 502.3; // base
const float re = 180.0;
const float rf = 105.48;
// trigonometric constants

const float sqrt3 = 1.73205; // sqrt(3.0);

const float pi = 3.141592653; // PI
const float sin120 = sqrt3 / 2.0;
const float cos120 = -0.5;
const float tan60 = sqrt3;
const float sin30 = 0.5;
const float tan30 = 1 / sqrt3;

// forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
// returned status: 0=OK, -1=non-existing position
int delta_calcForward(float theta1, float theta2, float theta3)
{
    float t, dtr, y1, z1, x2, y2, z2, x3, y3, z3, w1, w2, w3;
    float a, b, c, d, a1, b1, a2, b2, dnm;
    t = (f - e) * tan30 / 2;
    dtr = pi / (float)180.0;

    theta1 *= dtr;
    theta2 *= dtr;
    theta3 *= dtr;

    y1 = -(t + rf * cos(theta1));
    z1 = -rf * sin(theta1);

    y2 = (t + rf * cos(theta2)) * sin30;
    x2 = y2 * tan60;
    z2 = -rf * sin(theta2);

    y3 = (t + rf * cos(theta3)) * sin30;
    x3 = -y3 * tan60;
    z3 = -rf * sin(theta3);

    dnm = (y2 - y1) * x3 - (y3 - y1) * x2;

    w1 = y1 * y1 + z1 * z1;
    w2 = x2 * x2 + y2 * y2 + z2 * z2;
    w3 = x3 * x3 + y3 * y3 + z3 * z3;

    // x = (a1*z + b1)/dnm
    a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
    b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0;

    // y = (a2*z + b2)/dnm;
    a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
    b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0;

    // a*z^2 + b*z + c = 0
    a = a1 * a1 + a2 * a2 + dnm * dnm;
    b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm);
    c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - re * re);

    // discriminant
    d = b * b - (float)4.0 * a * c;
    if (d < 0)
        return -1; // non-existing point

    PM.z0 = (float)0.5 * (b + sqrt(d)) / a;
    PM.y0 = (a1 * PM.z0 + b1) / dnm;
    PM.x0 = (a2 * PM.z0 + b2) / dnm;
    return 0;
}

// inverse kinematics
// helper functions, calculates angle theta1 (for YZ-pane)
int delta_calcAngleYZ(float x0, float y0, float z0)

{
    float y1, a, b, d, yj, zj;
    y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
    y0 -= 0.5 * 0.57735 * e; // shift center to edge
    // z = a + b*y
    a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2 * z0);
    b = (y1 - y0) / z0;
    // discriminant
    d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf);
    if (d < 0)
        return -1;                             // non-existing point
    yj = (y1 - a * b - sqrt(d)) / (b * b + 1); // choosing outer point
    zj = a + b * yj;
    PN.theta0 = 180.0 * atan(-zj / (y1 - yj)) / pi + ((yj > y1) ? 180.0 : 0.0);
    return 0;
}

// inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
// returned status: 0=OK, -1=non-existing position
int delta_calcInverse(float x0, float y0, float z0)
{
    int status;
    PN.theta1 = PN.theta2 = PN.theta3 = 0;
    status = delta_calcAngleYZ(x0, y0, z0);
    PN.theta1 = PN.theta0;
    if (status == 0)
        status = delta_calcAngleYZ(x0 * cos120 + y0 * sin120, y0 * cos120 - x0 * sin120, z0);
    // rotate coords to +120 deg
    PN.theta2 = PN.theta0;
    if (status == 0)
        status = delta_calcAngleYZ(x0 * cos120 - y0 * sin120, y0 * cos120 + x0 * sin120, z0);
    // rotate coords to -120 deg
    PN.theta3 = PN.theta0;
    return status;
}

int str[8] = {0};
void DATE(void) //协议里关于数据类型转换后面可以做优化
{

    str[0] = 0x0a;
    str[1] = (int)((PM.x0 + X) * 1000000) + x * 1000;
    str[2] = (int)((PM.y0 + Y) * 1000000) + y * 1000;
    str[3] = (int)((PM.z0 + Z) * 1000000) + x * 1000;
    // str[4]=PM.rx;
    // str[5]=PM.ry;
    // str[6]=PM.rz;
    // str[7]=extra
    str[4] = 0x0d;
    /* str[0]=0x0a;
         str[1]=1.1;
         str[2]=2.2;
       str[3]=3.3;
         str[4]=4.4;
         str[5]=5.5;
         str[6]=6.6;
         str[7]=0x0d;
    */
}

void run()
{
    //得到差值并保存
    dif_qc1 = Real_Position_Value[0] - Origin_qc1;
    dif_qc2 = Real_Position_Value[1] - Origin_qc2;
    dif_qc3 = Real_Position_Value[2] - Origin_qc3;
    //得到真实角度差
    dif_angle1 = dif_qc1 * 0.18;
    dif_angle2 = dif_qc2 * 0.18;
    dif_angle3 = dif_qc3 * 0.18;
    //计算真实角度
    real_angle1 = dif_angle1 / 13.3;
    real_angle2 = dif_angle2 / 13.3;
    real_angle3 = dif_angle3 / 13.3;
    //解算
    delta_calcForward(real_angle2 + 31.4, real_angle3 + 31.4, real_angle1 + 31.4); //以theta1为y轴
                                                                                   // delta_calcInverse(PM.x0,PM.y0,PM.z0);//运动学逆解
}
void map() //位置和电流经验映射关系
{
    gravity1 = 420 - (real_angle1 - 32.4) * (real_angle1 - 32.4) * 0.005;
    gravity2 = 430 - (real_angle2 - 32.5) * (real_angle2 - 32.5) * 0.005;
    gravity3 = 325 - (real_angle3 - 29.9) * (real_angle3 - 29.9) * 0.005;
}
