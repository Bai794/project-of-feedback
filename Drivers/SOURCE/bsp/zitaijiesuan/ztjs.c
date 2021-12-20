/*
 * @Author: HideMe
 * @Date: 2020-10-10 12:56:39
 * @LastEditTime: 2021-12-05 23:03:06
 * @LastEditors: your name
 * @Description:
 * @FilePath: \MDK-ARMd:\stm32 code\feedback\Drivers\SOURCE\bsp\zitaijiesuan\ztjs.c
 * @e-mail: 1269724114@qq.com
 */
#include "ztjs.h"
#include "main.h"
float X, Y, Z;
#define L1 60
#define L2 90
#define L3 60
#define L4 60
#define L5 130

// 电机x 是中间  y是最上面的电机  z是最下面的 角度为正是逆时针旋转
void zitaijiesuan(float x, float y, float z)
{

	int y0 = L5, x0 = 0, z0 = 0, x1 = L4, y1 = -L3, z1 = 0, x2 = -L1, y2 = 0, z2 = L2;
	x += 90;
	x = x * PI / 180;
	y = y * PI / 180;
	z = z * PI / 180;
	X = x0 * (cos(y) * cos(z) - sin(x) * sin(y) * sin(z)) - sin(z) * (y1 * cos(x) - z1 * sin(x)) + z0 * (cos(z) * sin(y) + cos(y) * sin(x) * sin(z)) + x1 * cos(z) + x2 * cos(z) - y2 * sin(z) - y0 * cos(x) * sin(z);
	Y = cos(z) * (y1 * cos(x) - z1 * sin(x)) + x0 * (cos(y) * sin(z) + cos(z) * sin(x) * sin(y)) + z0 * (sin(y) * sin(z) - cos(y) * cos(z) * sin(x)) + y2 * cos(z) + x1 * sin(z) + x2 * sin(z) + y0 * cos(x) * cos(z);
	Z = z2 + y0 * sin(x) + y1 * sin(x) + z1 * sin(x) + z0 * cos(x) * cos(y) - x0 * cos(x) * sin(y);
}
