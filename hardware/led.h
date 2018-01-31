#ifndef _LED_H
#define _LED_H
#include "sys.h"
/////////////////////////////
/*
玉米变速恒株距播种控制系统
LED灯驱动代码
修改日期：2017/9/21
*/
/////////////////////////////
#define LED1 PCout(1)//PC1
void LED_Init(void);//初始化LED灯
#endif
