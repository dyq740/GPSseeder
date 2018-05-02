#include "gps.h"
#include "delay.h"
#include "usart2.h"
#include "led.h" 
#include "pwm.h"
/////////////////////////////
/*
无人机交互接口
玉米变速恒株距播种控制系统
主函数 main
修改日期：2017/9/21

*/
/////////////////////////////
nmea_msg gpsx; //GPS信息
u16 rxlen;
float tp=0.0;
u16 pwmval=0;
int main(void)
 { 
	delay_init();	    	 //延时函数初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级	
	USART3_Init(19200);  //初始化串口2波特率为19200
	LED_Init();         	//LED初始化	
  TIM1_PWM_Init(45000,0);	 //不分频。PWM频率=72000/(45000+1)=1.6Khz
 // TIM_SetCompare1(TIM1,20000);
	while(1) 
	{		
		if(USART3_RX_STA&0X8000)		//接收到一次数据了
		{
			LED1=~LED1;
			GPS_Analysis(&gpsx,(char*)USART3_RX_BUF);//分析字符串				
		tp = 3.704*gpsx.speed;//获取速度m/h 株距25cm
	//tp = 5.144*gpsx.speed;//获取速度m/h 株距18cm
	//	tp = 4.209*gpsx.speed;//获取速度m/h 株距22cm
//			
		  pwmval=(int)(tp); 
		  TIM_SetCompare1(TIM1,pwmval);	
      USART3_RX_STA=0;		   	//启动下一次接收			
		}
	}
}



