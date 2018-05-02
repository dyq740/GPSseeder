#include "gps.h"
#include "delay.h"
#include "usart2.h"
#include "led.h" 
#include "pwm.h"
/////////////////////////////
/*
���˻������ӿ�
���ױ��ٺ���ಥ�ֿ���ϵͳ
������ main
�޸����ڣ�2017/9/21

*/
/////////////////////////////
nmea_msg gpsx; //GPS��Ϣ
u16 rxlen;
float tp=0.0;
u16 pwmval=0;
int main(void)
 { 
	delay_init();	    	 //��ʱ������ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�	
	USART3_Init(19200);  //��ʼ������2������Ϊ19200
	LED_Init();         	//LED��ʼ��	
  TIM1_PWM_Init(45000,0);	 //����Ƶ��PWMƵ��=72000/(45000+1)=1.6Khz
 // TIM_SetCompare1(TIM1,20000);
	while(1) 
	{		
		if(USART3_RX_STA&0X8000)		//���յ�һ��������
		{
			LED1=~LED1;
			GPS_Analysis(&gpsx,(char*)USART3_RX_BUF);//�����ַ���				
		tp = 3.704*gpsx.speed;//��ȡ�ٶ�m/h ���25cm
	//tp = 5.144*gpsx.speed;//��ȡ�ٶ�m/h ���18cm
	//	tp = 4.209*gpsx.speed;//��ȡ�ٶ�m/h ���22cm
//			
		  pwmval=(int)(tp); 
		  TIM_SetCompare1(TIM1,pwmval);	
      USART3_RX_STA=0;		   	//������һ�ν���			
		}
	}
}



