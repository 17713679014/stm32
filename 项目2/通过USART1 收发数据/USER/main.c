/*************************************
 * 文件名  ：main.c
 * 描述    ：串口1(USART1)向电脑的超级终端以1s为间隔打印当前ADC1的转换电压值         
 * 实验平台：MINI STM32开发板 基于STM32F103C8T6
 * 库版本  ：ST3.0.0

**********************************************************************************/

#include "stm32f10x.h"
#include "usart1.h"
#include "adc.h"       
#include "led.h"
#include "Time_test.h"
extern __IO u16 ADC_ConvertedValue;	 
// 软件延时
/*********************语音触发数据*******************/
char yuyin1[]={0xAA,0x07,0x02,0x00,0x01,0xB4,'\0'};//药物错误 停止服药
char yuyin2[]={0xAA,0x07,0x02,0x00,0x02,0xB5,'\0'};//吃药时间提醒
char yuyin3[]={0xAA,0x07,0x02,0x00,0x03,0xB6,'\0'};//正在出药
char yuyin4[]={0xAA,0x07,0x02,0x00,0x04,0xB7,'\0'};//出药成功
char yuyin5[]={0xAA,0x07,0x02,0x00,0x05,0xB8,'\0'};//kaiji
char yuyin6[]={0xAA,0x07,0x02,0x00,0x06,0xB9,'\0'};//药物查询
char yuyin7[]={0xAA,0x07,0x02,0x00,0x07,0xBA,'\0'};//取药

void Delay(unsigned long time)
{unsigned long i,j;
  
	for(j=0; j<time; j++)
	{
	   for(i=0;i<12000;i++);
	}
}

volatile u32 time; // ms 计时变量
volatile int alarm_mark;//报警变量
volatile int yunxu_mark;//允许变量
volatile int yunxu_mark;//允许变量
int alarm_count;
int main(void)
{ 
  u32 AD_value;	  
  /* 配置系统时钟为72M */      
  SystemInit();	
  /* 配置串口 */
  USART1_Config();
	USART2_Config();
  /* 初始化ADC1 */
  ADC1_Init();
	TIM2_NVIC_Configuration(); /* TIM2 定时配置 */
	TIM2_Configuration(); 
	LED_GPIO_Config();

  printf("\r\n -------这是一个ADC实验------\r\n");
  printf("\r\n -------ADC采样的是PA0口------\r\n");
	START_TIME;	 /* TIM2 开始计时 */
//	LED1( ON );			  // 亮
//	Delay(0x200000);
//	LED1( OFF );		  // 灭
//	Delay(0x200000);
//	SendStringe(yuyin2,7);
  while (1)
  {
	  /*******0.5s采集一次数据***************/ 
	  if ( time == 2000 ) /* 500ms 时间到 */
    {
      time = 0;		
			AD_value  = 3300000/4096*ADC_ConvertedValue/1000;		
			printf("烟雾浓度 = %d   \r\n", AD_value);				
    } 
		/*******浓度值超限报警*******/ 
	  if(AD_value > 200)
		{
			/*******防止报警频率过高*******/ 
			if(alarm_mark ==0)//报警一次后  4s后报警
			{
			  SendStringe(yuyin1,7);
				alarm_mark = 1;
			}
		}
  }
}
/*******中断处理函数*******/ 
void TIM2_IRQHandler(void)
{
	if ( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET ) 
	{	
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);    
  		 time++;
		alarm_count++;
		if(alarm_count == 4000)
		{
			alarm_count = 0;
			alarm_mark = 0;
		}
	}		 	
}



