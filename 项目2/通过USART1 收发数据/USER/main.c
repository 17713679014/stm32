/*************************************
 * �ļ���  ��main.c
 * ����    ������1(USART1)����Եĳ����ն���1sΪ�����ӡ��ǰADC1��ת����ѹֵ         
 * ʵ��ƽ̨��MINI STM32������ ����STM32F103C8T6
 * ��汾  ��ST3.0.0

**********************************************************************************/

#include "stm32f10x.h"
#include "usart1.h"
#include "adc.h"       
#include "led.h"
#include "Time_test.h"
extern __IO u16 ADC_ConvertedValue;	 
// �����ʱ
/*********************������������*******************/
char yuyin1[]={0xAA,0x07,0x02,0x00,0x01,0xB4,'\0'};//ҩ����� ֹͣ��ҩ
char yuyin2[]={0xAA,0x07,0x02,0x00,0x02,0xB5,'\0'};//��ҩʱ������
char yuyin3[]={0xAA,0x07,0x02,0x00,0x03,0xB6,'\0'};//���ڳ�ҩ
char yuyin4[]={0xAA,0x07,0x02,0x00,0x04,0xB7,'\0'};//��ҩ�ɹ�
char yuyin5[]={0xAA,0x07,0x02,0x00,0x05,0xB8,'\0'};//kaiji
char yuyin6[]={0xAA,0x07,0x02,0x00,0x06,0xB9,'\0'};//ҩ���ѯ
char yuyin7[]={0xAA,0x07,0x02,0x00,0x07,0xBA,'\0'};//ȡҩ

void Delay(unsigned long time)
{unsigned long i,j;
  
	for(j=0; j<time; j++)
	{
	   for(i=0;i<12000;i++);
	}
}

volatile u32 time; // ms ��ʱ����
volatile int alarm_mark;//��������
volatile int yunxu_mark;//�������
volatile int yunxu_mark;//�������
int alarm_count;
int main(void)
{ 
  u32 AD_value;	  
  /* ����ϵͳʱ��Ϊ72M */      
  SystemInit();	
  /* ���ô��� */
  USART1_Config();
	USART2_Config();
  /* ��ʼ��ADC1 */
  ADC1_Init();
	TIM2_NVIC_Configuration(); /* TIM2 ��ʱ���� */
	TIM2_Configuration(); 
	LED_GPIO_Config();

  printf("\r\n -------����һ��ADCʵ��------\r\n");
  printf("\r\n -------ADC��������PA0��------\r\n");
	START_TIME;	 /* TIM2 ��ʼ��ʱ */
//	LED1( ON );			  // ��
//	Delay(0x200000);
//	LED1( OFF );		  // ��
//	Delay(0x200000);
//	SendStringe(yuyin2,7);
  while (1)
  {
	  /*******0.5s�ɼ�һ������***************/ 
	  if ( time == 2000 ) /* 500ms ʱ�䵽 */
    {
      time = 0;		
			AD_value  = 3300000/4096*ADC_ConvertedValue/1000;		
			printf("����Ũ�� = %d   \r\n", AD_value);				
    } 
		/*******Ũ��ֵ���ޱ���*******/ 
	  if(AD_value > 200)
		{
			/*******��ֹ����Ƶ�ʹ���*******/ 
			if(alarm_mark ==0)//����һ�κ�  4s�󱨾�
			{
			  SendStringe(yuyin1,7);
				alarm_mark = 1;
			}
		}
  }
}
/*******�жϴ�����*******/ 
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



