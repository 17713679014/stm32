#include "stm32l4xx_ll_gpio.h"
#include "I2C_2.h"
#include "delay.h"



void GPIO_SDA_IN_2()
{

  LL_GPIO_InitTypeDef GPIO_InitStruct;

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
/*ʱ������*/
void IIC_Init_2(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct;
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);//PB ʱ��ʹ��
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);//PB ʱ��ʹ��
	GPIO_InitStruct.Pin = LL_GPIO_PIN_10|LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH ;//���˹�
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	HAL_GPIO_WritePin(GPIOB, LL_GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, LL_GPIO_PIN_11, GPIO_PIN_SET);
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}


/*��ʼ�źţ�*/
void IIC_Start_2(void)
{

	SDA_OUT_2();//sda �����
	SDA_high_2();
	SCL_high_2();
	delay_us_4;
	SDA_low_2(); //START:when CLK is high,DATA change form high to low
	delay_us_4;
	SCL_low_2(); //ǯס I2C ���ߣ�׼�����ͻ��������


}

/*�����źţ�*/
void IIC_Stop_2(void)
{
	SDA_OUT_2(); //sda �����
	SCL_low_2();
	SDA_low_2(); //STOP:when CLK is high DATA change form low to high
	delay_us_4;
	SCL_high_2();
	SDA_high_2(); //���� I2C ���߽����ź�
	delay_us_4;
	
	//LL_I2C_GenerateStopCondition(I2C3);
}

/*�ȴ�Ӧ���źţ�*/
uint8_t IIC_Wait_Ack_2()
{
	uint8_t ucErrTime=0;
	SDA_OUT_2();
	SDA_high_2();delay_us_1;
	SCL_high_2();delay_us_1;	
	
	GPIO_SDA_IN_2(); //SDA ����Ϊ����
	while(READ_SDA_2)
	{ 
		ucErrTime++;
		if(ucErrTime>250)
			{ 
				IIC_Stop_2();
				return 1;
			}
	}
	SCL_low_2(); //ʱ����� 0
	return 0;
	
	//LL_I2C_IsEnabledOwnAddress1(I2C3);
}
/*����I2CӦ��*/
void IIC_Ack_2(void)
{ 
	SCL_low_2();
	SDA_OUT_2();
	SDA_low_2();
	delay_us_2;
	SCL_high_2();
	delay_us_2;
	SCL_low_2();
	
	//LL_I2C_AcknowledgeNextData(I2C3, ll_I2C_ACK);
	//LL_I2C_EnableOwnAddress1(I2C);
	
}

/*������ ACK Ӧ��*/
void IIC_NAck_2(void)
{ 
	SCL_low_2();
	SDA_OUT_2();
	SDA_high_2();
	delay_us_2;
	SCL_high_2();
	delay_us_2;
	SCL_low_2();	
	
}

//IIC ����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��
void IIC_Send_Byte_2(uint8_t txd)
{                        
    uint8_t t;   
		SDA_OUT_2(); 	    
    SCL_low_2();//
    for(t=0;t<8;t++)
    {              
        //IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)
			SDA_high_2();
		else
			SDA_low_2();
		txd<<=1; 	  
		delay_us_2;   //
		SCL_high_2();
		delay_us_2; 
		SCL_low_2();	
		delay_us_2;
    }	 
} 
//�� 1 ���ֽڣ� ack=1 ʱ������ ACK�� ack=0������ nACK
uint8_t IIC_Read_Byte_2(unsigned char ack)
{ 
	unsigned char i,receive=0;
	SDA_IN_2();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        SCL_low_2();	
        delay_us_2;
				SCL_high_2();
        receive<<=1;
        if(READ_SDA_2)
					receive++;   
				delay_us_1; 
    }					 
    if (!ack)
        IIC_NAck_2();//����nACK
    else
        IIC_Ack_2(); //����ACK   
    return receive;
}
