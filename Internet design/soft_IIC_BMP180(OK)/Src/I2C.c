/*******************************
***********模拟I2C**********************
********************************/

#include "stm32l4xx_ll_gpio.h"
#include "I2C.h"
#include "delay.h"

//#define IIC_SDA 		LL_GPIO_GetPinMode(GPIOB, LL_GPIO_PIN_6)
//#define IIC_SCL 		LL_GPIO_GetPinMode(GPIOB, LL_GPIO_PIN_7)
//#define SDA_OUT()  	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT )
//#define SDA_IN()		LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_INPUT )	
//#define 	SDA_high() 	SDA_high()
//#define 	SDA_low() 	SDA_low()
//#define 	SCL_high() 	SCL_high()
//#define 	SCL_low() 	SCL_low()

void GPIO_SDA_IN()
{

  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Ê±ÖÓÊ¹ÄÜ */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
	
	/*PA5*/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
/*时钟配置*/
void IIC_Init(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct;
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);//PB 时钟使能
	GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH ;//改了哈
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	HAL_GPIO_WritePin(GPIOB, LL_GPIO_PIN_7, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, LL_GPIO_PIN_6, GPIO_PIN_SET);
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}


/*开始信号：*/
void IIC_Start(void)
{

	SDA_OUT();//sda 线输出
	SDA_high();
	SCL_high();
	delay_us_4;
	SDA_low(); //START:when CLK is high,DATA change form high to low
	delay_us_4;
	SCL_low(); //钳住 I2C 总线，准备发送或接收数据


}

/*结束信号：*/
void IIC_Stop(void)
{
	SDA_OUT(); //sda 线输出
	SCL_low();
	SDA_low(); //STOP:when CLK is high DATA change form low to high
	delay_us_4;
	SCL_high();
	SDA_high(); //发送 I2C 总线结束信号
	delay_us_4;
	
	//LL_I2C_GenerateStopCondition(I2C3);
}

/*等待应答信号：*/
uint8_t IIC_Wait_Ack()
{
	uint8_t ucErrTime=0;
	SDA_OUT();
	SDA_high();delay_us_1;
	SCL_high();delay_us_1;	
	
	GPIO_SDA_IN(); //SDA 设置为输入
	while(READ_SDA)
	{ 
		ucErrTime++;
		if(ucErrTime>250)
			{ 
				IIC_Stop();
				return 1;
			}
	}
	SCL_low(); //时钟输出 0
	return 0;
	
	//LL_I2C_IsEnabledOwnAddress1(I2C3);
}
/*产生I2C应答*/
void IIC_Ack(void)
{ 
	SCL_low();
	SDA_OUT();
	SDA_low();
	delay_us_2;
	SCL_high();
	delay_us_2;
	SCL_low();
	
	//LL_I2C_AcknowledgeNextData(I2C3, ll_I2C_ACK);
	//LL_I2C_EnableOwnAddress1(I2C);
	
}

/*不产生 ACK 应答*/
void IIC_NAck(void)
{ 
	SCL_low();
	SDA_OUT();
	SDA_high();
	delay_us_2;
	SCL_high();
	delay_us_2;
	SCL_low();	
	
}

//IIC 发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
		SDA_OUT(); 	    
    SCL_low();//
    for(t=0;t<8;t++)
    {              
        //IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)
			SDA_high();
		else
			SDA_low();
		txd<<=1; 	  
		delay_us_2;   //
		SCL_high();
		delay_us_2; 
		SCL_low();	
		delay_us_2;
    }	 
} 
//读 1 个字节， ack=1 时，发送 ACK， ack=0，发送 nACK
uint8_t IIC_Read_Byte(unsigned char ack)
{ 
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        SCL_low();	
        delay_us_2;
				SCL_high();
        receive<<=1;
        if(READ_SDA)
					receive++;   
				delay_us_1; 
    }					 
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}
