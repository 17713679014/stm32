#include "GY30.h"
uint8_t Buf[3];
void GY30_Int(void)
{
	IIC_Init_2();
	GY30_WriteByte(1);
}
void GY30_WriteByte(uint8_t dat)
{
	IIC_Start_2();
	IIC_Send_Byte_2(GY30Address);
	IIC_Wait_Ack_2();
	IIC_Send_Byte_2(dat);
	IIC_Wait_Ack_2();
	IIC_Stop_2();
}
void GY30_Read3Bytes()
{
	uint8_t i;
	
	IIC_Start_2();
	IIC_Send_Byte_2(GY30Address + 1);
	IIC_Wait_Ack_2();
	for(i = 0; i < 2; i++)
	{
		if(i == 1)
		{
			Buf[i] = IIC_Read_Byte_2(0);
		}
		else
		{
			Buf[i] = IIC_Read_Byte_2(1);
		}
	}
	IIC_Stop_2();
	myDelay(500);
}
float GY30_GetLight()
{
	//u8 dat[2];
	float Light;
	GY30_WriteByte(1);
	myDelay(500);
	GY30_WriteByte(0x10);
	myDelay(500);
	GY30_Read3Bytes();
	Light =( (float)((uint8_t)Buf[0]<<8) + (float)Buf[1])/1.2;
	myDelay(500);
	return Light;
}
