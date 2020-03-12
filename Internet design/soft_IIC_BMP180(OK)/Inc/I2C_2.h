#ifndef __I2C_H
#define __I2C_H

#include "stm32l4xx_ll_gpio.h"
#include "I2C.h"

#define   delay_us_4	us_Dealy(10)
#define   delay_us_2	us_Dealy(8)
#define		delay_us_1	us_Dealy(1)
#define		READ_SDA_2		HAL_GPIO_ReadPin(GPIOB, LL_GPIO_PIN_11)

#define SDA_OUT_2()				LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_11, LL_GPIO_MODE_OUTPUT )
#define SDA_IN_2()				LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_11, LL_GPIO_MODE_INPUT )
#define SDA_high_2()			HAL_GPIO_WritePin(GPIOB, LL_GPIO_PIN_11, GPIO_PIN_SET)
#define SDA_low_2()				HAL_GPIO_WritePin(GPIOB, LL_GPIO_PIN_11, GPIO_PIN_RESET)
#define SCL_high_2()			HAL_GPIO_WritePin(GPIOB, LL_GPIO_PIN_10, GPIO_PIN_SET)
#define SCL_low_2()				HAL_GPIO_WritePin(GPIOB, LL_GPIO_PIN_10, GPIO_PIN_RESET)

void IIC_Init_2(void);
uint8_t IIC_Read_Byte_2(unsigned char ack);
void IIC_Send_Byte_2(uint8_t txd);
void IIC_NAck_2(void);
void IIC_Ack_2(void);
uint8_t IIC_Wait_Ack_2(void);
void IIC_Stop_2(void);
void IIC_Start_2(void);

/*void LL_GPIO_SetPinMode(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Mode)
{
  MODIFY_REG(GPIOx->MODER, (GPIO_MODER_MODE0 << (POSITION_VAL(Pin) * 2U)), (Mode << (POSITION_VAL(Pin) * 2U)));
}*/
#endif
