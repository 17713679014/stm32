#ifndef __I2C_H
#define __I2C_H

#include "stm32l4xx_ll_gpio.h"
#include "I2C.h"

#define   delay_us_4	us_Dealy(10)
#define   delay_us_2	us_Dealy(8)
#define		delay_us_1	us_Dealy(1)
#define		READ_SDA		HAL_GPIO_ReadPin(GPIOB, LL_GPIO_PIN_6)

#define SDA_OUT()				LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT )
#define SDA_IN()				LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_INPUT )
#define SDA_high()			HAL_GPIO_WritePin(GPIOB, LL_GPIO_PIN_6, GPIO_PIN_SET)
#define SDA_low()				HAL_GPIO_WritePin(GPIOB, LL_GPIO_PIN_6, GPIO_PIN_RESET)
#define SCL_high()			HAL_GPIO_WritePin(GPIOB, LL_GPIO_PIN_7, GPIO_PIN_SET)
#define SCL_low()				HAL_GPIO_WritePin(GPIOB, LL_GPIO_PIN_7, GPIO_PIN_RESET)

void IIC_Init(void);
uint8_t IIC_Read_Byte(unsigned char ack);
void IIC_Send_Byte(uint8_t txd);
void IIC_NAck(void);
void IIC_Ack(void);
uint8_t IIC_Wait_Ack(void);
void IIC_Stop(void);
void IIC_Start(void);

/*void LL_GPIO_SetPinMode(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Mode)
{
  MODIFY_REG(GPIOx->MODER, (GPIO_MODER_MODE0 << (POSITION_VAL(Pin) * 2U)), (Mode << (POSITION_VAL(Pin) * 2U)));
}*/
#endif
