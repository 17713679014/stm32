#ifndef  __GY30_H__
#define  __GY30_H__

#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_hal.h"
#include "I2C_2.h"
#include "math.h"
#include "delay.h"

#define	  GY30Address   0x46

extern uint8_t Buf[3];

void GY30_Int(void);
void GY30_WriteByte(uint8_t dat);
void GY30_Read3Bytes(void);
float GY30_GetLight(void);

#endif
