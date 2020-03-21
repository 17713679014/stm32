#include "main.h"
#include "delay.h"

void us_Dealy(uint16_t m)
{
	uint16_t i=1,j=12;
	while(m--)
	{
		while(i--);
		{
		i=1;
			j=j*j+i;
		}	
	}
}
void myDelay(uint16_t a)//a=1000,´óÔ¼ÊÇ1s=1000ms=1*10^6us
{
	uint16_t i=1000;
	while(a--)
	{
		while(i--);
		i=1000;
	}
}

void  LL_myDelay(uint32_t Delay)
{

  __IO uint32_t  tmp = SysTick->CTRL;  /* Clear the COUNTFLAG first */
  /* Add this code to indicate that local variable is not used */
  ((void)tmp);

  /* Add a period to guaranty minimum wait */
  if(Delay < LL_MAX_DELAY)
  {
    Delay++;
  }

  while (Delay)
  {
    if((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0U)
    {
      Delay--;
    }
  }
}