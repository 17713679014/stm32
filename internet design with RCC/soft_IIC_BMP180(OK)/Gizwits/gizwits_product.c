/**
************************************************************
* @file         gizwits_product.c
* @brief        Gizwits control protocol processing, and platform-related       hardware initialization 
* @author       Gizwits
* @date         2017-07-19
* @version      V03030000
* @copyright    Gizwits
* 
* @note         机智云.只为智能硬件而生
*               Gizwits Smart Cloud  for Smart Products
*               链接|增值ֵ|开放|中立|安全|自有|自由|生态
*               www.gizwits.com
*
***********************************************************/

#include <stdio.h>
#include <string.h>
#include "hal_key.h"
#include "bmp180.h"
#include "gizwits_product.h"
#include "common.h"
#include "dht11.h"
static uint32_t timerMsCount;
uint8_t aRxBuffer;
dataPoint_t  tDataPoint;   
/** User area the current device state structure*/
dataPoint_t currentDataPoint;
extern keysTypedef_t keys;

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
//extern _bmp180 bmp180;
//extern uint8_t UV_index;
//extern DHT11_Data_TypeDef dht11;
//extern uint32_t ADC_UV,ADC_Bat,ADC_Light,ADC_PB0;
/**@} */
/**@name Gizwits User Interface
* @{
*/

/**
* @brief Event handling interface

* Description:

* 1. Users can customize the changes in WiFi module status

* 2. Users can add data points in the function of event processing logic, such as calling the relevant hardware peripherals operating interface

* @param [in] info: event queue
* @param [in] data: protocol data
* @param [in] len: protocol data length
* @return NULL
* @ref gizwits_protocol.h
*/
int8_t gizwitsEventProcess(eventInfo_t *info, uint8_t *gizdata, uint32_t len)
{
  uint8_t i = 0;
  dataPoint_t *dataPointPtr = (dataPoint_t *)gizdata;
  moduleStatusInfo_t *wifiData = (moduleStatusInfo_t *)gizdata;
  protocolTime_t *ptime = (protocolTime_t *)gizdata;
  
#if MODULE_TYPE
  gprsInfo_t *gprsInfoData = (gprsInfo_t *)gizdata;
#else
  moduleInfo_t *ptModuleInfo = (moduleInfo_t *)gizdata;
#endif

  if((NULL == info) || (NULL == gizdata))
  {
    return -1;
  }

  for(i=0; i<info->num; i++)
  {
    switch(info->event[i])
    {




      case WIFI_SOFTAP:
        break;
      case WIFI_AIRLINK:
        break;
      case WIFI_STATION:
        break;
      case WIFI_CON_ROUTER:
 
        break;
      case WIFI_DISCON_ROUTER:
 
        break;
      case WIFI_CON_M2M:
 
        break;
      case WIFI_DISCON_M2M:
        break;
      case WIFI_RSSI:
        GIZWITS_LOG("RSSI %d\n", wifiData->rssi);
        break;
      case TRANSPARENT_DATA:
        GIZWITS_LOG("TRANSPARENT_DATA \n");
        //user handle , Fetch data from [data] , size is [len]
        break;
      case WIFI_NTP:
        GIZWITS_LOG("WIFI_NTP : [%d-%d-%d %02d:%02d:%02d][%d] \n",ptime->year,ptime->month,ptime->day,ptime->hour,ptime->minute,ptime->second,ptime->ntp);
        break;
      case MODULE_INFO:
            GIZWITS_LOG("MODULE INFO ...\n");
      #if MODULE_TYPE
            GIZWITS_LOG("GPRS MODULE ...\n");
            //Format By gprsInfo_t
      #else
            GIZWITS_LOG("WIF MODULE ...\n");
            //Format By moduleInfo_t
            GIZWITS_LOG("moduleType : [%d] \n",ptModuleInfo->moduleType);
      #endif
    break;
      default:
        break;
    }
  }

  return 0;
}

/**
* User data acquisition

* Here users need to achieve in addition to data points other than the collection of data collection, can be self-defined acquisition frequency and design data filtering algorithm

* @param none
* @return none
*/


/**
* Data point initialization function

* In the function to complete the initial user-related data
* @param none
* @return none
* @note The developer can add a data point state initialization value within this function
*/
void userInit(void)
{
    memset((uint8_t*)&currentDataPoint, 0, sizeof(dataPoint_t));
    
    /** Warning !!! DataPoint Variables Init , Must Within The Data Range **/ 
    /*
      currentDataPoint.valueUltraviolet_index = ;
      currentDataPoint.valuetemp_value = ;
      currentDataPoint.valuehumidity_value = ;
      currentDataPoint.valuelight_intensity = ;
      currentDataPoint.valueair_pressure = ;
    */
		  currentDataPoint.valueUltraviolet_index = 2 ;
      currentDataPoint.valuetemp_value = 2;
      currentDataPoint.valuehumidity_value = 2;
      currentDataPoint.valuelight_intensity = 2;
      currentDataPoint.valueair_pressure = 2;
}


/**
* @brief Millisecond timing maintenance function, milliseconds increment, overflow to zero

* @param none
* @return none
*/
void gizTimerMs(void)
{
    timerMsCount++;
}

/**
* @brief Read millisecond count

* @param none
* @return millisecond count
*/
uint32_t gizGetTimerCount(void)
{
    return timerMsCount;
}

/**
* @brief MCU reset function

* @param none
* @return none
*/
void mcuRestart(void)
{
    __set_FAULTMASK(1);
    HAL_NVIC_SystemReset();
}

/**@} */



/**
  * @brief  Period elapsed callback in non blocking mode 
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim2)
	{
			keyHandle((keysTypedef_t *)&keys);
			gizTimerMs();
	}
}

/**
* @brief Timer TIM3 init function

* @param none
* @return none
*/
void timerInit(void)
{
	HAL_TIM_Base_Start_IT(&htim2);
}

/**
  * @brief  This function handles USART IDLE interrupt.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef*UartHandle)  
{  
    if(UartHandle->Instance == USART1)  
    {  
				gizPutData((uint8_t *)&aRxBuffer, 1);

        HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer, 1);//开启下一次接收中断  
    }  
}  

/**
* @brief USART init function

* Serial communication between WiFi modules and device MCU
* @param none
* @return none
*/
void uartInit(void)
{
//	HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer, 1);//开启下一次接收中断  
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer, 1);//开启下一次接收中断
}

/**
* @brief Serial port write operation, send data to WiFi module
*
* @param buf      : buf address
* @param len      : buf length
*
* @return : Return effective data length;-1，return failure
*/
int32_t uartWrite(uint8_t *buf, uint32_t len)
{
		uint8_t crc[1] = {0x55};
    uint32_t i = 0;
	
    if(NULL == buf)
    {
        return -1;
    }

    for(i=0; i<len; i++)
    {
        HAL_UART_Transmit_IT(&huart1, (uint8_t *)&buf[i], 1);
				while (huart1.gState != HAL_UART_STATE_READY);//Loop until the end of transmission

        if(i >=2 && buf[i] == 0xFF)
        {
						HAL_UART_Transmit_IT(&huart1, (uint8_t *)&crc, 1);
						while (huart1.gState != HAL_UART_STATE_READY);//Loop until the end of transmission
        }
    }

#ifdef PROTOCOL_DEBUG
    GIZWITS_LOG("MCU2WiFi[%4d:%4d]: ", gizGetTimerCount(), len);
    for(i=0; i<len; i++)
    {
        GIZWITS_LOG("%02x ", buf[i]);

        if(i >=2 && buf[i] == 0xFF)
        {
            GIZWITS_LOG("%02x ", 0x55);
        }
    }
    GIZWITS_LOG("\n");
#endif
		
		return len;
}  
