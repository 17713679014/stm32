/******************************************************************************************************
 *【文件名称】   : mfrc.c
 *【文件描述】   : 移植C51例程
 *【文件功能】   : mfrc522底层驱动
 *【主控芯片】   : STM432F407zg
 *【实验平台】   : STM32F4xx开发板
 *【编写环境】   : IAR 8.30.1
 *【编写时间】   : 2020-03-12
 *【作    者】   : 薛晋涛(KevinLee)
 *【历史记录】   :   
					<1>	$【修改时间】             
						$【修改内容】
						$【修改详情】
						$【修改人员】

*******************************************************************************************************/




#include "mfrc.h"
#include "main.h"

#define MAXRLEN 18


//  关于寄存器 ，在MFRC522数据手册.pdf 中都有说明



/*******************************************************************
 @func		: PcdRequest
 @brief		: 寻卡
 @pram		: 1、req_code[IN]:寻卡方式
					0x52 = 寻感应区内所有符合14443A标准的卡
					0x26 = 寻未进入休眠状态的卡
                    第2条命令是读取完卡后还会再次读取。(除非在某次读取完成后系统进入休眠(Halt))
                    第1条命令是读取完卡后会等待卡离开开线作用范围，直到再次进入。
			  2、pTagType[OUT]：卡片类型代码
					0x4400 = Mifare_UltraLight
					0x0400 = Mifare_One(S50)
					0x0200 = Mifare_One(S70)
					0x0800 = Mifare_Pro(X)
					0x4403 = Mifare_DESFire
 @retval	: 寻卡成功  返回MI_OK
 @NOTE		: 外部调用
			: 函数将我们的寻卡命令PICC_REQIDL/ALL装填如要发送的数组，
			: 通过PcdComMF522函数发送出去
			: 回的两字节被装填入pTagType数组
*******************************************************************/
char PcdRequest(unsigned char req_code,unsigned char *pTagType)
{
   char status;  
   unsigned int  unLen;
   unsigned char ucComMF522Buf[MAXRLEN]; 

   ClearBitMask(Status2Reg,0x08);		// 清理指示MIFARECyptol单元(9.2.1.9) 
   WriteRawRC(BitFramingReg,0x07);		// 定义发送最后一个字节的位数 0x07即表示7bits
   SetBitMask(TxControlReg,0x03);		// TX1,2管脚输出经过发送调制的13.56MHz得能量载波信号
 
   ucComMF522Buf[0] = req_code;			// 蒋寻卡方式存储起来

   status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,1,ucComMF522Buf,&unLen);
   
   if ((status == MI_OK) && (unLen == 0x10))
   {    
       *pTagType     = ucComMF522Buf[0];
       *(pTagType+1) = ucComMF522Buf[1];
   }
   else
   {   
	   status = MI_ERR;  
   }
   
   return status;
}

/*******************************************************************
 @func		: PcdAnticoll
 @brief		: 防冲撞
 @pram		: pSnr[OUT]:卡片序列号，4字节
 @retval	: 寻卡成功  返回MI_OK
 @NOTE		: 外部调用
			: 防冲突操作就是将防冲突命令通过PcdComMF522函数与PICC卡进行交互
			: 防冲突命令是两个字节，其中第一字节为Mifare_One卡的防冲突命令字
			: PICC_ANTICOLL1(0x93)		第二个字节为0x20
			: 这两个字节在ISO14443协议中有解释

*******************************************************************/
char PcdAnticoll(unsigned char *pSnr)
{
    char status;
    unsigned char i,snr_check=0;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 
    

    ClearBitMask(Status2Reg,0x08);		// MFCyptol on 清零
    WriteRawRC(BitFramingReg,0x00);		// 清理寄存器 停止收发数据
    ClearBitMask(CollReg,0x80);			// VavluesAfterColl 清零 all receiving bits will be cleared after a collision
 
    ucComMF522Buf[0] = PICC_ANTICOLL1;	// 防冲突指令
    ucComMF522Buf[1] = 0x20;

	// 开始和卡片通信
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,2,ucComMF522Buf,&unLen);

    if (status == MI_OK)					// 如果通信成功
    {
    	 for (i=0; i<4; i++)
         {   
             *(pSnr+i)  = ucComMF522Buf[i];	// 读取UID
             snr_check ^= ucComMF522Buf[i];
         }
         if (snr_check != ucComMF522Buf[i])	// 如果校验位不对 则返回MI_ERR
         {   
			 status = MI_ERR;    
		 }
    }
    
    SetBitMask(CollReg,0x80);
    return status;
}


/*******************************************************************
 @func		: PcdSelect
 @brief		: 选定卡片
 @pram		: pSnr[IN]:卡片序列号，4字节
 @retval	: 成功选中  返回MI_OK
 @NOTE		: 外部调用
*******************************************************************/
char PcdSelect(unsigned char *pSnr)
{
    char status;
    unsigned char i;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 
    
    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x70;
    ucComMF522Buf[6] = 0;
    for (i=0; i<4; i++)
    {
    	ucComMF522Buf[i+2] = *(pSnr+i);
    	ucComMF522Buf[6]  ^= *(pSnr+i);
    }
    CalulateCRC(ucComMF522Buf,7,&ucComMF522Buf[7]);
  
    ClearBitMask(Status2Reg,0x08);

    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,9,ucComMF522Buf,&unLen);
    
    if ((status == MI_OK) && (unLen == 0x18))
	{
		status = MI_OK;  
	}else 
	{
		status = MI_ERR;    
	}

    return status;
}

/*******************************************************************
 @func		: PcdAuthState
 @brief		: 验证卡片密码
 @pram		: 1、auth_mode[IN]: 密码验证模式
					0x60 = 验证A密钥
					0x61 = 验证B密钥 
			  2、addr[IN]     : 块地址(0~3)
			  3、pKey[IN]     : 密码
			  4、pSnr[IN]     : 卡片序列号，4字节
 @retval	: 验证成功  返回MI_OK
 @NOTE		: 外部调用
*******************************************************************/
char PcdAuthState(unsigned char auth_mode,unsigned char addr,unsigned char *pKey,unsigned char *pSnr)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = auth_mode;
    ucComMF522Buf[1] = addr;
    for (i=0; i<6; i++)
    {    ucComMF522Buf[i+2] = *(pKey+i);   }
    for (i=0; i<6; i++)
    {    ucComMF522Buf[i+8] = *(pSnr+i);   }
 //   memcpy(&ucComMF522Buf[2], pKey, 6); 
 //   memcpy(&ucComMF522Buf[8], pSnr, 4); 
    
    status = PcdComMF522(PCD_AUTHENT,ucComMF522Buf,12,ucComMF522Buf,&unLen);
    if ((status != MI_OK) || (!(ReadRawRC(Status2Reg) & 0x08)))
    {   status = MI_ERR;   }
    
    return status;
}

/*******************************************************************
 @func		: PcdRead
 @brief		: 读取M1卡一块数据
 @pram		: 1、addr[IN] : 块地址
			  2、pData[IN]: 读出的数据，16字节

 @retval	: 成功  返回状态MI_OK
 @NOTE		: 外部调用
*******************************************************************/
char PcdRead(unsigned char addr,unsigned char *pData)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_READ;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
   
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
    if ((status == MI_OK) && (unLen == 0x90))
    {
        for (i=0; i<16; i++)
        {    *(pData+i) = ucComMF522Buf[i];   }
    }
    else
    {   status = MI_ERR;   }
    
    return status;
}



/*******************************************************************
 @func		: PcdWrite
 @brief		: 写数据到M1卡一块
 @pram		: 1、addr[IN] : 块地址
			  2、pData[IN]: 写入的数据，16字节

 @retval	: 验证成功  返回MI_OK
 @NOTE		: 外部调用
*******************************************************************/
char PcdWrite(unsigned char addr,unsigned char *pData)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 
    
    ucComMF522Buf[0] = PICC_WRITE;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }
        
    if (status == MI_OK)
    {
        //memcpy(ucComMF522Buf, pData, 16);
        for (i=0; i<16; i++)
        {    ucComMF522Buf[i] = *(pData+i);   }
        CalulateCRC(ucComMF522Buf,16,&ucComMF522Buf[16]);

        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,18,ucComMF522Buf,&unLen);
        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {   status = MI_ERR;   }
    }
    
    return status;
}





/*******************************************************************
 @func		: PcdHalt
 @brief		: 命令卡片进入休眠状态
 @pram		: None

 @retval	: 成功 返回状态MI_OK
 @NOTE		: 外部调用
	// 下面这两句是为了解决变量未使用的警告 下面两句顺序不可以变
	//status = MI_OK;
	//status = (int)status;
*******************************************************************/
char PcdHalt(void)
{
    char status;
	status = MI_OK;
	status = (int)status;
	
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_HALT;
    ucComMF522Buf[1] = 0;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    return MI_OK;
}


/*******************************************************************
 @func		: CalulateCRC
 @brief		: 用MF522计算CRC16函数  (循环冗余校验)
 @pram		: pIndata[IN]  计算CRC16的数组
			  len          计算CRC16的数组字节长度
			  pOutData     存放计算结果存放的首地址
 @retval	: None
 @NOTE		: 内部调用
*******************************************************************/
void CalulateCRC(unsigned char *pIndata,unsigned char len,unsigned char *pOutData)
{
    unsigned char i,n;
    ClearBitMask(DivIrqReg,0x04);
    WriteRawRC(CommandReg,PCD_IDLE);
    SetBitMask(FIFOLevelReg,0x80);
    for (i=0; i<len; i++)
    {   WriteRawRC(FIFODataReg, *(pIndata+i));   }
    WriteRawRC(CommandReg, PCD_CALCCRC);
    i = 0xFF;
    do 
    {
        n = ReadRawRC(DivIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x04));
    pOutData[0] = ReadRawRC(CRCResultRegL);
    pOutData[1] = ReadRawRC(CRCResultRegM);
}


/*******************************************************************
 @func		: PcdReset
 @brief		: 复位RC522
 @pram		: None
 @retval	: 成功返回MI_OK
 @NOTE		: 外部调用
*******************************************************************/
char PcdReset(void)
{
    /* MF522_RST=1; */
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    /* MF522_RST=0; */
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    /* MF522_RST=1; */
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    WriteRawRC(CommandReg,PCD_RESETPHASE);	// 复位
    HAL_Delay(10);
    
    WriteRawRC(ModeReg,0x3D);            	// 和Mifare卡通讯，CRC初始值0x6363
	WriteRawRC(TReloadRegL,30);      		// 16位定时器低位
	WriteRawRC(TReloadRegH,0);				// 16位定时器高位
	WriteRawRC(TModeReg,0x8D);				// 定时器内部设置
	WriteRawRC(TPrescalerReg,0x3E);			// 定时器分频系数设置
	WriteRawRC(TxAutoReg, 0x40);			// 调制发送信号为100%ASK	 调试的时候加上这一句试试
    return MI_OK;
}







/*
 * 函数名：M500PcdConfigISOType
 * 描述  ：设置RC522的工作方式
 * 输入  ：ucType，工作方式
 * 返回  : 无
 * 调用  ：外部调用
 */ 
char M500PcdConfigISOType(uint8_t type)
{ 
	if (type == 'A') 				//ISO14443_A 
	{ 
		ClearBitMask ( Status2Reg, 0x08 );
		WriteRawRC ( ModeReg, 0x3D );	//3F
		WriteRawRC ( RxSelReg, 0x86 );	//84 
		WriteRawRC( RFCfgReg, 0x7F ); 	//4F 
		WriteRawRC( TReloadRegL, 30 );	//tmoLength);// TReloadVal = 'h6a =tmoLength(dec) 
		WriteRawRC ( TReloadRegH, 0 ); 
		WriteRawRC ( TModeReg, 0x8D ); 
		WriteRawRC ( TPrescalerReg, 0x3E ); 
		HAL_Delay(10);
		PcdAntennaOn ();//开天线 
	} 
	
	else
	{
		return -1;
	}
	
	return MI_OK;
}


 
/*******************************************************************
 @func		: ReadRawRC
 @brief 	: 读RC632寄存器
 @pram		: Address[IN]:寄存器地址
 @retval	: 读出的值
 @NOTE		: MFRC522数据手册.pdf 10.2是关于SPI的详细说明   10.2.2 Read data
			: unsigned char === uint8_t
 @Call		: 内部调用
*******************************************************************/
unsigned char ReadRawRC(unsigned char Address) 
{
     unsigned char i, ucAddr;
     unsigned char ucResult=0;
   
	 HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);// MF522_NSS = 0;
	 HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);// MF522_SCK = 0;
	 
	 
	 // 地址左移一位是因为LSB是要保留 即RFU位(Reserved for Future Use)
	 // &0x7E 是把bit1~bit6 的地址(address)写入
	 // |0x80 是为了使最高位为1   1(Read) 0(Write) 即使能 '读'
     ucAddr = ((Address<<1)&0x7E)|0x80;
	 
	 for(i=8;i>0;i--)
	 {
		 if((ucAddr&0x80)==0x80)
		 {
			 HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_SET);
		 }
		 else
		 {
			 HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_RESET);
		 }
		 HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET);
		 ucAddr <<= 1;
		 HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);

	 }
	 
	 for(i=8;i>0;i--)
	 {
		HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET);
		ucResult <<= 1;
		ucResult |= HAL_GPIO_ReadPin(MISO_GPIO_Port, MISO_Pin);
		HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);
	 }

      
     HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);// MF522_NSS = 1;
	 HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET);// MF522_SCK = 1; 
	 
	 
     return ucResult;
}



/*******************************************************************
 @func		: WriteRawRC
 @brief 	: 写RC632寄存器
 @pram		: Address[IN]:寄存器地址
			: value[IN]:写入的值
 @retval	: None
 @Call		: 内部调用
*******************************************************************/
void WriteRawRC(unsigned char Address, unsigned char value)
{  
    unsigned char i, ucAddr;
	
	HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);// MF522_SCK = 0;
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);// MF522_NSS = 0;
	
	ucAddr = ((Address << 1) & 0x7E);
	
	for(i=8;i>0;i--)
	{
		if ((ucAddr&0x80)==0x80)
		{
			HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_RESET);
		}
		HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET);
		ucAddr <<= 1;
		HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);
	}
    
	
	for(i=8;i>0;i--)
	{
		// MF522_SI = ((value&0x80)==0x80);
		if ((value&0x80)==0x80)
		{
			HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_RESET);
		}
		HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET);
		value <<= 1;
		HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);
	}
	
    
	 
	
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);// MF522_NSS = 1; 
	HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET);// MF522_SCK = 1;
	
}



/*******************************************************************
 @func		: SetBitMask
 @brief		: 置RC522寄存器位
 @pram		: reg[IN]:寄存器地址
			  mask[IN]:置位值
 @retval	: None
 @Call		: 内部调用
*******************************************************************/
void SetBitMask(unsigned char reg,unsigned char mask)  
{
    char tmp = 0x0;
    tmp = ReadRawRC(reg);
    WriteRawRC(reg,tmp | mask);  // set bit mask
}


/*******************************************************************
 @func		: ClearBitMask
 @brief		: 清RC522寄存器位
 @pram		: 1、reg[IN]:寄存器地址
			  2、mask[IN]:清位值
 @retval	: None
 @Call		: 内部调用
*******************************************************************/
void ClearBitMask(unsigned char reg,unsigned char mask)  
{
    char tmp = 0x0;
    tmp = ReadRawRC(reg);
    WriteRawRC(reg, tmp & ~mask);  // clear bit mask
}




/*******************************************************************
 @func		: PcdComMF522
 @brief		: 通过RC522和ISO14443卡通讯
 @pram		: 1、Command[IN]:RC522命令字
			  2、pInData[IN]:通过RC522发送到卡片的数据
			  3、InLenByte[IN]:发送数据的字节长度
			  4、pOutData[OUT]:接收到的卡片返回数据
			  5、*pOutLenBit[OUT]:返回数据的位长度
 @retval	: 返回状态值
			  = MI_OK， 表示成功通讯
 @Call		: 内部调用
*******************************************************************/
char PcdComMF522(unsigned char Command, 
                 unsigned char *pInData, 
                 unsigned char InLenByte,
                 unsigned char *pOutData, 
                 unsigned int  *pOutLenBit)
{
    char status = MI_ERR;
    unsigned char irqEn   = 0x00;
    unsigned char waitFor = 0x00;
    unsigned char lastBits;
    unsigned char n;
    unsigned int i;
    switch (Command)		// 判断指令寄存器中的指令
    {
       case PCD_AUTHENT:	// Mifare 验证秘钥 (MFAuthent)
          irqEn   = 0x12;	// CommlEnReg->空闲中断请求使能 & 错误中断申请使能
          waitFor = 0x10;	// ComIrqReg->认证寻卡时.查询空闲中断标志位
          break;
       case PCD_TRANSCEIVE:	// 发送并接收数据
          irqEn   = 0x77;	// CommlEnReg->使能TxIEn、RxIEn、IdleIEn、
		  					// LoAlertIEn、ErrIEn、TimerIEn
          waitFor = 0x30;	// 寻卡等待的时候,查询接收中断标志位和空闲中断标志位
          break;
       default:
         break;
    }
   
	/* IRqInv置1 引脚IRQ上的信号相对于Status1Reg中的IRq位取反 */
    WriteRawRC(ComIEnReg,irqEn|0x80);
    ClearBitMask(ComIrqReg,0x80);		// ComIrqReg中Set1位清零(屏蔽位清零)
    WriteRawRC(CommandReg,PCD_IDLE);	// 写空闲指令
    SetBitMask(FIFOLevelReg,0x80);		// FlushBuffer置1 清除内部FIFO读写指针
										// 清除ErrReg中的BufferOvfl标志位
    
    for (i=0; i<InLenByte; i++)
	{
		WriteRawRC(FIFODataReg, pInData[i]);	// 写数据进FIFOData寄存器
	}
	
    WriteRawRC(CommandReg, Command);			// 写命令到CommandReg
   
    if (Command == PCD_TRANSCEIVE)				// 如果是接受发送指令
    {    
		SetBitMask(BitFramingReg,0x80);			// StartSend位置1 启动数据发送
												// 该位与收发指令一起使用才有效
	}
    
    i = 2000;//根据时钟频率调整，操作M1卡最大等待时间25ms
    do 											// 认证 & 寻卡等待时间
    {
         n = ReadRawRC(ComIrqReg);				// 等待过程中不断查询事件中断
         i--;
    }
    while ((i!=0) && !(n&0x01) && !(n&waitFor));// n&0x01 是检测定时器是否减到0
    ClearBitMask(BitFramingReg,0x80);			// 清除 StartSend 位
	      
    if (i!=0)
    {    
         if(!(ReadRawRC(ErrorReg)&0x1B))		// 读错误标志寄存器
         {
             status = MI_OK;					// 没有错误就把OK的状态存起来
             if (n & irqEn & 0x01)				// 判断是否发生定时器中断
             {   
				 status = MI_NOTAGERR;
			 }
             if (Command == PCD_TRANSCEIVE)
             {
               	n = ReadRawRC(FIFOLevelReg);	// 读取FIFO中保存的字节数
				/* 最后接收到的字节的有效个数 &0x07即该寄存器的最后三位 */
				/* 是RxLastBits	手册9.2.1.13 */
              	lastBits = ReadRawRC(ControlReg) & 0x07;
                if (lastBits)
                {   
					/* 返回数据的长度 */
					//N个字节数减去1（最后一个字节）+ 最后一位的位数=读取到的数据总位数
					*pOutLenBit = (n-1)*8 + lastBits; 
				}
                else
                {   
					*pOutLenBit = n*8;   // 如果lastBits不为1 则最后接收的字节整个是有效的
				}
                if (n == 0)
                {   
					n = 1;   
				}
                if (n > MAXRLEN)
                {   
					n = MAXRLEN;  
				}
                for (i=0; i<n; i++)
                {   
					pOutData[i] = ReadRawRC(FIFODataReg);	// 存储读出的FIFO数据
				}
            }
         }
         else
         {   
			 status = MI_ERR;
		 }
        
   }
   

   SetBitMask(ControlReg,0x80);           // stop timer now
   WriteRawRC(CommandReg,PCD_IDLE); 
   return status;
}


/*******************************************************************
 @func		: PcdAntennaOn
 @brief		: 开启天线
 @pram		: None
 @retval	: None
 @NOTE		: 每次启动或关闭天险发射之间应至少有1ms的间隔
			: 外部调用
*******************************************************************/
void PcdAntennaOn()
{
    unsigned char i;
    i = ReadRawRC(TxControlReg);
    if (!(i & 0x03))
    {
        SetBitMask(TxControlReg, 0x03);
    }
}


/*******************************************************************
 @func		: PcdAntennaOff
 @brief		: 关闭天线
 @pram		: None
 @retval	: None
 @NOTE		: 外部调用
*******************************************************************/
void PcdAntennaOff()
{
    ClearBitMask(TxControlReg, 0x03);
}



/*******************************************************************
 @func		: PcdValue
 @brief		: 扣款和充值
 @pram		: 1、dd_mode[IN] : 命令字
						0xC0 = 扣款
	               		0xC1 = 充值
			  2、addr[IN]    : 钱包地址
			  3、pValue[IN]  : 4字节增(减)值，低位在前

 @retval	: 成功 返回状态MI_OK
 @NOTE		: 外部调用
*******************************************************************/
char PcdValue(unsigned char dd_mode,unsigned char addr,unsigned char *pValue)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 
    
    ucComMF522Buf[0] = dd_mode;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }
        
    if (status == MI_OK)
    {
       // memcpy(ucComMF522Buf, pValue, 4);
        for (i=0; i<16; i++)
        {    ucComMF522Buf[i] = *(pValue+i);   }
        CalulateCRC(ucComMF522Buf,4,&ucComMF522Buf[4]);
        unLen = 0;
        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,6,ucComMF522Buf,&unLen);
        if (status != MI_ERR)
        {    status = MI_OK;    }
    }
    
    if (status == MI_OK)
    {
        ucComMF522Buf[0] = PICC_TRANSFER;
        ucComMF522Buf[1] = addr;
        CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]); 
   
        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {   status = MI_ERR;   }
    }
    return status;
}



/*******************************************************************
 @func		: PcdBakValue
 @brief		: 备份钱包
 @pram		: 1、sourceaddr[IN] : 源地址
			  2、goaladdr[IN]   : 目标地址

 @retval	: 成功 返回状态MI_OK
 @NOTE		: 外部调用
*******************************************************************/
char PcdBakValue(unsigned char sourceaddr, unsigned char goaladdr)
{
    char status;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_RESTORE;
    ucComMF522Buf[1] = sourceaddr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }
    
    if (status == MI_OK)
    {
        ucComMF522Buf[0] = 0;
        ucComMF522Buf[1] = 0;
        ucComMF522Buf[2] = 0;
        ucComMF522Buf[3] = 0;
        CalulateCRC(ucComMF522Buf,4,&ucComMF522Buf[4]);
 
        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,6,ucComMF522Buf,&unLen);
        if (status != MI_ERR)
        {    status = MI_OK;    }
    }
    
    if (status != MI_OK)
    {    return MI_ERR;   }
    
    ucComMF522Buf[0] = PICC_TRANSFER;
    ucComMF522Buf[1] = goaladdr;

    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }

    return status;
}
