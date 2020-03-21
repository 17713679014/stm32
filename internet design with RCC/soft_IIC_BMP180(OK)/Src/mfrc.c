/******************************************************************************************************
 *���ļ����ơ�   : mfrc.c
 *���ļ�������   : ��ֲC51����
 *���ļ����ܡ�   : mfrc522�ײ�����
 *������оƬ��   : STM432F407zg
 *��ʵ��ƽ̨��   : STM32F4xx������
 *����д������   : IAR 8.30.1
 *����дʱ�䡿   : 2020-03-12
 *����    �ߡ�   : Ѧ����(KevinLee)
 *����ʷ��¼��   :   
					<1>	$���޸�ʱ�䡿             
						$���޸����ݡ�
						$���޸����顿
						$���޸���Ա��

*******************************************************************************************************/




#include "mfrc.h"
#include "main.h"

#define MAXRLEN 18


//  ���ڼĴ��� ����MFRC522�����ֲ�.pdf �ж���˵��



/*******************************************************************
 @func		: PcdRequest
 @brief		: Ѱ��
 @pram		: 1��req_code[IN]:Ѱ����ʽ
					0x52 = Ѱ��Ӧ�������з���14443A��׼�Ŀ�
					0x26 = Ѱδ��������״̬�Ŀ�
                    ��2�������Ƕ�ȡ�꿨�󻹻��ٴζ�ȡ��(������ĳ�ζ�ȡ��ɺ�ϵͳ��������(Halt))
                    ��1�������Ƕ�ȡ�꿨���ȴ����뿪�������÷�Χ��ֱ���ٴν��롣
			  2��pTagType[OUT]����Ƭ���ʹ���
					0x4400 = Mifare_UltraLight
					0x0400 = Mifare_One(S50)
					0x0200 = Mifare_One(S70)
					0x0800 = Mifare_Pro(X)
					0x4403 = Mifare_DESFire
 @retval	: Ѱ���ɹ�  ����MI_OK
 @NOTE		: �ⲿ����
			: ���������ǵ�Ѱ������PICC_REQIDL/ALLװ����Ҫ���͵����飬
			: ͨ��PcdComMF522�������ͳ�ȥ
			: �ص����ֽڱ�װ����pTagType����
*******************************************************************/
char PcdRequest(unsigned char req_code,unsigned char *pTagType)
{
   char status;  
   unsigned int  unLen;
   unsigned char ucComMF522Buf[MAXRLEN]; 

   ClearBitMask(Status2Reg,0x08);		// ����ָʾMIFARECyptol��Ԫ(9.2.1.9) 
   WriteRawRC(BitFramingReg,0x07);		// ���巢�����һ���ֽڵ�λ�� 0x07����ʾ7bits
   SetBitMask(TxControlReg,0x03);		// TX1,2�ܽ�����������͵��Ƶ�13.56MHz�������ز��ź�
 
   ucComMF522Buf[0] = req_code;			// ��Ѱ����ʽ�洢����

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
 @brief		: ����ײ
 @pram		: pSnr[OUT]:��Ƭ���кţ�4�ֽ�
 @retval	: Ѱ���ɹ�  ����MI_OK
 @NOTE		: �ⲿ����
			: ����ͻ�������ǽ�����ͻ����ͨ��PcdComMF522������PICC�����н���
			: ����ͻ�����������ֽڣ����е�һ�ֽ�ΪMifare_One���ķ���ͻ������
			: PICC_ANTICOLL1(0x93)		�ڶ����ֽ�Ϊ0x20
			: �������ֽ���ISO14443Э�����н���

*******************************************************************/
char PcdAnticoll(unsigned char *pSnr)
{
    char status;
    unsigned char i,snr_check=0;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 
    

    ClearBitMask(Status2Reg,0x08);		// MFCyptol on ����
    WriteRawRC(BitFramingReg,0x00);		// ����Ĵ��� ֹͣ�շ�����
    ClearBitMask(CollReg,0x80);			// VavluesAfterColl ���� all receiving bits will be cleared after a collision
 
    ucComMF522Buf[0] = PICC_ANTICOLL1;	// ����ͻָ��
    ucComMF522Buf[1] = 0x20;

	// ��ʼ�Ϳ�Ƭͨ��
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,2,ucComMF522Buf,&unLen);

    if (status == MI_OK)					// ���ͨ�ųɹ�
    {
    	 for (i=0; i<4; i++)
         {   
             *(pSnr+i)  = ucComMF522Buf[i];	// ��ȡUID
             snr_check ^= ucComMF522Buf[i];
         }
         if (snr_check != ucComMF522Buf[i])	// ���У��λ���� �򷵻�MI_ERR
         {   
			 status = MI_ERR;    
		 }
    }
    
    SetBitMask(CollReg,0x80);
    return status;
}


/*******************************************************************
 @func		: PcdSelect
 @brief		: ѡ����Ƭ
 @pram		: pSnr[IN]:��Ƭ���кţ�4�ֽ�
 @retval	: �ɹ�ѡ��  ����MI_OK
 @NOTE		: �ⲿ����
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
 @brief		: ��֤��Ƭ����
 @pram		: 1��auth_mode[IN]: ������֤ģʽ
					0x60 = ��֤A��Կ
					0x61 = ��֤B��Կ 
			  2��addr[IN]     : ���ַ(0~3)
			  3��pKey[IN]     : ����
			  4��pSnr[IN]     : ��Ƭ���кţ�4�ֽ�
 @retval	: ��֤�ɹ�  ����MI_OK
 @NOTE		: �ⲿ����
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
 @brief		: ��ȡM1��һ������
 @pram		: 1��addr[IN] : ���ַ
			  2��pData[IN]: ���������ݣ�16�ֽ�

 @retval	: �ɹ�  ����״̬MI_OK
 @NOTE		: �ⲿ����
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
 @brief		: д���ݵ�M1��һ��
 @pram		: 1��addr[IN] : ���ַ
			  2��pData[IN]: д������ݣ�16�ֽ�

 @retval	: ��֤�ɹ�  ����MI_OK
 @NOTE		: �ⲿ����
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
 @brief		: ���Ƭ��������״̬
 @pram		: None

 @retval	: �ɹ� ����״̬MI_OK
 @NOTE		: �ⲿ����
	// ������������Ϊ�˽������δʹ�õľ��� ��������˳�򲻿��Ա�
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
 @brief		: ��MF522����CRC16����  (ѭ������У��)
 @pram		: pIndata[IN]  ����CRC16������
			  len          ����CRC16�������ֽڳ���
			  pOutData     ��ż�������ŵ��׵�ַ
 @retval	: None
 @NOTE		: �ڲ�����
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
 @brief		: ��λRC522
 @pram		: None
 @retval	: �ɹ�����MI_OK
 @NOTE		: �ⲿ����
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
    WriteRawRC(CommandReg,PCD_RESETPHASE);	// ��λ
    HAL_Delay(10);
    
    WriteRawRC(ModeReg,0x3D);            	// ��Mifare��ͨѶ��CRC��ʼֵ0x6363
	WriteRawRC(TReloadRegL,30);      		// 16λ��ʱ����λ
	WriteRawRC(TReloadRegH,0);				// 16λ��ʱ����λ
	WriteRawRC(TModeReg,0x8D);				// ��ʱ���ڲ�����
	WriteRawRC(TPrescalerReg,0x3E);			// ��ʱ����Ƶϵ������
	WriteRawRC(TxAutoReg, 0x40);			// ���Ʒ����ź�Ϊ100%ASK	 ���Ե�ʱ�������һ������
    return MI_OK;
}







/*
 * ��������M500PcdConfigISOType
 * ����  ������RC522�Ĺ�����ʽ
 * ����  ��ucType��������ʽ
 * ����  : ��
 * ����  ���ⲿ����
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
		PcdAntennaOn ();//������ 
	} 
	
	else
	{
		return -1;
	}
	
	return MI_OK;
}


 
/*******************************************************************
 @func		: ReadRawRC
 @brief 	: ��RC632�Ĵ���
 @pram		: Address[IN]:�Ĵ�����ַ
 @retval	: ������ֵ
 @NOTE		: MFRC522�����ֲ�.pdf 10.2�ǹ���SPI����ϸ˵��   10.2.2 Read data
			: unsigned char === uint8_t
 @Call		: �ڲ�����
*******************************************************************/
unsigned char ReadRawRC(unsigned char Address) 
{
     unsigned char i, ucAddr;
     unsigned char ucResult=0;
   
	 HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);// MF522_NSS = 0;
	 HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);// MF522_SCK = 0;
	 
	 
	 // ��ַ����һλ����ΪLSB��Ҫ���� ��RFUλ(Reserved for Future Use)
	 // &0x7E �ǰ�bit1~bit6 �ĵ�ַ(address)д��
	 // |0x80 ��Ϊ��ʹ���λΪ1   1(Read) 0(Write) ��ʹ�� '��'
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
 @brief 	: дRC632�Ĵ���
 @pram		: Address[IN]:�Ĵ�����ַ
			: value[IN]:д���ֵ
 @retval	: None
 @Call		: �ڲ�����
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
 @brief		: ��RC522�Ĵ���λ
 @pram		: reg[IN]:�Ĵ�����ַ
			  mask[IN]:��λֵ
 @retval	: None
 @Call		: �ڲ�����
*******************************************************************/
void SetBitMask(unsigned char reg,unsigned char mask)  
{
    char tmp = 0x0;
    tmp = ReadRawRC(reg);
    WriteRawRC(reg,tmp | mask);  // set bit mask
}


/*******************************************************************
 @func		: ClearBitMask
 @brief		: ��RC522�Ĵ���λ
 @pram		: 1��reg[IN]:�Ĵ�����ַ
			  2��mask[IN]:��λֵ
 @retval	: None
 @Call		: �ڲ�����
*******************************************************************/
void ClearBitMask(unsigned char reg,unsigned char mask)  
{
    char tmp = 0x0;
    tmp = ReadRawRC(reg);
    WriteRawRC(reg, tmp & ~mask);  // clear bit mask
}




/*******************************************************************
 @func		: PcdComMF522
 @brief		: ͨ��RC522��ISO14443��ͨѶ
 @pram		: 1��Command[IN]:RC522������
			  2��pInData[IN]:ͨ��RC522���͵���Ƭ������
			  3��InLenByte[IN]:�������ݵ��ֽڳ���
			  4��pOutData[OUT]:���յ��Ŀ�Ƭ��������
			  5��*pOutLenBit[OUT]:�������ݵ�λ����
 @retval	: ����״ֵ̬
			  = MI_OK�� ��ʾ�ɹ�ͨѶ
 @Call		: �ڲ�����
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
    switch (Command)		// �ж�ָ��Ĵ����е�ָ��
    {
       case PCD_AUTHENT:	// Mifare ��֤��Կ (MFAuthent)
          irqEn   = 0x12;	// CommlEnReg->�����ж�����ʹ�� & �����ж�����ʹ��
          waitFor = 0x10;	// ComIrqReg->��֤Ѱ��ʱ.��ѯ�����жϱ�־λ
          break;
       case PCD_TRANSCEIVE:	// ���Ͳ���������
          irqEn   = 0x77;	// CommlEnReg->ʹ��TxIEn��RxIEn��IdleIEn��
		  					// LoAlertIEn��ErrIEn��TimerIEn
          waitFor = 0x30;	// Ѱ���ȴ���ʱ��,��ѯ�����жϱ�־λ�Ϳ����жϱ�־λ
          break;
       default:
         break;
    }
   
	/* IRqInv��1 ����IRQ�ϵ��ź������Status1Reg�е�IRqλȡ�� */
    WriteRawRC(ComIEnReg,irqEn|0x80);
    ClearBitMask(ComIrqReg,0x80);		// ComIrqReg��Set1λ����(����λ����)
    WriteRawRC(CommandReg,PCD_IDLE);	// д����ָ��
    SetBitMask(FIFOLevelReg,0x80);		// FlushBuffer��1 ����ڲ�FIFO��дָ��
										// ���ErrReg�е�BufferOvfl��־λ
    
    for (i=0; i<InLenByte; i++)
	{
		WriteRawRC(FIFODataReg, pInData[i]);	// д���ݽ�FIFOData�Ĵ���
	}
	
    WriteRawRC(CommandReg, Command);			// д���CommandReg
   
    if (Command == PCD_TRANSCEIVE)				// ����ǽ��ܷ���ָ��
    {    
		SetBitMask(BitFramingReg,0x80);			// StartSendλ��1 �������ݷ���
												// ��λ���շ�ָ��һ��ʹ�ò���Ч
	}
    
    i = 2000;//����ʱ��Ƶ�ʵ���������M1�����ȴ�ʱ��25ms
    do 											// ��֤ & Ѱ���ȴ�ʱ��
    {
         n = ReadRawRC(ComIrqReg);				// �ȴ������в��ϲ�ѯ�¼��ж�
         i--;
    }
    while ((i!=0) && !(n&0x01) && !(n&waitFor));// n&0x01 �Ǽ�ⶨʱ���Ƿ����0
    ClearBitMask(BitFramingReg,0x80);			// ��� StartSend λ
	      
    if (i!=0)
    {    
         if(!(ReadRawRC(ErrorReg)&0x1B))		// �������־�Ĵ���
         {
             status = MI_OK;					// û�д���Ͱ�OK��״̬������
             if (n & irqEn & 0x01)				// �ж��Ƿ�����ʱ���ж�
             {   
				 status = MI_NOTAGERR;
			 }
             if (Command == PCD_TRANSCEIVE)
             {
               	n = ReadRawRC(FIFOLevelReg);	// ��ȡFIFO�б�����ֽ���
				/* �����յ����ֽڵ���Ч���� &0x07���üĴ����������λ */
				/* ��RxLastBits	�ֲ�9.2.1.13 */
              	lastBits = ReadRawRC(ControlReg) & 0x07;
                if (lastBits)
                {   
					/* �������ݵĳ��� */
					//N���ֽ�����ȥ1�����һ���ֽڣ�+ ���һλ��λ��=��ȡ����������λ��
					*pOutLenBit = (n-1)*8 + lastBits; 
				}
                else
                {   
					*pOutLenBit = n*8;   // ���lastBits��Ϊ1 �������յ��ֽ���������Ч��
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
					pOutData[i] = ReadRawRC(FIFODataReg);	// �洢������FIFO����
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
 @brief		: ��������
 @pram		: None
 @retval	: None
 @NOTE		: ÿ��������ر����շ���֮��Ӧ������1ms�ļ��
			: �ⲿ����
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
 @brief		: �ر�����
 @pram		: None
 @retval	: None
 @NOTE		: �ⲿ����
*******************************************************************/
void PcdAntennaOff()
{
    ClearBitMask(TxControlReg, 0x03);
}



/*******************************************************************
 @func		: PcdValue
 @brief		: �ۿ�ͳ�ֵ
 @pram		: 1��dd_mode[IN] : ������
						0xC0 = �ۿ�
	               		0xC1 = ��ֵ
			  2��addr[IN]    : Ǯ����ַ
			  3��pValue[IN]  : 4�ֽ���(��)ֵ����λ��ǰ

 @retval	: �ɹ� ����״̬MI_OK
 @NOTE		: �ⲿ����
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
 @brief		: ����Ǯ��
 @pram		: 1��sourceaddr[IN] : Դ��ַ
			  2��goaladdr[IN]   : Ŀ���ַ

 @retval	: �ɹ� ����״̬MI_OK
 @NOTE		: �ⲿ����
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
