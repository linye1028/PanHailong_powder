/***************************************************************
*琴岛学院 创新实验室
*机电工程系 NERV
*30-Apr-2012
*杨悦 13589226458
*QQ 357017598 
FreeModbus 通过 
eMBRegInputCB eMBRegHoldingCB 
eMBRegCoilsCB	eMBRegDiscreteCB 四个接口函数完成数据的读写操作 
**************************************************************/
#include "stm32f10x.h"
#include "mb.h" 
void ENTER_CRITICAL_SECTION(void)//进入超临界 关总中断
{
	__set_PRIMASK(1);
}

void EXIT_CRITICAL_SECTION(void)//退出超临界 开总中断
{
	__set_PRIMASK(0);
}


//u16 usRegInputBuf[10]={0x0000,0xfe02,0x1203,0x1304,0x1405,0x1506,0x1607,0x1708,0x1809};
//一个测试用的 寄存器数组 地址0-7
u16 usRegInputBuf[16]={0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000};  //输入寄存器  ×3
u16 usRegHoldingBuf[16]={1200,0x0002,0x0003,0x0004,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000};	//保持寄存器 ×4
u8 ucRegCoilsBuf[4]={0x00,0x00,0x00,0x00};  //输出线圈 ×0
u8 ucRegDiscBuf[4]={0x00,0x00,0x00,0x00}; //离散输入  ×1
static u8 REG_INPUT_START=0,REG_HOLDING_START=0,REG_COILS_START=0,REG_DISC_START=0;
u8 REG_INPUT_NREGS=17,REG_HOLDING_NREGS=17,REG_COILS_SIZE=33,REG_DISC_SIZE=33;
u8 usRegInputStart=1,usRegHoldingStart=1,usRegCoilsStart=1,usRegDiscStart=1;

//读数字寄存器 功能码0x04

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
	eMBErrorCode    eStatus = MB_ENOERR;
	int             iRegIndex;

    if( ( usAddress >= REG_INPUT_START )&& ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;

}
// 寄存器的读写函数 支持的命令为读 0x03 和写0x06

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;
	u16 *PRT=(u16*)pucRegBuffer;  

    if( ( usAddress >= REG_HOLDING_START ) && ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );
        switch ( eMode )
        {
        case MB_REG_READ:
            while( usNRegs > 0 )
            {
                *PRT++ = __REV16(usRegHoldingBuf[iRegIndex++]); //数据序转 REV16.W

// 				*pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
//              *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
//				iRegIndex++;
                usNRegs--;
            }
            break;

        case MB_REG_WRITE:
            while( usNRegs > 0 )
            {
                usRegHoldingBuf[iRegIndex++] = __REV16(*PRT++); //数据序转 REV16.W

//				usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
//              usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
//              iRegIndex++;
                usNRegs--;
            }
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

// //读/写开关寄存器  0x01  x05
// eMBErrorCode
// eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode )
// {
//     ( void )pucRegBuffer;
//     ( void )usAddress;
//     ( void )usNCoils;
//     ( void )eMode;
//     return MB_ENOREG;
// }
eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    short           iNCoils = ( short )usNCoils;
    unsigned short  usBitOffset;

    /* Check if we have registers mapped at this block. */
    if( ( usAddress >= REG_COILS_START ) &&
        ( usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE ) )
    {
        usBitOffset = ( unsigned short )( usAddress - usRegCoilsStart );
        switch ( eMode )
        {
                /* Read current values and pass to protocol stack. */
            case MB_REG_READ:
                while( iNCoils > 0 )
                {
                    *pucRegBuffer++ =
                        xMBUtilGetBits( ucRegCoilsBuf, usBitOffset,
                                        ( unsigned char )( iNCoils >
                                                           8 ? 8 :
                                                           iNCoils ) );
                    iNCoils -= 8;
                    usBitOffset += 8;
                }
                break;

                /* Update current register values. */
            case MB_REG_WRITE:
                while( iNCoils > 0 )
                {
                    xMBUtilSetBits( ucRegCoilsBuf, usBitOffset,
                                    ( unsigned char )( iNCoils > 8 ? 8 : iNCoils ),
                                    *pucRegBuffer++ );
                    iNCoils -= 8;
                }
                break;
        }

    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}


// //读开关寄存器 0x02
// eMBErrorCode
// eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
// {
//     ( void )pucRegBuffer;
//     ( void )usAddress;
//     ( void )usNDiscrete;
//     return MB_ENOREG;
// }
eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    short           iNDiscrete = ( short )usNDiscrete;
    unsigned short  usBitOffset;

    /* Check if we have registers mapped at this block. */
    if( ( usAddress >= REG_DISC_START ) &&
        ( usAddress + usNDiscrete <= REG_DISC_START + REG_DISC_SIZE ) )
    {
        usBitOffset = ( unsigned short )( usAddress - usRegDiscStart );
        while( iNDiscrete > 0 )
        {
            *pucRegBuffer++ =
                xMBUtilGetBits( ucRegDiscBuf, usBitOffset,
                                ( unsigned char )( iNDiscrete >
                                                   8 ? 8 : iNDiscrete ) );
            iNDiscrete -= 8;
            usBitOffset += 8;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}
