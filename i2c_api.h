/*
 * @File: i2c_api.h
 * @Author: panqunfeng  at <panqf1989@foxmail.com>
 * @Version: rev1.0.0
 * @Created Date: Thursday, January 3rd 2019, 4:18:44 pm
 * @Last Modified: Monday, 14th January 2019 5:13:49 pm
 * @Modified By: the developer formerly known as panqunfeng at <panqf1989@foxmail.com>
 * @Brief:
 * Copyright (c) 2019-2019 *** CO.，LTD
 * **************************************************************************************************
 * @Function List:
 * 1.
 * 2.
 * 3.
 * **************************************************************************************************
 * @Attention
 *  
 *  
 * --------------------------------------------------------------------------------------------------
 * The MIT License (MIT)
 * 
 * Copyright (c) 2018 *** CO.，LTD
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the 
 * ), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED 
 * , WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * --------------------------------------------------------------------------------------------------
 * HISTORY:
 * Date      	By	Comments
 * ----------	---	---------------------------------------------------------------------------------
 */
 
#ifndef _I2C_API_H__
#define _I2C_API_H__
#include "types.h"
#include "WPT_2837x_I2C.h"


/*----------------------------------------- Cut-Off Line -----------------------------------------*/
#define I2C_STOP_BIT_CHECK() I2CA_StopBitCheck()
#define I2C_SET_SLAVE_ADDR(addr) I2CA_SetSAddr(addr)
#define I2C_BUS_BUSY_CHECK() I2CA_BusBusyCheck()
#define I2C_SET_DATA_COUNT(num) I2CA_DataCount(num)
#define I2C_DATA_TRANSMIT(data) I2CA_DataTransmit(data)
#define I2C_START_SEND_DATA()              \
    do {                                   \
        I2CA_Start(I2C_START_TRANSMIT_DATA); \
    } while (0)
#define I2C_START_SEND_ADDR()              \
    do {                                   \
        I2CA_Start(I2C_START_TRANSMIT_ADDR); \
    } while (0)
#define I2C_START_RECEIVE_DATA()     \
    do {                             \
        I2CA_Start(I2C_START_RECEIVE); \
    } while (0)
#define I2C_READ_DATA() I2CA_ReadData()
#define I2C_NACK_CHECK() I2CA_NACKCheck()
#define I2C_SET_STOP_BIT() I2CA_SetStopBit()
#define I2C_NACK_CLEAR() I2CA_NACKClear()
#define I2C_TXFIFO_INT_ENABLE() I2CA_TXFIFOIntEnable()
#define I2C_TXFIFO_INT_DISABLE() I2CA_TXFIFOIntDisable()
/* IIC从机地址，只取底7位，最后一位根据I2CMD寄存器TRX设定，所以实际地址位A0/A1 */
#define I2C_SLAVE_ADDR              0x50
/* 写操作间隔 建议为10ms,即:I2C_OPERATION_INTERVAL * I2C_Loop的调用周期*/
#define I2C_OPERATION_INTERVAL     5
//设定单次最大写入数据长度 单位byte. 如AT24C02单次写入最大8字节.
#define I2C_SINGLE_WRITE_DATA_MAX_LEN   8

//
// I2C  Message Commands for I2CMSG struct
//
// #define I2C_MSGSTAT_INACTIVE 0x0000
// #define I2C_MSGSTAT_SEND_WITHSTOP 0x0010
// #define I2C_MSGSTAT_WRITE_BUSY 0x0011
#define I2C_MSGSTAT_WRITE_RESEND 0x0012
// #define I2C_MSGSTAT_SEND_NOSTOP 0x0020
// #define I2C_MSGSTAT_SEND_NOSTOP_BUSY 0x0021
// #define I2C_MSGSTAT_RESTART 0x0022
// #define I2C_MSGSTAT_READ_BUSY 0x0023
/* public typedef -------------------------------------------------------------------------------*/
/* public macro ---------------------------------------------------------------------------------*/
/* public variables -----------------------------------------------------------------------------*/
/** @defgroup
  * @{
  */
typedef enum {
    I2C_Device_A = 0x00,
    I2C_Device_B = 0x01
} I2C_Device_List;

//
// I2C Message Structure
//
struct _I2CMsg_TypeDef {
    Uint16 MsgStatus;       // Word stating what state msg is in:
                            //  I2C_MSGCMD_INACTIVE = do not send msg
                            //  I2C_MSGCMD_BUSY = msg start has been sent,
                            //                    awaiting stop
                            //  I2C_MSGCMD_SEND_WITHSTOP = command to send
                            //    master trans msg complete with a stop bit
                            //  I2C_MSGCMD_SEND_NOSTOP = command to send
                            //    master trans msg without the stop bit
                            //  I2C_MSGCMD_RESTART = command to send a
                            //    restart as a master receiver with a
                            //    stop bit
    Uint16 SlaveAddress;    // I2C address of slave msg is intended for
    Uint16 NumOfBytes;      // Num of valid bytes in (or to be put
                            // in MsgBuffer)
    Uint16 SentNum;         // Num of bytes has been sent
    Uint16 StartAddr;       // start address
    Uint16 MemoryHighAddr;  // EEPROM address of data associated with
                            // msg (high byte)
    Uint16 MemoryLowAddr;   // EEPROM address of data associated with
                            // msg (low byte)
    Uint16* pMsgBuffer;     // Array holding msg data - max that
                            // MAX_BUFFER_SIZE can be is 16 due
                            // to the FIFO's
};

/*
 **************************************************************************************************
 * @func        I2C_Write_Multi_Bytes
 * @brief       连续写入多个字节
 * @param       addr---EEPROM地址
 *              data---要写入的数据
 *              len---数据长度
 * @retval  
 * @remarks
 * @note
 * @see
 * @auther         pqf.pan at <panqf1989@foxmail.com>
 * @created date   Friday, January, 01 th 2019, 19:57:08
 * @last modified  Friday, January, 01 th 2019, 19:57:08
 * @modified by    
 * @version        rev1.0.0
 */
Uint16 I2C_Write_Multi_Bytes(Uint16 addr, Uint16* data, Uint16 len);

/*
 **************************************************************************************************
 * @func        I2C_Read_Multi_Bytes
 * @brief       连续读取多个字节
 * @param       addr---EEPROM地址
 *              len---要读取的数据长度
 * @retval  
 * @remarks
 * @note
 * @see
 * @auther         pqf.pan at <panqf1989@foxmail.com>
 * @created date   Friday, January, 01 th 2019, 20:04:57
 * @last modified  Friday, January, 01 th 2019, 20:04:57
 * @modified by    
 * @version        rev1.0.0
 */
Uint16 I2C_Read_Multi_Bytes(Uint16 addr, Uint16 len);

/*
 **************************************************************************************************
 * @func        I2C_Get_Read_Data
 * @brief       I2C 获取读取到的数据
 * @param       pbuffer:数据存储地址
 * @retval      I2C_SUCCESS:读取成功
 *              I2C_ERROR:读取失败
 * @remarks
 * @note
 * @see
 * @auther         pqf.pan at <panqf1989@foxmail.com>
 * @created date   Monday, January, 01 th 2019, 15:59:04
 * @last modified  Monday, January, 01 th 2019, 15:59:04
 * @modified by    
 * @version        rev1.0.0
 */
Uint16 I2C_Get_Read_Data(Uint16* pbuffer);

/*
 **************************************************************************************************
 * @func        I2C_Loop
 * @brief       eeprom轮询函数
 * @param 
 * @retval      0：没有数据需要读取 其他：有数据需要读取，数值表示数据长度
 * @remarks
 * @note        需要定时调用,周期约5ms
 * @see
 * @auther         pqf.pan at <panqf1989@foxmail.com>
 * @created date   Friday, January, 01 th 2019, 20:08:47
 * @last modified  Friday, January, 01 th 2019, 20:08:47
 * @modified by    
 * @version        rev1.0.0
 */
Uint16 I2C_Loop(void);

/*
 **************************************************************************************************
 * @func        I2C_IER_Isr
 * @brief       iic中断服务函数
 * @param       中断源
 * @retval  
 * @remarks
 * @note
 * @see
 * @auther         pqf.pan at <panqf1989@foxmail.com>
 * @created date   Friday, January, 01 th 2019, 20:09:31
 * @last modified  Friday, January, 01 th 2019, 20:09:31
 * @modified by    
 * @version        rev1.0.0
 */
void I2C_IER_Isr(Uint16 IntSource);

/*
 **************************************************************************************************
 * @func        I2C_TX_FIFO_Isr
 * @brief       i2c tx fifo中断服务函数
 * @param 
 * @retval  
 * @remarks
 * @note
 * @see
 * @auther         pqf.pan at <panqf1989@foxmail.com>
 * @created date   Friday, January, 01 th 2019, 17:20:35
 * @last modified  Friday, January, 01 th 2019, 17:20:35
 * @modified by    
 * @version        rev1.0.0
 */
void I2C_TX_FIFO_Isr(void);

/*
 **************************************************************************************************
 * @func        I2C_RX_FIFO_Isr
 * @brief       i2c rx fifo 中断服务函数
 * @param 
 * @retval  
 * @remarks
 * @note
 * @see
 * @auther         pqf.pan at <panqf1989@foxmail.com>
 * @created date   Friday, January, 01 th 2019, 17:20:54
 * @last modified  Friday, January, 01 th 2019, 17:20:54
 * @modified by    
 * @version        rev1.0.0
 */
void I2C_RX_FIFO_Isr(void);
#endif

