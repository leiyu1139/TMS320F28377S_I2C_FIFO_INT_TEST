/*
 * @File: i2c_api.c
 * @Author: panqunfeng  at <panqf1989@foxmail.com>
 * @Version: rev1.0.0
 * @Created Date: Thursday, January 3rd 2019, 4:18:44 pm
 * @Last Modified: Monday, 14th January 2019 8:34:23 pm
 * @Modified By: the developer formerly known as panqunfeng at <panqunfeng@xmnewyea.com>
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






/* Includes --------------------------------------------------------------------------------------*/
#include "WPT_2837x_I2C.h"
#include "i2c_api.h"
#include "string.h"
#include "types.h"
/* Private define --------------------------------------------------------------------------------*/
/* Private typedef -------------------------------------------------------------------------------*/
/* Private macro ---------------------------------------------------------------------------------*/
static Uint16 I2C_Data_Buffer[256] = { 0x00 };
static Uint16 I2C_Interval_Count = I2C_OPERATION_INTERVAL;
// Uint8 I2C_TXData_Buffer[256] = { 0x00 };
// Two bytes will be used for the outgoing address,
// thus only setup 14 bytes maximum
static struct _I2CMsg_TypeDef I2cMsgOut1 = { I2C_MSGSTAT_INACTIVE,
    I2C_SLAVE_ADDR,
    0,
    0,
    0,
    0,
    NULL };

static struct _I2CMsg_TypeDef I2cMsgIn1 = { I2C_MSGSTAT_INACTIVE,
    I2C_SLAVE_ADDR,
    0,
    0,
    0,
    0,
    NULL };
/* Private variables -----------------------------------------------------------------------------*/
/* Private function prototypes -------------------------------------------------------------------*/
/* Private functions -----------------------------------------------------------------------------*/
static Uint16 I2C_Write_Msg(struct _I2CMsg_TypeDef* msg);
static Uint16 I2C_Read_Msg(struct _I2CMsg_TypeDef* msg);
static struct _I2CMsg_TypeDef* CurrentMsgPtr; // Used in interrupts


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
Uint16 I2C_Write_Multi_Bytes(Uint16 addr, Uint16* data, Uint16 len)
{
    Uint16 i;

    /* 正在接收或发送数据，返回忙错误 */
    if((I2cMsgOut1.MsgStatus != I2C_MSGSTAT_INACTIVE) || \
        (I2cMsgIn1.MsgStatus != I2C_MSGSTAT_INACTIVE))
        return I2C_BUS_BUSY_ERROR;

//    i2ca_send.addr = addr;
//    i2ca_send.data_len = len;
//    i2ca_send.operation_index = 0;
    I2cMsgOut1.NumOfBytes = len;
    I2cMsgOut1.SentNum = 0;
    I2cMsgOut1.MemoryHighAddr = (addr >> 8);
    I2cMsgOut1.MemoryLowAddr = (addr & 0xFF);
    I2cMsgOut1.pMsgBuffer = I2C_Data_Buffer;
    I2cMsgIn1.pMsgBuffer = NULL;
    for (i = 0; i < len; i++) {
        // i2ca_send.buf[i] = data[i];
        I2cMsgOut1.pMsgBuffer[i] = data[i];
    }
    /* 发送一次数据，剩下的等待完成中断后继续完成 */
    // return i2ca_send_continuous();
    return I2C_Write_Msg(&I2cMsgOut1);
}

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
Uint16 I2C_Read_Multi_Bytes(Uint16 addr, Uint16 len)
{
    Uint16 i, error;

    /* 正在接收或发送数据，返回忙错误 */
    if((I2cMsgOut1.MsgStatus != I2C_MSGSTAT_INACTIVE) || \
        (I2cMsgIn1.MsgStatus != I2C_MSGSTAT_INACTIVE))
//        (i2ca_recv_interval_count != 0))
        return I2C_BUS_BUSY_ERROR;

    // i2ca_recv.addr = addr;
    // i2ca_recv.data_len = len;
    // i2ca_recv.operation_index = 0;
    // for(i = 0; i < len; i ++) {
    //     i2ca_recv.buf[i] = 0;
    // }
    I2cMsgIn1.NumOfBytes = len;
    I2cMsgIn1.SentNum = 0;
    I2cMsgIn1.MemoryHighAddr = (addr >> 8);
    I2cMsgIn1.MemoryLowAddr = (addr & 0xFF);
    I2cMsgIn1.pMsgBuffer = I2C_Data_Buffer;
    I2cMsgOut1.pMsgBuffer = NULL;
    for (i = 0; i < len; i++) {
        I2cMsgIn1.pMsgBuffer[i] = 0;
    }
    I2cMsgIn1.MsgStatus = I2C_MSGSTAT_SEND_NOSTOP;
    CurrentMsgPtr = &I2cMsgIn1;

    /* 发送一次数据，剩下的等待完成中断后继续完成 */
    // error = i2ca_recv_continuous();
    error = I2C_Read_Msg(&I2cMsgIn1);
    /* 设置读取状态 */
    if (error != I2C_SUCCESS)
        I2cMsgIn1.MsgStatus = I2C_MSGSTAT_INACTIVE;

    return error;
}

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
Uint16 I2C_Get_Read_Data(Uint16* pbuffer)
{
    Uint16 i = 0;
    if (I2cMsgIn1.MsgStatus == I2C_MSGSTAT_INACTIVE)
    {
        for (i = 0; i < I2cMsgIn1.NumOfBytes;i++)
        {
            *(pbuffer + i) = I2C_Data_Buffer[i];
        }
        return I2C_SUCCESS;
    }
    return I2C_ERROR;
}
/*
 **************************************************************************************************
 * @func        I2C_Write_Msg
 * @brief       i2c写消息函数
 * @param       msg---要写入的数据结构
 * @retval      操作结果
 * @remarks
 * @note
 * @see
 * @auther         pqf.pan at <panqf1989@foxmail.com>
 * @created date   Friday, January, 01 th 2019, 20:06:19
 * @last modified  Friday, January, 01 th 2019, 20:06:19
 * @modified by    
 * @version        rev1.0.0
 */
static Uint16 I2C_Write_Msg(struct _I2CMsg_TypeDef *msg)
{
    Uint16 i;
    Uint16 len;

    //
    // Wait until the STP bit is cleared from any previous master communication.
    // Clearing of this bit by the module is delayed until after the SCD bit is
    // set. If this bit is not checked prior to initiating a new message, the
    // I2C could get confused.
    //
    if (I2C_STOP_BIT_CHECK() == 1) {
        return I2C_STP_NOT_READY_ERROR;
    }

    //
    // Setup slave address
    //
    I2C_SET_SLAVE_ADDR(msg->SlaveAddress);

    //
    // Check if bus busy
    //
    if(I2C_BUS_BUSY_CHECK() == 1)
    {
        return I2C_BUS_BUSY_ERROR;
    }

    if (msg->NumOfBytes - msg->SentNum > I2C_SINGLE_WRITE_DATA_MAX_LEN) {
        len = I2C_SINGLE_WRITE_DATA_MAX_LEN;
    } else {
        len = msg->NumOfBytes - msg->SentNum;
    }
    //
    // Setup number of bytes to send MsgBuffer + Address
    //
    I2C_SET_DATA_COUNT(len + 1);

    //
    // Setup data to send
    //
//    I2C_DATA_TRANSMIT(msg->MemoryHighAddr);
    I2C_DATA_TRANSMIT(msg->MemoryLowAddr);

    // for (i=0; i<msg->NumOfBytes-2; i++)
    for (i = 0; i < len; i++) {
        // I2caRegs.I2CDXR.all = *(msg->MsgBuffer + i);
        I2C_DATA_TRANSMIT(*(msg->pMsgBuffer + i));
        msg->SentNum++;
    }

    //
    // Send start as master transmitter
    //
    if (len > 14) {
        I2C_TXFIFO_INT_ENABLE();
    }
    I2C_START_SEND_DATA();
    /* 设置发送状态 */
    I2cMsgOut1.MsgStatus = I2C_MSGSTAT_WRITE_BUSY;
    CurrentMsgPtr = &I2cMsgOut1;
    return I2C_SUCCESS;
}

/*
 **************************************************************************************************
 * @func        I2C_Read_Msg
 * @brief       i2c读信息函数
 * @param       msg---读取配置的数据结构
 * @retval  
 * @remarks
 * @note
 * @see
 * @auther         pqf.pan at <panqf1989@foxmail.com>
 * @created date   Friday, January, 01 th 2019, 20:08:03
 * @last modified  Friday, January, 01 th 2019, 20:08:03
 * @modified by    
 * @version        rev1.0.0
 */
static Uint16 I2C_Read_Msg(struct _I2CMsg_TypeDef *msg)
{
    Uint16 len;
    //
    // Wait until the STP bit is cleared from any previous master communication.
    // Clearing of this bit by the module is delayed until after the SCD bit is
    // set. If this bit is not checked prior to initiating a new message, the
    // I2C could get confused.
    //
    if (I2C_STOP_BIT_CHECK() == 1) {
        return I2C_STP_NOT_READY_ERROR;
    }
    I2C_SET_SLAVE_ADDR(msg->SlaveAddress);

    if(msg->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP)
    {
        //
        // Check if bus busy
        //
        if (I2C_BUS_BUSY_CHECK() == 1) {
            return I2C_BUS_BUSY_ERROR;
        }
        I2C_SET_DATA_COUNT(1);
//        I2C_DATA_TRANSMIT(msg->MemoryHighAddr);
        I2C_DATA_TRANSMIT(msg->MemoryLowAddr);
        //
        // Send data to setup EEPROM address
        //
        I2C_START_SEND_ADDR();
        msg->MsgStatus = I2C_MSGSTAT_SEND_NOSTOP_BUSY;
    }

    else if(msg->MsgStatus == I2C_MSGSTAT_RESTART)
    {
        // TODO 根据FIFO level 进行处理.避免最后几个字节无法触发fifo中断.
        if((msg->NumOfBytes % 2) != 0)
        {
            len = msg->NumOfBytes + 1;
        }
        else
        {
            len = msg->NumOfBytes;
        }
        msg->SentNum = 0;
        // Setup how many bytes to expect
        I2C_SET_DATA_COUNT(len);
        // Send restart as master receiver
        I2C_START_RECEIVE_DATA();
        //    I2caRegs.I2CFFTX.bit.TXFFRST = 0x0D;
        // I2caRegs.I2CFFTX.all = 0x6000;    // Enable FIFO mode and TXFIFO
//        I2caRegs.I2CFFRX.bit.RXFFIENA = 1;
//        I2caRegs.I2CFFRX.bit.RXFFINTCLR = 1;
//        I2caRegs.I2CFFTX.bit.TXFFRST = 1;
//        I2caRegs.I2CFFTX.bit.TXFFINTCLR = 1;
        msg->MsgStatus = I2C_MSGSTAT_READ_BUSY;
    }

    return I2C_SUCCESS;
}

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
Uint16 I2C_Loop(void)
{
    Uint16 addr;
    if ((I2cMsgOut1.MsgStatus == I2C_MSGSTAT_WRITE_RESEND) && (I2C_Interval_Count == 0)) {
        I2C_Interval_Count = I2C_OPERATION_INTERVAL;
        addr = ((I2cMsgOut1.MemoryHighAddr << 8) | I2cMsgOut1.MemoryLowAddr);
        addr += I2C_SINGLE_WRITE_DATA_MAX_LEN;
        I2cMsgOut1.MemoryHighAddr = (addr >> 8);
        I2cMsgOut1.MemoryLowAddr = (addr & 0xFF);
        I2cMsgOut1.pMsgBuffer = I2C_Data_Buffer + I2cMsgOut1.SentNum;
        I2cMsgIn1.pMsgBuffer = NULL;
        /* 发送一次数据，剩下的等待完成中断后继续完成 */
        // return i2ca_send_continuous();
        I2C_Write_Msg(&I2cMsgOut1);
    }else
    {
        I2C_Interval_Count--;
    }
    if (I2cMsgOut1.MsgStatus != I2C_MSGSTAT_WRITE_RESEND) {
        I2C_Interval_Count = I2C_OPERATION_INTERVAL;
    }
    // if (I2cMsgIn1.MsgStatus == I2C_MSGSTAT_RESTART) {
    //     while (I2C_Read_Msg(&I2cMsgIn1) != I2C_SUCCESS) {
    //         //
    //         // Maybe setup an attempt counter to break an infinite while
    //         // loop.
    //         //
    //     }
    //     //
    //     // Update current message pointer and message status
    //     //
    //     CurrentMsgPtr = &I2cMsgIn1;
    //     I2cMsgIn1.MsgStatus = I2C_MSGSTAT_READ_BUSY;
    // }
    return 0;
}

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
void I2C_IER_Isr(Uint16 IntSource)
{
//    Uint16 i;
    //
    // Interrupt source = stop condition detected
    //
    if(CurrentMsgPtr == NULL)
        return;
    if(IntSource == I2C_SCD_ISRC)
    {
        //
        // If completed message was writing data, reset msg to inactive state
        //
        if (CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_WRITE_BUSY)
        {
            // /* 移动EEPROM地址和操作索引 */
            // i2ca_send.addr += CurrentMsgPtr->NumOfBytes;
            // i2ca_send.operation_index += CurrentMsgPtr->NumOfBytes;
            // if(i2ca_send.operation_index >= i2ca_send.data_len)
            // {
            //     i2ca_recv_interval_count = I2CA_OPERATION_INTERVAL;
            //     CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_INACTIVE;
            // }
            // else {
            //     CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_WRITE_RESEND;
            // }
            if(CurrentMsgPtr->SentNum >= CurrentMsgPtr->NumOfBytes)
            {
                //写入完成
                CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_INACTIVE;
                CurrentMsgPtr = NULL;
            }
            else
            {
                CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_WRITE_RESEND;
            }
        }
        else
        {
            //
            // If a message receives a NACK during the address setup portion
            // of the EEPROM read, the code further below included in the
            // register access ready interrupt source code will generate a stop
            // condition. After the stop condition is received (here), set the
            // message status to try again. User may want to limit the number
            // of retries before generating an error.
            //
            if(CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP_BUSY)
            {
                CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_SEND_NOSTOP;
            }

            //
            // If completed message was reading EEPROM data, reset msg to
            // inactive state and read data from FIFO.
            //
            else if (CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_READ_BUSY)
            {
                // for(i=0; i < CurrentMsgPtr->NumOfBytes; i++)
                // {
                //     CurrentMsgPtr->pMsgBuffer[i] = I2C_READ_DATA();
                // }
                // 读取完成
//                I2caRegs.I2CFFRX.bit.RXFFIENA = 0;
                CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_INACTIVE;
                CurrentMsgPtr = NULL;
            }
        }
    }

    //
    // Interrupt source = Register Access Ready
    // This interrupt is used to determine when the EEPROM address setup 
    // portion of the read data communication is complete. Since no stop bit is 
    // commanded, this flag tells us when the message has been sent instead of 
    // the SCD flag. If a NACK is received, clear the NACK bit and command a 
    // stop. Otherwise, move on to the read data portion of the communication.
    //
    else if(IntSource == I2C_ARDY_ISRC)
    {
        if (I2C_NACK_CHECK() == 1) {
            I2C_SET_STOP_BIT();
            I2C_NACK_CLEAR();
        } else if (CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP_BUSY) {
            CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_RESTART;
            if (I2C_Read_Msg(&I2cMsgIn1) == I2C_SUCCESS) {
                // Update current message pointer and message status
                CurrentMsgPtr = &I2cMsgIn1;
                I2cMsgIn1.MsgStatus = I2C_MSGSTAT_READ_BUSY;
            }
            else
            {

            }
        }
    }
    else
    {
        // Generate some error due to invalid interrupt source
        __asm("   ESTOP0");
    }
}

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
void I2C_TX_FIFO_Isr(void)
{
    Uint16 i;
    Uint16 len;
    if (CurrentMsgPtr == NULL)
        return;
    if (CurrentMsgPtr->SentNum >= CurrentMsgPtr->NumOfBytes) {
        I2C_TXFIFO_INT_DISABLE();
        CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_INACTIVE;
    } else {
        // CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_WRITE_RESEND;
        if ((CurrentMsgPtr->NumOfBytes - CurrentMsgPtr->SentNum) > 14) {
            len = 14;
        } else {
            len = (CurrentMsgPtr->NumOfBytes - CurrentMsgPtr->SentNum);
            // I2caRegs.I2CFFTX.bit.TXFFIENA = 0;
            I2C_TXFIFO_INT_DISABLE();
        }
        for (i = 0; i < len;i++)
        {
            // I2caRegs.I2CDXR.all = *(msg->MsgBuffer + i);
            if (CurrentMsgPtr->pMsgBuffer != NULL)
                I2C_DATA_TRANSMIT(*(CurrentMsgPtr->pMsgBuffer + CurrentMsgPtr->SentNum));
            CurrentMsgPtr->SentNum++;
            if (CurrentMsgPtr->SentNum >= CurrentMsgPtr->NumOfBytes)
            {
                break;
            }
        }
    }
}

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
void I2C_RX_FIFO_Isr(void)
{
    Uint16 i;
//    Uint16 len;
    Uint16 data;
    if (CurrentMsgPtr == NULL)
        return;
    if (CurrentMsgPtr->SentNum >= CurrentMsgPtr->NumOfBytes) {
        CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_INACTIVE;
    } else {
        // CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_WRITE_RESEND;
//        if ((CurrentMsgPtr->NumOfBytes - CurrentMsgPtr->SentNum) > 14) {
//            len = 14;
//        } else {
//            len = (CurrentMsgPtr->NumOfBytes - CurrentMsgPtr->SentNum);
//        }
        if (CurrentMsgPtr->pMsgBuffer != NULL)
        {
            for (i = 0; i < 2; i++) {
                // I2caRegs.I2CDXR.all = *(msg->MsgBuffer + i);
                // I2C_DATA_TRANSMIT(*(CurrentMsgPtr->pMsgBuffer + CurrentMsgPtr->SentNum));
                // CurrentMsgPtr->SentNum++;
                data = I2C_READ_DATA();
                *(CurrentMsgPtr->pMsgBuffer + CurrentMsgPtr->SentNum) = data;
                CurrentMsgPtr->SentNum++;
                if (CurrentMsgPtr->SentNum >= CurrentMsgPtr->NumOfBytes) {
                    //读取完成
                    break;
                }
            }
        }
    }
}

/*********************************** (C) COPYRIGHT 2018 ******************* END OF FILE ***********/
