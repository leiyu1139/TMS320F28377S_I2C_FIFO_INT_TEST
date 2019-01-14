/*
 * @File: 2837x_I2C.c
 * @Author: panqunfeng  at <panqf1989@foxmail.com>
 * @Version: rev1.0.0
 * @Created Date: Thursday, January 3rd 2019, 4:16:48 pm
 * @Last Modified: Monday, 14th January 2019 2:35:00 pm
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








/* Includes --------------------------------------------------------------------------------------*/
#include "2837x_I2C.h"
#include "i2c_api.h"
/* Private define --------------------------------------------------------------------------------*/
#define I2C_SLAVE_ADDR 0x50
/* Private typedef -------------------------------------------------------------------------------*/
/* Private macro ---------------------------------------------------------------------------------*/
/* Private variables -----------------------------------------------------------------------------*/
/* Private function prototypes -------------------------------------------------------------------*/
/* Private functions -----------------------------------------------------------------------------*/
static void InitI2CGpio(void);
static void Init_I2C(void);
/************************************************************************************
 * Function Name : InitConfig_I2C
 * Description   : This function initializes and config adc,include adca,adcb,adcc;
 * Arguments     : None
 * Return Value  : None
 ************************************************************************************/
void InitConfig_I2C(void)
{
    InitI2CGpio();
    Init_I2C();
}

/************************************************************************************
* Function Name : InitI2CGpio
* Description   : This function initializes the gpio of the I2C;
* Arguments     : None
* Return Value  : None
************************************************************************************/
static void InitI2CGpio(void)
{
    GPIO_SetupPinMux(42, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinMux(43, GPIO_MUX_CPU1, 6);
}

/************************************************************************************
* Function Name : Init_I2C
* Description   : This function initializes the module of the I2C;
* Arguments     : None
* Return Value  : None
************************************************************************************/
static void Init_I2C(void)
{
    EALLOW;

    I2caRegs.I2CSAR.all = 0x0050; // Slave address - EEPROM control code
    // I2caRegs.I2CSAR.all = 0x0000;        // Slave address - EEPROM control code

    //I2caRegs.I2CPSC.all = 6;         // Prescaler - need 7-12 Mhz on module clk
    //I2caRegs.I2CCLKL = 10;           // NOTE: must be non zero
    //I2caRegs.I2CCLKH = 5;            // NOTE: must be non zero

//    I2caRegs.I2CPSC.all = 0x006; // Prescaler - need 7-12 Mhz on module clk
//    I2caRegs.I2CCLKL = 0x000b; // NOTE: must be non zero
//    I2caRegs.I2CCLKH = 0x000b; // NOTE: must be non zero
    I2caRegs.I2CPSC.all = 16;         // Prescaler - need 7-12 Mhz on module clk
    I2caRegs.I2CCLKL = 10;            // NOTE: must be non zero
    I2caRegs.I2CCLKH = 5;             // NOTE: must be non zero
    I2caRegs.I2CIER.all = 0x24;       // Enable SCD & ARDY __interrupts

    I2caRegs.I2CMDR.all = 0x0020; // Take I2C out of reset
        // Stop I2C when suspended
//    I2caRegs.I2CFFTX.all = 0x6000;    // Enable FIFO mode and TXFIFO
    I2caRegs.I2CFFTX.all = 0x6042; // Enable FIFO mode and TXFIFO
    I2caRegs.I2CFFRX.all = 0x2042; // Enable RXFIFO, clear RXFFINT,
    I2caRegs.I2CFFRX.bit.RXFFIENA = 1;
    I2caRegs.I2CFFRX.bit.RXFFINTCLR = 1;
    I2caRegs.I2CFFTX.bit.TXFFINTCLR = 1;
    EDIS;
    return;
}

interrupt void i2c_int1a_fifo_isr(void)
{
    //
    // If receive FIFO interrupt flag is set, read data
    //
    if((I2caRegs.I2CFFTX.bit.TXFFINT == 1) && (I2caRegs.I2CFFTX.bit.TXFFIENA == 1))
    {
        I2C_TX_FIFO_Isr();
        I2caRegs.I2CFFTX.bit.TXFFINTCLR = 1; //clears the TXFFINT flag.
    }
    //
    // If transmit FIFO interrupt flag is set, put data in the buffer
    //
    if((I2caRegs.I2CFFRX.bit.RXFFINT == 1) && (I2caRegs.I2CFFRX.bit.RXFFIENA == 1))
    {
        I2C_RX_FIFO_Isr();
        I2caRegs.I2CFFRX.bit.RXFFINTCLR = 1; //clears the RXFFINT flag.
    }
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}

interrupt void i2c_int1a_isr(void) // I2C-A
{
    Uint16 IntSource;

    // Read __interrupt source
    IntSource = I2caRegs.I2CISRC.all;
    I2C_IER_Isr(IntSource);

    // Enable future I2C (PIE Group 8) __interrupts
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}

/*
 **************************************************************************************************
 * @func        I2CA_StopBitCheck
 * @brief       check i2c stop bit
 * @param 
 * @retval      STP has been set by the device to generate a STOP
 *              condition when the internal data counter of the I2C module counts
 *              down to 0.
 * @remarks
 * @note
 * @see
 * @auther         pqf.pan at <panqf1989@foxmail.com>
 * @created date   Wednesday, January, 01 th 2019, 21:47:50
 * @last modified  Wednesday, January, 01 th 2019, 21:47:50
 * @modified by    
 * @version        rev1.0.0
 */
Uint16 I2CA_StopBitCheck(void)
{
    return I2caRegs.I2CMDR.bit.STP;
}

/*
 **************************************************************************************************
 * @func        I2CA_SetSAddr
 * @brief       i2c set slave address
 * @param       addr: slave address
 * @retval  
 * @remarks
 * @note
 * @see
 * @auther         pqf.pan at <panqf1989@foxmail.com>
 * @created date   Wednesday, January, 01 th 2019, 21:48:03
 * @last modified  Wednesday, January, 01 th 2019, 21:48:03
 * @modified by    
 * @version        rev1.0.0
 */
void I2CA_SetSAddr(Uint16 addr)
{
    I2caRegs.I2CSAR.all = addr;
}

/*
 **************************************************************************************************
 * @func        I2CA_BusBusyCheck
 * @brief       i2c bus busy check
 * @param 
 * @retval      0:inactive
 *              1:busy
 * @remarks
 * @note
 * @see
 * @auther         pqf.pan at <panqf1989@foxmail.com>
 * @created date   Wednesday, January, 01 th 2019, 21:48:27
 * @last modified  Wednesday, January, 01 th 2019, 21:48:27
 * @modified by    
 * @version        rev1.0.0
 */
Uint16 I2CA_BusBusyCheck(void)
{
    return I2caRegs.I2CSTR.bit.BB;
}

/*
 **************************************************************************************************
 * @func        I2CA_DataTransmit
 * @brief       i2c transmit an byte data
 * @param       data:The data to be sent
 * @retval  
 * @remarks
 * @note
 * @see
 * @auther         pqf.pan at <panqf1989@foxmail.com>
 * @created date   Wednesday, January, 01 th 2019, 21:48:33
 * @last modified  Wednesday, January, 01 th 2019, 21:48:33
 * @modified by    
 * @version        rev1.0.0
 */
void I2CA_DataTransmit(Uint16 data)
{
    I2caRegs.I2CDXR.bit.DATA = (data & 0xFF);
}

/*
 **************************************************************************************************
 * @func        I2CA_DataCount
 * @brief       set the number of data to sent
 * @param       num:the number of data to sent
 * @retval  
 * @remarks
 * @note
 * @see
 * @auther         pqf.pan at <panqf1989@foxmail.com>
 * @created date   Wednesday, January, 01 th 2019, 21:48:39
 * @last modified  Wednesday, January, 01 th 2019, 21:48:39
 * @modified by    
 * @version        rev1.0.0
 */
void I2CA_DataCount(Uint16 num)
{
    I2caRegs.I2CCNT = num;
}

/*
 **************************************************************************************************
 * @func        I2CA_Start
 * @brief       i2c start transmit
 * @param       status:I2C_START_TRANSMIT_DATA 
 *                      I2C_START_TRANSMIT_ADDR
 *                      I2C_START_RECEIVE
 * @retval  
 * @remarks
 * @note
 * @see
 * @auther         pqf.pan at <panqf1989@foxmail.com>
 * @created date   Wednesday, January, 01 th 2019, 21:48:45
 * @last modified  Wednesday, January, 01 th 2019, 21:48:45
 * @modified by    
 * @version        rev1.0.0
 */
void I2CA_Start(Uint8 status)
{
    I2caRegs.I2CMDR.bit.MST = 1;
    I2caRegs.I2CMDR.bit.STT = 1;
    if (status == I2C_START_TRANSMIT_DATA) {
//         I2caRegs.I2CMDR.all = 0x6E20;// TRX为1 STP FREE  MST  STT
        I2caRegs.I2CMDR.bit.TRX = 1;
        I2caRegs.I2CMDR.bit.STP = 1;
        I2caRegs.I2CMDR.bit.FREE = 1;
    } else if (status == I2C_START_TRANSMIT_ADDR) {
        // Send data to setup EEPROM address
        // I2caRegs.I2CMDR.all = 0x2620; // TRX为1  MST STT
        I2caRegs.I2CMDR.bit.TRX = 1;
    } else if (status == I2C_START_RECEIVE) {
        // Send restart as master receiver
//         I2caRegs.I2CMDR.all = 0x2C20; // STP  MST STT TRX为0
        I2caRegs.I2CMDR.bit.STP = 1;
        I2caRegs.I2CMDR.bit.TRX = 0;
        I2caRegs.I2CMDR.bit.FREE = 1;
    }
}

/*
 **************************************************************************************************
 * @func        I2CA_ReadData
 * @brief       read the i2c bus  data
 * @param       
 * @retval      received data
 * @remarks
 * @note
 * @see
 * @auther         pqf.pan at <panqf1989@foxmail.com>
 * @created date   Wednesday, January, 01 th 2019, 21:48:52
 * @last modified  Wednesday, January, 01 th 2019, 21:48:52
 * @modified by    
 * @version        rev1.0.0
 */
Uint16 I2CA_ReadData(void)
{
    return I2caRegs.I2CDRR.all;
}

/*
 **************************************************************************************************
 * @func        I2CA_NACKClear
 * @brief       clear nack bit 
 * @param 
 * @retval  
 * @remarks
 * @note
 * @see
 * @auther         pqf.pan at <panqf1989@foxmail.com>
 * @created date   Wednesday, January, 01 th 2019, 21:48:57
 * @last modified  Wednesday, January, 01 th 2019, 21:48:57
 * @modified by    
 * @version        rev1.0.0
 */
void I2CA_NACKClear(void)
{
    I2caRegs.I2CSTR.all = I2C_CLR_NACK_BIT;
}

/*
 **************************************************************************************************
 * @func        I2CA_SetStopBit
 * @brief       i2c set stop bit
 * @param
 * @retval  
 * @remarks
 * @note
 * @see
 * @auther         pqf.pan at <panqf1989@foxmail.com>
 * @created date   Wednesday, January, 01 th 2019, 21:49:03
 * @last modified  Wednesday, January, 01 th 2019, 21:49:03
 * @modified by    
 * @version        rev1.0.0
 */
void I2CA_SetStopBit(void)
{
    I2caRegs.I2CMDR.bit.STP = 1;
}

/*
 **************************************************************************************************
 * @func        I2CNACKCheck
 * @brief       check the nack bit 
 * @param 
 * @retval      0:NACK bit not received.
 *              1:NACK bit received.
 * @remarks
 * @note
 * @see
 * @auther         pqf.pan at <panqf1989@foxmail.com>
 * @created date   Wednesday, January, 01 th 2019, 21:49:08
 * @last modified  Wednesday, January, 01 th 2019, 21:49:08
 * @modified by    
 * @version        rev1.0.0
 */
Uint16 I2CA_NACKCheck(void)
{
    return I2caRegs.I2CSTR.bit.NACK;
}

/*
 **************************************************************************************************
 * @func        I2CA_TXFIFOEnable
 * @brief       i2ca tx fifo interrupt enable
 * @param 
 * @retval  
 * @remarks
 * @note        注意操作顺序,一定要先开启ENA,在进行CLR RST.否则会导致fifo无法进入中断.
 * @see
 * @auther         pqf.pan at <panqf1989@foxmail.com>
 * @created date   Friday, January, 01 th 2019, 20:46:10
 * @last modified  Friday, January, 01 th 2019, 20:46:10
 * @modified by    
 * @version        rev1.0.0
 */
void I2CA_TXFIFOIntEnable(void)
{
    I2caRegs.I2CFFTX.bit.TXFFIENA = 1;
    I2caRegs.I2CFFTX.bit.TXFFINTCLR = 1;
    I2caRegs.I2CFFTX.bit.TXFFRST = 1;
}

/*
 **************************************************************************************************
 * @func        I2CA_TXFIFOIntDisable
 * @brief       i2ca tx fifo interrupt disable
 * @param 
 * @retval  
 * @remarks
 * @note
 * @see
 * @auther         pqf.pan at <panqf1989@foxmail.com>
 * @created date   Friday, January, 01 th 2019, 20:46:10
 * @last modified  Friday, January, 01 th 2019, 20:46:10
 * @modified by    
 * @version        rev1.0.0
 */
void I2CA_TXFIFOIntDisable(void)
{
    I2caRegs.I2CFFTX.bit.TXFFIENA = 0;
}



/*********************************** (C) COPYRIGHT 2018 ******************* END OF FILE ***********/
