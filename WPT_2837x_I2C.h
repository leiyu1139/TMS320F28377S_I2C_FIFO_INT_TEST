/*
 * @File: WPT_2837x_I2C.h
 * @Author: panqunfeng  at <panqf1989@foxmail.com>
 * @Version: rev1.0.0
 * @Created Date: Thursday, January 3rd 2019, 4:16:48 pm
 * @Last Modified: Monday, 14th January 2019 10:49:25 am
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

/* Define to prevent recursive inclusion --------------------------------------------------------*/
#ifndef WPT_2837X_I2C_H_
#define WPT_2837X_I2C_H_
/* Includes -------------------------------------------------------------------------------------*/
#include "bsp.h"
/* public define --------------------------------------------------------------------------------*/
#define I2C_START_TRANSMIT_DATA 0x01
#define I2C_START_TRANSMIT_ADDR 0x02
#define I2C_START_RECEIVE 0x03
/* public typedef -------------------------------------------------------------------------------*/
/* public macro ---------------------------------------------------------------------------------*/
/* public variables -----------------------------------------------------------------------------*/
/** @defgroup
  * @{
  */
/**
  * @}
  */
/* public function prototypes -------------------------------------------------------------------*/
/* public functions -----------------------------------------------------------------------------*/

void WPT_InitConfig_I2C(void);
interrupt void i2c_int1a_fifo_isr(void);
interrupt void i2c_int1a_isr(void);
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
Uint16 I2CA_StopBitCheck(void);

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
void I2CA_SetSAddr(Uint16 addr);

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
Uint16 I2CA_BusBusyCheck(void);

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
void I2CA_DataTransmit(Uint16 data);

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
void I2CA_DataCount(Uint16 num);

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
void I2CA_Start(Uint8 status);

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
Uint16 I2CA_ReadData(void);

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
void I2CA_NACKClear(void);

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
void I2CA_SetStopBit(void);

/*
 **************************************************************************************************
 * @func        I2CA_NACKCheck
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
Uint16 I2CA_NACKCheck(void);

/*
 **************************************************************************************************
 * @func        I2CA_TXFIFOEnable
 * @brief       i2ca tx fifo interrupt enable
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
void I2CA_TXFIFOIntEnable(void);

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
void I2CA_TXFIFOIntDisable(void);

#endif /* WPT_2837X_I2C_H_ */
    /*********************************** (C) COPYRIGHT 2018 ******************* END OF FILE ***********/
