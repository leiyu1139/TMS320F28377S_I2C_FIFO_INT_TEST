# TMS320F28377S_I2C_FIFO_INT_TEST

TMS320F28377S i2c fifo中断收发代码

## 使用说明

- 首先进行I2C的初始化,

```c
void main(void)
{
    Uint16 buffer[256];
    ...
    InitConfig_I2C();
    PieVectTable.I2CA_INT = &i2c_int1a_isr;
    PieVectTable.I2CA_FIFO_INT = &i2c_int1a_fifo_isr;  
    // Enable I2C interrupt 1 in the PIE: Group 8 interrupt 1 I2CA
    PieCtrlRegs.PIEIER8.bit.INTx1 = 1;    
    PieCtrlRegs.PIEIER8.bit.INTx2 = 1;
    //Enable CPU INT5 which is connected to SCIC-RX-INT and I2CA-INT ;
    IER |= M_INT8; 
    // Enable Global interrupt INTM
    EINT; 
    ERTM;
    ...
    for(;;)
    {
        //写入
        if (I2C_Write_Multi_Bytes(0x00, (Uint16*)buffer, 256) == I2C_SUCCESS)
        {
            ...
        }
        ...
        //读取
        I2C_Read_Multi_Bytes(0x00, 256);

        while (I2C_Get_Read_Data(buffer) != I2C_SUCCESS) {
            ;
        }
        ...
    }
}
```

- 另外开个定时器,1-10ms周期,然后定时调用

```c
    //I2C 数据处理
    I2C_Loop();
```

- 同时要注意 I2C_OPERATION_INTERVAL 的值.

**ok,尽情的干别的活吧...**

## Feature

- none

## 联系人

- **FilingBy:** *panqunfeng*

- **MailBox:** *[panqf1989@foxmail.com](panqf1989@foxmail.com)*

- **Signature:** *一切有为法 如梦幻泡影 如露亦如电 应作如是观*