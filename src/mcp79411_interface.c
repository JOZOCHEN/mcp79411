/**
 * @file mcp79411_interface.c
 * @author jozochen (jozocyz@hotmail.com)
 * @brief 
 * @date 2020-02-11
 * @copyright Apache License 2.0
 *            jozochen (jozocyz@hotmail.com) Copyright (c) 2020
 */
#include "mcp79411_interface.h"
#include "iic_init.h"

int mcp79411_rtc_iic_write(unsigned char *tx_buffer, short len)
{
    int ret = -1;
    int retry_cnt = 1000;

    while ( (IfxI2c_I2c_write(&iic_obj[IIC_CHANNEL_MCP79411].i2cDev[0], tx_buffer, len) ==
     IfxI2c_I2c_Status_nak) && (retry_cnt--) ) {}

    if(retry_cnt > 0){
        ret = 0;
    }

    return ret;
}

int mcp79411_rtc_iic_read(unsigned char *rx_buffer, short len)
{
    int ret = -1;
    int retry_cnt = 1000;

    while ( (IfxI2c_I2c_read(&iic_obj[IIC_CHANNEL_MCP79411].i2cDev[0], rx_buffer, len) ==
     IfxI2c_I2c_Status_nak) && (retry_cnt--) ){}

    if(retry_cnt > 0){
        ret = 0;
    }

    return ret;
}

int mcp79411_eep_iic_write(unsigned char *tx_buffer, short len)
{
    int ret = -1;
    int retry_cnt = 1000;

    while ( (IfxI2c_I2c_write(&iic_obj[IIC_CHANNEL_MCP79411].i2cDev[1], tx_buffer, len) ==
     IfxI2c_I2c_Status_nak) && (retry_cnt--) ) {}

    if(retry_cnt != 0){
        ret = 0;
    }

    return ret;
}

int mcp79411_eep_iic_read(unsigned char *rx_buffer, short len)
{
    int ret = -1;
    int retry_cnt = 1000;

    while ( (IfxI2c_I2c_read(&iic_obj[IIC_CHANNEL_MCP79411].i2cDev[1], rx_buffer, len) ==
     IfxI2c_I2c_Status_nak) && (retry_cnt--) ){}

    if(retry_cnt != 0){
        ret = 0;
    }

    return ret;
}
