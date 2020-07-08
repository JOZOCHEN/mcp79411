/**
 * @file mcp79411_interface.h
 * @author jozochen (jozocyz@hotmail.com)
 * @brief 
 * @date 2020-02-11
 * @copyright Apache License 2.0
 *            jozochen (jozocyz@hotmail.com) Copyright (c) 2020
 */
#ifndef _MCP79411_INTERFACE_H
#define _MCP79411_INTERFACE_H

#ifndef NULL_PTR
#define NULL_PTR ((void*)0)
#endif

extern int mcp79411_rtc_iic_write(unsigned char *tx_buffer, short len);
extern int mcp79411_rtc_iic_read(unsigned char *rx_buffer, short len);
extern int mcp79411_eep_iic_write(unsigned char *tx_buffer, short len);
extern int mcp79411_eep_iic_read(unsigned char *rx_buffer, short len);
#endif
