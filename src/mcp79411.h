/**
 * @file mcp79411.h
 * @author jozochen (jozocyz@hotmail.com)
 * @brief 
 * @date 2020-02-12
 * @copyright Apache License 2.0
 *            jozochen (jozocyz@hotmail.com) Copyright (c) 2020
 */
#ifndef _MCP79411_H
#define _MCP79411_H

typedef struct {
    char sec;
    char min;
    char hour;
    char wkday;
    char date;
    char mth;
    char year;
}mcp79411_time;

typedef struct{
    char sec;
    char min;
    char hour;
    char wkday;
    char date;
    char mth;    
}mcp79411_alarm;

typedef enum{
    MCP79411_ALRM_MODE_SEC = 0x00,
    MCP79411_ALRM_MODE_MIN = 0x01,
    MCP79411_ALRM_MODE_HOUR = 0x02,
    MCP79411_ALRM_MODE_WKDAY = 0x03,
    MCP79411_ALRM_MODE_DATE = 0x04,
    MCP79411_ALRM_MODE_ALL = 0x07,
    MCP79411_ALRM_MODE_MAX
}mcp79411_alarm_mode;

typedef enum{
    MCP79411_ALRM_CHANNEL_0 = 0,  
    MCP79411_ALRM_CHANNEL_1,
    MCP79411_ALRM_CHANNEL_MAX,  
}mcp79411_alarm_channel;

extern void mcp79411_init(void);
extern int mcp79411_set_time(mcp79411_time* time);
extern int mcp79411_get_time(mcp79411_time* time);
extern int mcp79411_set_alarm(mcp79411_alarm_channel chnl, mcp79411_alarm_mode mode,
 mcp79411_alarm *alarm);
extern int mcp79411_stop_alarm(mcp79411_alarm_channel chnl);
#endif
