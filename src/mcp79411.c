/**
 * @file mcp79411.c
 * @author jozochen (jozocyz@hotmail.com)
 * @brief 
 * @date 2020-02-12
 * @copyright Apache License 2.0
 *            jozochen (jozocyz@hotmail.com) Copyright (c) 2020
 */

#include "mcp79411.h"
#include "mcp79411_interface.h"

#define MCP79411_BUFFER_MAX     (256)
#define MCP79411_REG_ADDR_LEN   (1)

#define MCP79411_DEC_MAX        ((unsigned char)(0x99))
#define MCP79411_DECADE         (10)
#define MCP79411_4BIT_SHIFT     (4)
#define MCP79411_HIGH_4BIT_MASK    (0X0fu)
/**reg define**/
#define MCP79411_REG_RTCC_RTCSEC       ((unsigned char)(0x00))
#define MCP79411_REG_RTCC_RTCMIN       ((unsigned char)(0x01))
#define MCP79411_REG_RTCC_RTCHOUR      ((unsigned char)(0x02))
#define MCP79411_REG_RTCC_RTCWKDAY     ((unsigned char)(0x03))
#define MCP79411_REG_RTCC_RTCDATE      ((unsigned char)(0x04))
#define MCP79411_REG_RTCC_RTCMTH       ((unsigned char)(0x05))
#define MCP79411_REG_RTCC_RTCYEAR      ((unsigned char)(0x06))
#define MCP79411_REG_RTCC_CONTROL      ((unsigned char)(0x07))
#define MCP79411_REG_RTCC_OSCTRIM      ((unsigned char)(0x08))
#define MCP79411_REG_RTCC_EEUNLOCK     ((unsigned char)(0x09))
#define MCP79411_REG_RTCC_ALM0SEC      ((unsigned char)(0x0A))
#define MCP79411_REG_RTCC_ALM0MIN      ((unsigned char)(0x0B))
#define MCP79411_REG_RTCC_ALM0HOUR     ((unsigned char)(0x0C))
#define MCP79411_REG_RTCC_ALM0WKDAY    ((unsigned char)(0x0D))
#define MCP79411_REG_RTCC_ALM0DATE     ((unsigned char)(0x0E))
#define MCP79411_REG_RTCC_ALM0MTH      ((unsigned char)(0x0F))
#define MCP79411_REG_RTCC_ALM1SEC      ((unsigned char)(0x11))
#define MCP79411_REG_RTCC_ALM1MIN      ((unsigned char)(0x12))
#define MCP79411_REG_RTCC_ALM1HOUR     ((unsigned char)(0x13))
#define MCP79411_REG_RTCC_ALM1WKDAY    ((unsigned char)(0x14))
#define MCP79411_REG_RTCC_ALM1DATE     ((unsigned char)(0x15))
#define MCP79411_REG_RTCC_ALM1MTH      ((unsigned char)(0x16))
#define MCP79411_REG_RTCC_PWRDNMIN     ((unsigned char)(0x18))
#define MCP79411_REG_RTCC_PWRDNHOUR    ((unsigned char)(0x19))
#define MCP79411_REG_RTCC_PWRDNDATE    ((unsigned char)(0x1A))
#define MCP79411_REG_RTCC_PWRDNMTH     ((unsigned char)(0x1B))
#define MCP79411_REG_RTCC_PWRUPMIN     ((unsigned char)(0x1C))
#define MCP79411_REG_RTCC_PWRUPHOUR    ((unsigned char)(0x1D))
#define MCP79411_REG_RTCC_PWRUPDATE    ((unsigned char)(0x1E))
#define MCP79411_REG_RTCC_PWRUPMTH     ((unsigned char)(0x1F))


typedef union {
    struct{
        char SQWFS   :2;
        char CRSTRIM :1;
        char EXTOSC  :1;
        char ALM0EN  :1;
        char ALM1EN  :1;
        char SQWEN   :1;
        char OUT     :1;
    }bits;
    unsigned char byte;
}mcp79411_CONTROL;

typedef union {
    struct{
        char TRIMVA  :7;
        char SIGN    :1;
    }bits;
    unsigned char byte;
}mcp79411_OSCTRIM;

typedef union {
    struct {
        struct{
            char SEC :7;
            char ST  :1;
        }RTCSEC_bits;
        struct{
            char MIN   :7;
            char RSVD  :1;
        }RTCMIN_bits;
        struct{
            char HOUR   :5;
            char AM_PM  :1;
            char b12_24  :1;
            char RSVD   :1;
        }RTCHOUR_bits;
        struct{
            char WKDAY   :3;
            char VBATEN  :1;
            char PWRFAIL :1;
            char OSCRUN  :1;
            char RSVD    :2;
        }RTCWKDAY_bits;
        struct{
            char DATE    :6;
            char RSVD    :2;
        }RTCDATE_bits;
        struct{
            char MTH     :5;
            char LPYR    :1;
            char RSVD    :2;
        }RTCMTH_bits;
        struct{
            char YEAR :8;
        }RTCYEAR_bits;
    }regs;
    unsigned char bytes[7];
}mcp79411_TIME_KEEPING;

typedef union{
    struct{
        struct{
            char SEC     :7;
            char RSVD    :1;
        }ALMXSEC_bits;
        struct{
            char MIN     :7;
            char RSVD    :1;
        }ALMXMIN_bits;    
        struct{
            char HOUR    :5;
            char AM_PM   :1;
            char b12_24  :1;
            char RSVD    :1;
        }ALMXHOUR_bits;
        struct{
            char WKDAY   :3;
            char ALMXIF  :1;
            char ALMXMSK :3;
            char ALMPOL  :1;
        }ALMXWKDAY_bits;
        struct{
            char DATE;
        }ALMXDATE_bits;
        struct{
            char MTH    :5;
            char RSVD   :3;
        }ALMXMTH_bits;
    }regs;
    unsigned char bytes[6];
}mcp79411_ALARMS;

typedef struct{
    struct{
        unsigned char tx_buffer[MCP79411_BUFFER_MAX];
        unsigned char rx_buffer[MCP79411_BUFFER_MAX];
    }buffers;
}mcp79411_obj;

static mcp79411_obj mcp79411;

static int mcp79411_rtc_reg_read(unsigned char reg_addr, unsigned char* rx_buffer, short len);
static int mcp79411_rtc_reg_write(unsigned char reg_addr, unsigned char* tx_buffer, short len);
static unsigned char mcp79411_dec2bcd(unsigned char dec);
static unsigned char mcp79411_bcd2dec(unsigned char bcd);

static unsigned char mcp79411_dec2bcd(unsigned char dec)
{
    unsigned char l_highHalfByte = 0;
    unsigned char l_lowHalfByte  = 0;
    unsigned char ret = 0x00;

    if(dec > MCP79411_DEC_MAX)
    {
        /*Invalid decimal data*/
        ret = 0x00;
    }
    else
    {
        /*Convert decimal data to dec data*/
        l_highHalfByte = dec / MCP79411_DECADE;
        l_lowHalfByte  = dec % MCP79411_DECADE;

        ret = (unsigned char)(l_highHalfByte << MCP79411_4BIT_SHIFT) | (l_lowHalfByte);
    }

    return ret;
}

static unsigned char mcp79411_bcd2dec(unsigned char bcd)
{
    unsigned char l_highHalfByte = 0;
    unsigned char l_lowHalfByte  = 0;
    unsigned char ret = 0x00;

    if(bcd > MCP79411_DEC_MAX)
    {
        /*Invalid bcd data*/
        ret = 0x00;
    }
    else
    {
        /*Convert bcd data to decimal data*/
        l_highHalfByte = bcd >> MCP79411_4BIT_SHIFT;
        l_lowHalfByte  = bcd & MCP79411_HIGH_4BIT_MASK;

        ret = (l_highHalfByte * MCP79411_DECADE) + l_lowHalfByte;
    }

    return ret;    
}

static int mcp79411_rtc_reg_read(unsigned char reg_addr, unsigned char* rx_buffer, short len)
{
    int ret = -1;

    if( (rx_buffer == NULL_PTR) || (len <= 0) || (len >= MCP79411_BUFFER_MAX)){
        ret = -1;
    }else{
        ret = mcp79411_rtc_iic_write(&reg_addr, sizeof(reg_addr));
        if(ret == 0){
            ret = mcp79411_rtc_iic_read(rx_buffer, len);
        }else{
            /*nothing*/
        }
    }

    return ret;    
}

static int mcp79411_rtc_reg_write(unsigned char reg_addr, unsigned char* tx_buffer, short len)
{
	unsigned char* ptx = &mcp79411.buffers.tx_buffer[0];
    int ret = -1;
    int i = 0;

    if( (tx_buffer == NULL_PTR) || (len <= 0) || (len >= MCP79411_BUFFER_MAX)){
        ret = -1;
    }else{
        ptx[0] = reg_addr;
        for(i = 0; i < len; i++){
            ptx[i + MCP79411_REG_ADDR_LEN] = tx_buffer[i];
        }
        ret = mcp79411_rtc_iic_write(ptx, len + MCP79411_REG_ADDR_LEN);
    }

    return ret;    
}

int mcp79411_set_time(mcp79411_time* time)
{
    mcp79411_TIME_KEEPING reg_time;
    int ret = -1;
    unsigned int i = 0;

    //TODO:check arg time
    if(time == NULL_PTR){
        ret = -1;
    }else{
        for( i = 0; i < sizeof(reg_time); i++){
            reg_time.bytes[i] = 0x00;
        }

        reg_time.regs.RTCSEC_bits.ST = 1;
        reg_time.regs.RTCSEC_bits.SEC = mcp79411_dec2bcd(time->sec);
        reg_time.regs.RTCMIN_bits.MIN = mcp79411_dec2bcd(time->min);
        reg_time.regs.RTCHOUR_bits.AM_PM = 0;
        reg_time.regs.RTCHOUR_bits.b12_24 = 0;
        reg_time.regs.RTCHOUR_bits.HOUR = mcp79411_dec2bcd(time->hour);
        reg_time.regs.RTCWKDAY_bits.PWRFAIL = 1;
        reg_time.regs.RTCWKDAY_bits.VBATEN = 0;
        reg_time.regs.RTCWKDAY_bits.WKDAY = mcp79411_dec2bcd(time->wkday);
        reg_time.regs.RTCDATE_bits.DATE = mcp79411_dec2bcd(time->date);
        reg_time.regs.RTCMTH_bits.MTH = mcp79411_dec2bcd(time->mth);
        reg_time.regs.RTCYEAR_bits.YEAR = mcp79411_dec2bcd(time->year);
        ret = mcp79411_rtc_reg_write(MCP79411_REG_RTCC_RTCSEC, reg_time.bytes, sizeof(reg_time));
    }

    return ret;
}

int mcp79411_get_time(mcp79411_time* time)
{
    mcp79411_TIME_KEEPING reg_time;
    int ret = -1;

    //TODO:check arg time
    if(time == NULL_PTR){
        ret = -1;
    }else{
        ret = mcp79411_rtc_reg_read(MCP79411_REG_RTCC_RTCSEC, reg_time.bytes, sizeof(reg_time));
        if(ret == 0){
            time->sec = mcp79411_bcd2dec(reg_time.regs.RTCSEC_bits.SEC);
            time->min = mcp79411_bcd2dec(reg_time.regs.RTCMIN_bits.MIN);
            time->hour = mcp79411_bcd2dec(reg_time.regs.RTCHOUR_bits.HOUR);
            time->wkday = mcp79411_bcd2dec(reg_time.regs.RTCWKDAY_bits.WKDAY);
            time->date = mcp79411_bcd2dec(reg_time.regs.RTCDATE_bits.DATE);
            time->mth = mcp79411_bcd2dec(reg_time.regs.RTCMTH_bits.MTH);
            time->year = mcp79411_bcd2dec(reg_time.regs.RTCYEAR_bits.YEAR);            
        }else{
            /*nohitng*/
        }
    }

    return ret;
}

int mcp79411_set_alarm(mcp79411_alarm_channel chnl, mcp79411_alarm_mode mode,
 mcp79411_alarm *alarm)
{
    int ret = -1;
    unsigned int i = 0;
    mcp79411_ALARMS reg_alarms;
    mcp79411_CONTROL reg_CONTROL = {{0}};
    unsigned char alarm_reg_start_addr = MCP79411_REG_RTCC_ALM0SEC;
    unsigned char reg_addr_WKDAY = MCP79411_REG_RTCC_ALM0WKDAY;

    if( (chnl >= MCP79411_ALRM_CHANNEL_MAX) ||
        (mode >= MCP79411_ALRM_MODE_MAX) ||
        (alarm == NULL_PTR)){
            ret = -1;
    }else{
        if(chnl == MCP79411_ALRM_CHANNEL_1){
            alarm_reg_start_addr = MCP79411_REG_RTCC_ALM1SEC;
            reg_addr_WKDAY = MCP79411_REG_RTCC_ALM1WKDAY;
        }else{
            alarm_reg_start_addr = MCP79411_REG_RTCC_ALM0SEC;
            reg_addr_WKDAY = MCP79411_REG_RTCC_ALM0WKDAY;
        }
        for( i = 0; i < sizeof(reg_alarms); i++){
            reg_alarms.bytes[i] = 0x00;
        }
        //config alarm value
        reg_alarms.regs.ALMXSEC_bits.SEC = mcp79411_dec2bcd(alarm->sec);
        reg_alarms.regs.ALMXMIN_bits.MIN = mcp79411_dec2bcd(alarm->min);
        reg_alarms.regs.ALMXHOUR_bits.AM_PM = 0;
        reg_alarms.regs.ALMXHOUR_bits.b12_24 = 0;
        reg_alarms.regs.ALMXHOUR_bits.HOUR = mcp79411_dec2bcd(alarm->hour);
        reg_alarms.regs.ALMXWKDAY_bits.ALMPOL = 0;
        reg_alarms.regs.ALMXWKDAY_bits.ALMXMSK = (char)mode;
        reg_alarms.regs.ALMXWKDAY_bits.WKDAY = mcp79411_dec2bcd(alarm->wkday);
        reg_alarms.regs.ALMXDATE_bits.DATE = mcp79411_dec2bcd(alarm->date);
        reg_alarms.regs.ALMXMTH_bits.MTH = mcp79411_dec2bcd(alarm->mth);
        ret = mcp79411_rtc_reg_write(alarm_reg_start_addr, reg_alarms.bytes, sizeof(reg_alarms));
        if(ret == 0){
            //ensure ALMxIF flag is cleared
            ret = mcp79411_rtc_reg_write(reg_addr_WKDAY, (unsigned char*)&(reg_alarms.regs.ALMXWKDAY_bits),
             sizeof(reg_alarms.regs.ALMXWKDAY_bits));
            if(ret == 0){
                //enable ALMxEN
                ret = mcp79411_rtc_reg_read(MCP79411_REG_RTCC_CONTROL, &reg_CONTROL.byte, sizeof(reg_CONTROL));
                if(ret == 0){
                    if(chnl == MCP79411_ALRM_CHANNEL_1){
                        reg_CONTROL.bits.ALM1EN = 1;
                    }else{
                        reg_CONTROL.bits.ALM0EN = 1;
                    }
                }
                ret = mcp79411_rtc_reg_write(MCP79411_REG_RTCC_CONTROL, &reg_CONTROL.byte, sizeof(reg_CONTROL));
            }
        }
    }

    return ret;
}

int mcp79411_stop_alarm(mcp79411_alarm_channel chnl)
{
    mcp79411_CONTROL reg_CONTROL = {{0}};
    int ret = -1;

    ret = mcp79411_rtc_reg_read(MCP79411_REG_RTCC_CONTROL, &reg_CONTROL.byte, sizeof(reg_CONTROL));
    if(ret == 0){
        if(chnl == MCP79411_ALRM_CHANNEL_1){
            reg_CONTROL.bits.ALM1EN = 0;
        }else{
            reg_CONTROL.bits.ALM0EN = 0;
        }
        ret = mcp79411_rtc_reg_write(MCP79411_REG_RTCC_CONTROL, &reg_CONTROL.byte, sizeof(reg_CONTROL));
    }

    return ret;
}

void mcp79411_init(void)
{
    mcp79411_CONTROL reg_CONTROL;
    mcp79411_OSCTRIM reg_OSCTRIM;
    int i = 0;

    for(i = 0; i < MCP79411_BUFFER_MAX; i++){
        mcp79411.buffers.tx_buffer[i] = 0;
        mcp79411.buffers.rx_buffer[i] = 0;
    }

    reg_CONTROL.byte = 0;
    reg_CONTROL.bits.SQWFS = 0;
    reg_CONTROL.bits.CRSTRIM = 0;
    reg_CONTROL.bits.EXTOSC = 1;
    reg_CONTROL.bits.ALM0EN = 0;
    reg_CONTROL.bits.ALM1EN = 0;
    reg_CONTROL.bits.SQWEN = 0;
    reg_CONTROL.bits.OUT = 1;
    (void)mcp79411_rtc_reg_write(MCP79411_REG_RTCC_CONTROL, &reg_CONTROL.byte, sizeof(reg_CONTROL));

    reg_OSCTRIM.byte = 0;
    (void)mcp79411_rtc_reg_write(MCP79411_REG_RTCC_OSCTRIM, &reg_OSCTRIM.byte, sizeof(reg_OSCTRIM));
}
