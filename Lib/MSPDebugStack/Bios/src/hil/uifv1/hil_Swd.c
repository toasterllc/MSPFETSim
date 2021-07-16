#include "hw_compiler_specific.h"
#include "HalGlobalVars.h"
#include "arch.h"
#include "edt.h"
#include <stdio.h>

static struct jtag _Jtag = {0};

#define SWD_DELAY   { _NOP();}

// DAP Transfer Response
#define DAP_TRANSFER_OK                 (1<<0)
#define DAP_TRANSFER_WAIT               (1<<1)
#define DAP_TRANSFER_FAULT              (1<<2)
#define DAP_TRANSFER_ERROR              (1<<3)
#define DAP_TRANSFER_MISMATCH           (1<<4)


//#pragma inline=forced
void SWDTCKset1()
{ (*_Jtag.Out) |=  _Jtag.TCK;}

//#pragma inline=forced
void SWDTCKset0()
{ (*_Jtag.Out) &= ~_Jtag.TCK;}

//#pragma inline=forced
void SWDIOset1TckCycle()
{
    (*_Jtag.Out) |=  _Jtag.TDI;
    SWDTCKset0();
    SWD_DELAY;
    SWDTCKset1();
}

//#pragma inline=forced
void SWDIOset0TckCycle()
{
    (*_Jtag.Out) &=  ~_Jtag.TDI;
    SWDTCKset0();
    SWD_DELAY;
    SWDTCKset1();
}

//#pragma inline=forced
void SWDTckCycle()
{
    SWDTCKset0();
    SWD_DELAY;
    SWDTCKset1();
    SWD_DELAY;
}
//#pragma inline=forced
void SWDIOwriteBit(unsigned char bit)
{
    if(bit & 0x1)
    {
        (*_Jtag.Out) |=  _Jtag.TDI;
    }
    else
    {
        (*_Jtag.Out) &= ~_Jtag.TDI;
    }
    SWDTCKset0();
    SWD_DELAY;;
    SWDTCKset1();
}

// turn around direction before
//#pragma inline=forced
unsigned char ScanTDO()
{
    unsigned char val = 0;
    SWDTCKset0();
    SWD_DELAY;
    SWD_DELAY;

    if(*_Jtag.In  & _Jtag.TDO)
    {
        val = 1;
    }
    else
    {
        val = 0;
    }
    SWDTCKset1();
    __enable_interrupt();
    return val;
}

void hil_Swd_InitJtag(struct jtag tmp)
{
    _Jtag = tmp;
}

unsigned char Swd_TransferData(unsigned char regiser, unsigned long* data)
{
    volatile unsigned char ack = DAP_TRANSFER_ERROR;
    static unsigned long bit = 0;
    static unsigned long val = 0;
    static unsigned long parity = 0;

    bit = 0;
    val = 0;
    parity = 0;

    ///host sends request
    SWDIOwriteBit(1);                      //Start Bit
    bit = regiser >> 0;
    SWDIOwriteBit(bit);                    //APnDP Bit
    parity += bit;
    bit = regiser >> 1;
    SWDIOwriteBit(bit);                    //RnW Bit
    parity += bit;
    bit = regiser >> 2;
    SWDIOwriteBit(bit);                    //A2 Bit
    parity += bit;
    bit = regiser >> 3;
    SWDIOwriteBit(bit);                    //A3 Bit
    parity += bit;
    SWDIOwriteBit((unsigned char)parity);                 //Parity Bit
    SWDIOwriteBit(0);                      //Stop Bit
    SWDIOwriteBit(1);                      //Park Bit

    // set UIF to input dir
    (*_Jtag.TSTCTRL_PORT) ^= (_Jtag._ENI2O);
    SWDTckCycle();

    // read response and check for ack
    bit = ScanTDO();
    ack  = bit << 0;
    bit = ScanTDO();
    ack |= bit << 1;
    bit = ScanTDO();
    ack |= bit << 2;

    if (ack == DAP_TRANSFER_OK)
    {
        unsigned char count = 0;
        ///Read now all 32 bit data RDATA[0:31]
        val = 0;
        parity = 0;
        for (count = 32; count; count--)
        {
            bit = ScanTDO();
            parity += bit;
            val >>= 1;
            val  |= bit << 31;
        }
        //Read Parity
        bit = ScanTDO();
        if ((parity ^ bit) & 1)
        {
            ack = DAP_TRANSFER_ERROR;
        }
        if (data)
        {
            *data = val;
            ack = DAP_TRANSFER_OK;
        }
    }

     // set UIF to output dir
    SWDTckCycle();
    (*_Jtag.TSTCTRL_PORT) ^= (_Jtag._ENI2O);

    return ack;
}

void hil_Swd_Seq(unsigned short length, unsigned char *sequence)
{
    unsigned char dataptr = 0;
    unsigned short n = 0;
    while (length--)
    {
        if (n == 0)
        {
            dataptr = *sequence++;
            n = 8;
        }
        if (dataptr & 1)
        {
            SWDIOset1TckCycle();
        }
        else
        {
            SWDIOset0TckCycle();
        }
        dataptr >>= 1;
        n--;
    }
    (*_Jtag.Out) |=  _Jtag.TDI;
}