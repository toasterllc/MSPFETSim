#include "hw_compiler_specific.h"
#include "HalGlobalVars.h"
#include "arch.h"
#include "edt.h"
#include "archFpga.h"
#include <stdio.h>

static struct jtag _Jtag = {0};

#define SWD_DELAY   { _NOP();}

// DAP Transfer Response
#define DAP_TRANSFER_OK                 (1<<0)
#define DAP_TRANSFER_WAIT               (1<<1)
#define DAP_TRANSFER_FAULT              (1<<2)
#define DAP_TRANSFER_ERROR              (1<<3)
#define DAP_TRANSFER_MISMATCH           (1<<4)


void SWDTCKset1()
{ (*_Jtag.Out) |=  _Jtag.TCK;}

#pragma inline=forced
void SWDTCKset0()
{ (*_Jtag.Out) &= ~_Jtag.TCK;}

#pragma inline=forced
void SWDIOset1TckCycle()
{
    (*_Jtag.Out) |=  _Jtag.TMS;
    SWDTCKset0();
    SWD_DELAY;
    SWDTCKset1();
}

#pragma inline=forced
void SWDIOset0TckCycle()
{
    (*_Jtag.Out) &=  ~_Jtag.TMS;
    SWDTCKset0();
    SWD_DELAY;
    SWDTCKset1();
}

#pragma inline=forced
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
        (*_Jtag.Out) |=  _Jtag.TMS;
    }
    else
    {
        (*_Jtag.Out) &= ~_Jtag.TMS;
    }
    SWDTCKset0();
    SWD_DELAY;;
    SWDTCKset1();
}

// turn around direction before
#pragma inline=forced
unsigned char ScanTDO()
{
    unsigned char val = 0;
    SWDTCKset0();
    SWD_DELAY;
    SWD_DELAY;

    if(*_Jtag.In  & _Jtag.TMS)
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

unsigned char Swd_TransferData(unsigned char regiser, unsigned long* data, unsigned char rnw)
{
    volatile unsigned char ack = DAP_TRANSFER_ERROR;
    unsigned long bit = 0;
    unsigned long val = 0;
    unsigned long parity = 0;
    unsigned char count = 0;

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

    // set MSP-FET to input dir
    (*_Jtag.DIRECTION) &= ~(_Jtag.TMS);
    FPGA_BYPASS_DIR_CTRL_PORT_OUT &= ~_Jtag.TMS;
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
        if(rnw)
        {
            ///Read 32 bit data
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
            // sest MSP-FET TMS pin to output
            (*_Jtag.DIRECTION) |= (_Jtag.TMS);
            FPGA_BYPASS_DIR_CTRL_PORT_OUT |= _Jtag.TMS;
            SWDTckCycle();

            return ack;
        }
        else // write
        {
            // sest MSP-FET TMS pin to output
            (*_Jtag.DIRECTION) |= (_Jtag.TMS);
            FPGA_BYPASS_DIR_CTRL_PORT_OUT |= _Jtag.TMS;
            SWDTckCycle();

            val = *data;
            parity = 0;
            for (count = 32; count; count--)
            {
                SWDIOwriteBit((unsigned char)val);
                parity += val;
                val >>= 1;
            }
            //write parity to DAP
            SWDIOwriteBit(parity);
            //ToDo do we need idle cycles here?
            (*_Jtag.Out) &=  ~_Jtag.TMS;
            SWDTckCycle();
            SWDTckCycle();
            SWDTckCycle();
            SWDTckCycle();
            SWDTckCycle();
            SWDTckCycle();
            SWDTckCycle();
            // set SWDIO pin to 1
            (*_Jtag.Out) |=  _Jtag.TMS;
            return ack;
        }
    }
    if ((ack == DAP_TRANSFER_WAIT) || (ack == DAP_TRANSFER_FAULT))
    {
        //WAIT or FAULT response
        if (rnw)
        {
            for (count = 33; count; count--)
            {
                SWDTckCycle();               /* Dummy Read RDATA[0:31] + Parity */
            }
        }

        // sest MSP-FET TMS pin to output
        (*_Jtag.DIRECTION) |= (_Jtag.TMS);
        FPGA_BYPASS_DIR_CTRL_PORT_OUT |= _Jtag.TMS;
        SWDTckCycle();
        if (!rnw)
        {
            // set SWDIO pin to 0
            (*_Jtag.Out) &=  ~_Jtag.TMS;
            for (count = 33; count; count--)
            {
                SWDTckCycle();               /* Dummy Write WDATA[0:31] + Parity */
            }
        }
        // set SWDIO pin to 1
        (*_Jtag.Out) |=  _Jtag.TMS;
        return (ack);
    }

    // sest MSP-FET TMS pin to output
    (*_Jtag.DIRECTION) |= (_Jtag.TMS);
    FPGA_BYPASS_DIR_CTRL_PORT_OUT |= _Jtag.TMS;
    SWDTckCycle();

    //Protocol error
    for (count = 34; count; count--)
    {
        SWDTckCycle();                   /* Back off data phase */
    }
    // set SWDIO pin to 1
    (*_Jtag.Out) |=  _Jtag.TMS;
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
    (*_Jtag.Out) |=  _Jtag.TMS;
}