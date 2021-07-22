#pragma once

#define MSP_FET

// Make sure the C basic types aren't used.
// They must be replaced by explicit-width types (eg uint16_t) to ensure
// execution that's consistent with the embedded device.
#define char    BADTYPE
#define short   BADTYPE
#define int     BADTYPE
#define long    BADTYPE

// static variables/functions aren't allowed since the entirety of firmware
// exists within the MSPProbeSim class, and class instances shouldn't share
// variables, while functions should only affect their respective instances.
// Globals should just be member variables, and static functions should just
// be normal functions.
#define static  BADSTATIC

#define extern  BADEXTERN

#include "USB.h"
#include "Descriptor.h"

void dummy_Scheduler(void){};
void dummy_UsbLoop(void){};

int16_t debuggerOff = 0;

struct LOOP_INFOS
{
    void (MSPProbeSim::*Scheduler)(void);
    void (MSPProbeSim::*UsbLoop)(void);
};
typedef struct LOOP_INFOS LOOP_INFOS_t;

LOOP_INFOS_t loopInfos_ =
{
    MEMBER_FN_PTR(V3OP_Scheduler),
    MEMBER_FN_PTR(dummy_UsbLoop),
};

//void init_BiosPorts(void)
//{
//	// Ports all back to reset state
//    P1DIR = 0;  P2DIR = 0;  P3DIR = 0;    // Reset all ports direction to be inputs
//    P4DIR = 0;  P5DIR = 0;  P6DIR = 0;
//    P7DIR = 0;  P8DIR = 0;  P9DIR = 0;
//    P1SEL = 0;  P2SEL = 0;  P3SEL = 0;    // Reset all ports alternate function
//    P4SEL = 0;  P5SEL = 0;  P6SEL = 0;
//    P7SEL = 0;  P8SEL = 0;  P9SEL = 0;
//    P1OUT = 0;  P2OUT = 0;  P3OUT = 0;    // Reset all port output registers
//    P4OUT = 0;  P5OUT = 0;  P6OUT = 0;
//    P7OUT = 0;  P8OUT = 0;  P9OUT = 0;
//
//    // Port2
//    //  P2.0 <- DCDC_PULSE
//    //  P2.1 <- UART_RTS
//    //  P2.2 -> CMD[0]
//    //  P2.3 -> CMD[1]
//    //  P2.4 -> CMD[2]
//    //  P2.5 -> CMD[3]
//    //  P2.6 -> WR_TRIG
//    //  P2.7 -> SYS_CLK
//    //P2SEL = BIT0;
//
//    // Port5
//    //  P5.0 -> VREF+ (output of reference voltage to ADC)
//    //  P5.1 -> FPGA_RESET
//    //  P5.2 -> VF2TEST_CTRL
//    //  P5.3 -> LED1
//    //  P5.4 -> VF2TDI_CTRL
//    //  P5.5 -> TDIOFF_CTRL (0 = turns off TDI, after that ok to select VF for fuse blowing)
//    //  P5.6 <- MCU_DMAE0 / RD_TRIG (DMA trigger input)
//    //  P5.7 -> DCDC_IO1
//    P5DIR |= (BIT3 | BIT4 | BIT5 | BIT7);	// set pins initially to output direction
//    //P5SEL = (BIT0);
//    P5OUT = (BIT5);
//    P5OUT &= ~BIT7;
//
//
//      // Port6
//      //  P6.0 <- A_VBUS5 (input to ADC channel A0)
//      //  P6.1 <- A_VCC_SUPPLY
//      //  P6.2 <- A_VF
//      //  P6.3 <- A_VCC_SENSE0_TRGT
//      //  P6.4 <- A_VCC_DT
//      //  P6.5 <- A_VCC_DT_BSR
//      //  P6.6 -> VCC_JTAGLDO2TRGT_CTRL
//      //  P6.7 -> LED0
//      P6DIR |= (BIT7);	// set pins initially to output direction
//      //P6SEL = (BIT0+BIT1+BIT2+BIT3+BIT4+BIT5);
//
//      // Port7
//      //  P7.0 -> n/c
//      //  P7.1 -> n/c
//      //  P7.2 <- XT2IN
//      //  P7.3 -> XT2OUT
//      //  P7.4 -> DCDC_TEST
//      //  P7.5 -> DCDC_RST (0 = reset)
//      //  P7.6 -> VCC_DT_REF
//      //  P7.7 -> VCC_DCDC_REF
//      P7DIR |= (BIT0+BIT1);	        // set pins initially to output direction
//      P7SEL |= (BIT6+BIT7);	        // set pins to alternate port function BIT2+BIT3
//
//      // Port8
//      //  P8.0 -> VCC_DT2TRGT_CTRL (control signal to switches to provide debug signals to target via JTAG.x)
//      //  P8.1 -> IO_CTRL
//      //  P8.2 -> UART_TXD
//      P8DIR |= BIT3; // set UART_TXD to output direction\r
//      //  P8.3 <- UART_RXD
//    //  P8.4 <- DCDC_IO0
//      //  P8.5 -> HOST_SDA
//      //  P8.6 -> HOST_SCL
//      //  P8.7 -> VCC_SUPPLY2TRGT_CTRL (DCDC VCC to target VCC)
//      P8DIR |= (BIT0+BIT7);	        // set pins initially to output direction
//}
//
//void init_Clock(void)
//{
//     XT2_Stop(); // stop X2 for default state
//
//     SetVCore(3);
//     _DINT_FET();
//
//    //Initialization of clock module
//    #if defined (__MSP430F552x) || defined (__MSP430F550x)
//        P5SEL |= 0x0C;                                      //enable XT2 pins for F5529
//    #elif defined (__MSP430F563x_F663x)
//        P7SEL |= 0x0C;
//    #endif
//
//     //use REFO for FLL and ACLK
//    UCSCTL3 = (UCSCTL3 & ~(SELREF_7)) | (SELREF__REFOCLK);
//    UCSCTL4 = (UCSCTL4 & ~(SELA_7)) | (SELA__REFOCLK);
//
//    //MCLK will be driven by the FLL (not by XT2), referenced to the REFO
//    Init_FLL_Settle(USB_MCLK_FREQ / 1000, USB_MCLK_FREQ / 32768);   //Start the FLL, at the freq indicated by the config
//                                                                    //constant USB_MCLK_FREQ
//    XT1_Stop();                 // Stop xt1 to prevent any error
//
//    if( UCS_STATUS_ERROR == XT2_Start_Timeout(XT2DRIVE_0, 0x4E20))
//    {
//        BIOS_LedOff(0);
//        BIOS_LedOn(1);
//        while(1);
//    }
//}

void Main(void)
{
//    WDTCTL = WDTPW + WDTHOLD;       // Stop watchdog timer
//
//    if(!V3OP_SystemOk())
//    {
//        WDTCTL = 0;
//    }

//    init_BiosPorts();
//    init_Clock();

    BIOS_LedOff(BIOS_LED_MODE);
    BIOS_LedOn(BIOS_LED_POWER);

    // Initialize USB and enable various events
    USB_init();
//    WORD USBEVIE = kUSB_VbusOnEvent+kUSB_VbusOffEvent+kUSB_dataReceivedEvent+kUSB_UsbSuspendEvent+kUSB_UsbResumeEvent+kUSB_UsbResetEvent;
//    USB_setEnabledEvents(USBEVIE);
//    // See if we're already attached physically to USB, and if so, connect to it
//    // Normally applications don't invoke the event handlers, but this is an exception.
//    if (USB_connectionInfo() & kUSB_vbusPresent)
//    {
//        if (USB_enable() == kUSB_succeed)
//        {
//            USB_reset();
//            USB_connect();
//        }
//        else
//        {
//            BIOS_LedOff(BIOS_LED_POWER);
//            BIOS_LedOn(BIOS_LED_MODE);
//            while(1);
//        }
//    }
//    else
//    {
//        BIOS_LedOff(BIOS_LED_POWER);
//        BIOS_LedOn(BIOS_LED_MODE);
//        while(1);
//    }

    // init Bios
    BIOS_InitSystem();
    BIOS_InitCom();

    V3OP_ComInterfaceClear();
    V3OP_DcdcInterfaceClear();

    // int Sw Layers
    V3OP_DcdcInterfaceInit();
    V3OP_HalInterfaceInit();
    V3OP_ComInterfaceInit();

    IccMonitor_StartVoltageSupervision();
    _EINT_FET();

    IccMonitor_TriggerAdc();

    while(1)
    {
        CALL_MEMBER_FN_PTR(loopInfos_.UsbLoop)();
        CALL_MEMBER_FN_PTR(loopInfos_.Scheduler)();
    }
}

#undef extern
#undef static
#undef char
#undef short
#undef int
#undef long
