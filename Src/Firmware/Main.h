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
