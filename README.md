# Overview

MSPFETSim simulates a [MSP-FET](https://www.ti.com/tool/MSP-FET) debug probe to allow generic hardware (like FTDI or STM32 chips) to be used to debug MSP430 targets. MSPFETSim runs a modified version of the real MSP-FET firmware published by TI as a part of the [MSP Debug Stack](https://www.ti.com/tool/MSPDS) package. By running the real firmware, MSPFETSim should (in theory) support any MSP430 chip that the real MSP-FET hardware supports.

MSPFETSim instantiates a simulated USB device, which appears to the host system as a real MSP-FET. This strategy means that MSPFETSim should work with existing tools that are compatible with the MSP-FET, such as [TI's Code Composer Studio](https://www.ti.com/tool/CCSTUDIO) or [mspdebug](https://github.com/dlbeer/mspdebug).

MSPFETSim only supports Spy-bi-wire (2 wire) debugging of target MSP430 devices. JTAG (4 wire) support is technically feasible but unimplemented.

MSPFETSim currently supports Linux.


# Motivation

This project was motivated by the need to flash/debug a MSP430 in a custom device via the device's normal USB port. A STM32 handles USB communication on this device, so the STM32 also needed to handle the MSP430 debugging.

With MSPFETSim, enabling flashing/debugging of the MSP430 on this device was accomplished by simply: (1) creating a new MSPFETSim driver to send GPIO toggling commands to the STM32, and (2) implementing GPIO-toggling commands on the STM2.


# Supported Debug Probe Hardware

MSPFETSim has these hardware drivers:

- FTDI driver
    - Supports MPSSE FTDI chips, such as:
        - FT232H
        - FT2232H
        - FT4232H
    
    - Supports the [FTDI C232HM](https://ftdichip.com/products/c232hm-ddhsl-0-2/) cable
        - TCK (orange) <-> MSP TEST
        - TDO (green) <-> MSP RST


# Writing New Drivers

Adding support for new debug probe hardware requires implementing a minimal driver interface. This driver interface, declared in `MSPDebugDriver.h`, consists of 5 functions with semantics that amount to toggling the MSP430's `TEST` and `RST` pins:

- `void sbwTestSet(bool val);`
- `void sbwRstSet(bool val);`
    - Set the output value of a pin

- `void sbwTestPulse();`
    - Pulse TEST=[0,1]

- `void sbwIO(bool tms, bool tclk, bool tdi, bool tdoRead);`
    - Perform a Spy-bi-wire IO cycle

- `void sbwRead(void* buf, size_t len);`
    - Retrieve data previously stored via `sbwIO()`


# Supported MSP430 Devices

In theory MSPFETSim should support any MSP430 supported by the real MSP-FET hardware, but in practice there are surely bugs and unimplemented functionality.

Flashing and general debugging (with both TI CCS and mspdebug) has been verified working with:

- MSP430G2452
- MSP430G2553
- MSP430FR2433
- MSP430FR2422
- MSP430I2041


# Installation / Usage


# Caveats

- MSPFETSim is highly dependent on the version of the MSP Debug Stack/`libmsp430.so` that's being used to talk to it.
    - MSPFETSim is currently known to be compatible with:
        - MSP Debug Stack/`libmsp430.so` version 3.15.1.001 (as noted in `revisions.txt`)
        - Code Composer Studio version 10.4.0.00006 (as noted in Help > About Code Composer Studio)
    
    - If your debug software (such as TI CCS or mspdebug) says that it needs to update the version of firmware used by the MSP-FET, it's likely because it's using an incompatible version of `libmsp430.so`. Try using the versions specified above.

- Programming flash-based (ie non-FRAM devices) devices correctly requires strobing the `TEST` signal within a certain frequency range. Although the FTDI driver sets its clock frequency to be in this range, and flashing with MSPFETSim appears to work correctly on the tested hardware (see Supported MSP430 Devices section), adherence to this requirement hasn't been thoroughly investigated yet.


# Development Philosophy
- Make copy/paste from tilib easy
- Not trying to clean up tilib
    - Large and messy codebase

