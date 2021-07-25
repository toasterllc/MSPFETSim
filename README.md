## Overview

MSPFETSim simulates a [MSP-FET](https://www.ti.com/tool/MSP-FET) debug probe to allow generic hardware (like FTDI or STM32 chips) to be used to debug MSP430 targets. MSPFETSim runs a modified version of the real MSP-FET firmware published by TI as a part of the [MSP Debug Stack](https://www.ti.com/tool/MSPDS) package. By running the real firmware, MSPFETSim should (in theory) support any MSP430 chip that the real MSP-FET hardware supports.

MSPFETSim instantiates a simulated USB device which appears to the host system as a real MSP-FET. This strategy means that MSPFETSim should work with existing tools that are compatible with the MSP-FET, such as [TI's Code Composer Studio](https://www.ti.com/tool/CCSTUDIO) or [mspdebug](https://github.com/dlbeer/mspdebug).

MSPFETSim only supports Spy-bi-wire (2 wire) debugging of target MSP430 devices. JTAG (4 wire) support is technically feasible but unimplemented.

MSPFETSim currently supports Linux.


## Motivation

This project was motivated by the need to flash/debug a MSP430 in a custom device via the device's normal USB port. A STM32 handles USB communication on this device, so the STM32 also needed to handle the MSP430 debugging.

With MSPFETSim, enabling flashing/debugging of the MSP430 on this device was accomplished by simply: (1) creating a new MSPFETSim driver to send GPIO toggling commands to the STM32, and (2) implementing GPIO-toggling commands on the STM32.


## Supported Debug Probe Hardware

MSPFETSim has these hardware drivers:

- FTDI driver
    - Supports MPSSE FTDI chips, such as:
        - FT232H
        - FT2232H
        - FT4232H
    
    - Supports the [FTDI C232HM](https://ftdichip.com/products/c232hm-ddhsl-0-2/) cable
        - TCK (orange) <-> MSP TEST
        - TDO (green) <-> MSP RST


## Writing New Drivers

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


## Supported MSP430 Devices

In theory MSPFETSim should support any MSP430 supported by the real MSP-FET hardware, but in practice there are surely bugs and unimplemented functionality.

Flashing and general debugging (with both TI CCS and mspdebug) has been verified working with:

- MSP430G2452
- MSP430G2553
- MSP430FR2433
- MSP430FR2422
- MSP430I2041


## Usage

### Install Depedencies
    sudo apt install libudev-dev
    sudo apt install libusb-1.0-0-dev
    sudo apt install libftdi1-dev

### Clone Repository
    git clone --recurse-submodules git@github.com:heytoaster/MSPFETSim.git

### Disable ModemManager

Because the MSP-FET appears to the host as a modem, the ModemManager daemon may attempt to probe the device. This behavior needs to be disabled for correct operation:

    echo 'ATTRS{idVendor}=="2047", ATTRS{idProduct}=="0014", ENV{ID_MM_DEVICE_IGNORE}="1"' | sudo tee /etc/udev/rules.d/42-mspfetsim.rules > /dev/null
    sudo udevadm control -R

TI Code Composer Studio will also do this, so it may not be necessary if you've installed TI CCS.

### Build

    cd MSPFETSim
    make

### Run

Load the virtual host controller kernel module (necessary to create virtual USB devices):

    sudo modprobe vhci-hcd

Make sure your FTDI-based debug probe is plugged in, and finally run MSPFETSim:

    sudo ./MSPFETSim

At this point `lsusb` should list a MSP-FET device (`Texas Instruments MSP Tools Driver`), and your MSP430 debug tools should see a MSP-FET attached to the system.


## Tips

- If you're using MSPFETSim from a virtual machine like VirtualBox, USB performance can be improved by configuring the VM to use a USB 3.0 (xHCI) controller.


## Caveats

- MSPFETSim is highly dependent on the version of the MSP Debug Stack/`libmsp430.so` that's being used to talk to it.
    - MSPFETSim is currently known to be compatible with:
        - MSP Debug Stack/`libmsp430.so` version 3.15.1.001 (as noted in MSPDS's `revisions.txt`)
        - Code Composer Studio version 10.4.0.00006 (as noted in Help > About Code Composer Studio)
    
    - If your debug software (such as TI CCS or mspdebug) says that it needs to update the MSP-FET's firmware, it's probably using an incompatible version of `libmsp430.so`. Try using the versions specified above.

- Programming flash-based (ie non-FRAM devices) devices correctly requires strobing the `TEST` signal within a certain frequency range. Although the MSPFETSim FTDI driver sets its clock frequency to be in this range, and flashing with MSPFETSim has been tested on the listed hardware (see Supported MSP430 Devices section), adherence to this requirement hasn't been thoroughly investigated.


## Development Notes

MSPFETSim reuses much of the real MSP-FET firmware to avoid the need to re-implement a MSP430 debugger. As TI adds support for new MSP430 devices, MSPFETSim will need to be updated to reflect the changes made to the MSP-FET firmware. To minimize this maintenance burden, as much of the MSP-FET firmware has been copy-pasted as possible, so that the changes made to the MSP-FET firmware can be easily ported to MSPFETSim. The repository also maintains a clean separation between MSP-FET firmware and the rest of MSPFETSim by storing the firmware in Src/Firmware.
