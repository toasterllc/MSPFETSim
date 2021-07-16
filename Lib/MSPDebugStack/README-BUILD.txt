Building MSPDebugStack
======================

1. Windows
----------

1.1 Development environment
---------------------------

* You will need to have Microsoft Visual Studio 2017 installed.
* You will need to have a IAR EW430 5.51 or later installed if you want to
  build the firmware or BSL.


1.2. Dependencies
-----------------

In order to compile msp430.dll with MSVC you will need:

* boost
    -Download and build (http://www.boost.org)
    -Visual Studio users can get pre-built boost binaries from
     http://sourceforge.net/projects/boost/files/boost-binaries/
    -Version used in official build is 1.67
    -Only libboost_system-xxx.lib is required
    -Visual Studio must be configured to find boost headers and libs

* hidapi
    -Download and build (https://github.com/signal11/hidapi/releases)
    -Version used in official build is 0.8.0-rc1
    -hidapi.h must be copied to ThirdParty\include
    -hidapi.lib must be copied to ThirdParty\lib


To compile the firmware and/or BSL with IAR EW430, the following tools are required:

* srecord
    -Download from http://sourceforge.net/projects/srecord/files/srecord-win32
    -Version used in official build is 1.59
    -srec_cat.exe must be copied to Bios\tools

* sed
    -Any Windows port, for example from http://sourceforge.net/projects/unxutils
    -sed.exe must be copied to Bios\tools


1.3.1 Building the firmware (optional)
--------------------------------------

* Build all projects in Bios\bios_core.eww

* Build all projects in Bios\eZ_FET_Bios.eww
    -Make sure eZ_FET configuration is selected for all projects

* Build all projects in Bios\MSP-FET_Bios.eww
    -Make sure MSP_FET configuration is selected for all projects

NOTE: Due to the license of DirectC, the required DirectC source is not part
      of the OS package and the depending project MSP-FET_FpgaUpdate.ewp is not
      built by default.

      To build the MSP-FET_FpgaUpdate project, please download DirectC 2.7 from
      http://www.actel.com/download/program_debug/directc/dc27.aspx
      and copy the sources to MSPDebugStack/Bios/src/fpga_update/DirectC. See
      the README.txt in this directory for details.


1.3.2 Building the BSL (optional)
---------------------------------

* Build all project configurations in ThirdParty\BSL430_Firmware\BSL.eww


1.3.3 Building the DLL
----------------------

* Build BSL430_DLL.sln in ThirdParty\BSL430_DLL

* Build DLL430_v3.sln

NOTE: Due to the license of DirectC, there is no image for the FPGA update
      included in the OS package. To build this image, please refer to 1.3.1.


	  To enable the FPGA update, comment in the line "#define FPGA_UPDATE" in
	  MSPDebugStack/DLL430_v3/src/TI/DLL430/UpdateManagerFet.cpp.


2. Linux
--------

2.1 Development environment
---------------------------

* You will need the GNU tool chain (GCC 4.8.4 or higher) to use the existing Makefile.


2.2. Dependencies
-----------------

In order to compile the libmsp430.so with GCC you will need:

* boost
    -Download from http://www.boost.org
    -Version used in official build is 1.67
    -Only libboost_system.a/so and libboost_filesystem.a/so are required

* hidapi
    -Download and build (https://github.com/signal11/hidapi/releases)
    -Version used in official build is 0.8.0-rc1
    -The MSPDebugStack project assumes hidapi being built against libusb-1.0
     (the default used in the Makefile coming with hidapi 0.8.0-rc1)
    -hidapi.h must be copied to ThirdParty\include
    -hid-libusb.o must be copied to ThirdParty\lib


2.3 Building the shared object
------------------------------

* run "make STATIC=1" in MSPDebugStack
    -If boost is not globally installed, use "make BOOST_DIR=<path to boost>".
     The makefile assumes headers in boost/ and libraries in stage/lib/ of the
     specified directory.

    -Linking dependencies statically (STATIC=1) is advised if the library will
     be copied between machines. A 64bit build is required for use
     with existing IDEs.


3. OSX
--------

2.1 Development environment
---------------------------

You will need Clang/LLVM to use the existing Makefile.

2.2. Dependencies
-----------------

In order to compile libmsp430.dylib with Clang you will need:

* boost
    -Download from http://www.boost.org
    -Version used in official build is 1.67
    -Only libboost_system.a/dylib and libboost_filesystem.a/dylib are required

* hidapi
    -Download and build (https://github.com/signal11/hidapi/releases)
    -Version used in official build is 0.8.0-rc1
    -The MSPDebugStack project assumes hidapi being built against libusb-1.0
     (the default used in the Makefile coming with hidapi 0.8.0-rc1)
    -hidapi.h must be copied to ThirdParty\include
    -libhidapi.a must be copied to ThirdParty\lib64


2.3 Building the shared object
------------------------------

2.3 Building the shared object
------------------------------

* run "make STATIC=1" in MSPDebugStack
    -If boost is not globally installed, use "make BOOST_DIR=<path to boost>".
     The makefile assumes headers in include/boost and libraries in lib/ of the
     specified directory.

    -Linking dependencies statically (STATIC=1) is advised if the library will
     be copied between machines.
