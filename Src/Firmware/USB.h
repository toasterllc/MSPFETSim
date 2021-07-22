#pragma once
#include "Types.h"
#include "BIOS.h"
#include "V3OP.h"

using CHAR      = int8_t;
using UCHAR     = uint8_t;
using INT       = int16_t;
using UINT      = uint16_t;
using SHORT     = int16_t;
using USHORT    = uint16_t;
using LONG      = int32_t;
using ULONG     = uint32_t;
using VOID      = void;
using HANDLE    = uint32_t;
using PSTR      = int8_t*;
using BOOL      = int16_t;
using DOUBLE    = double;
using BYTE      = uint8_t;
using PBYTE     = uint8_t*;
using WORD      = uint16_t;
using DWORD     = uint32_t;
using PDWORD    = uint32_t*;

/*----------------------------------------------------------------------------+
 | USB Constants, Type Definition & Macro                                      |
 +----------------------------------------------------------------------------*/

//USB related Constant
#define MAX_ENDPOINT_NUMBER     0x07    //A maximum of 7 endpoints is available
#define EP0_MAX_PACKET_SIZE     0x08
#define EP0_PACKET_SIZE         0x08
#define EP_MAX_PACKET_SIZE      0x40

//Base addresses of transmit and receive buffers
#define OEP1_X_BUFFER_ADDRESS   0x1C00  //Input  Endpoint 1 X Buffer Base-address
#define OEP1_Y_BUFFER_ADDRESS   0x1C40  //Input  Endpoint 1 Y Buffer Base-address
#define IEP1_X_BUFFER_ADDRESS   0x1C80  //Output Endpoint 1 X Buffer Base-address
#define IEP1_Y_BUFFER_ADDRESS   0x1CC0  //Output Endpoint 1 Y Buffer Base-address

#define OEP2_X_BUFFER_ADDRESS   0x1D00  //Input  Endpoint 2 X Buffer Base-address
#define OEP2_Y_BUFFER_ADDRESS   0x1D40  //Input  Endpoint 2 Y Buffer Base-address
#define IEP2_X_BUFFER_ADDRESS   0x1D80  //Output Endpoint 2 X Buffer Base-address
#define IEP2_Y_BUFFER_ADDRESS   0x1DC0  //Output Endpoint 2 Y Buffer Base-address

#define OEP3_X_BUFFER_ADDRESS   0x1E00  //Input  Endpoint 2 X Buffer Base-address
#define OEP3_Y_BUFFER_ADDRESS   0x1E40  //Input  Endpoint 2 Y Buffer Base-address
#define IEP3_X_BUFFER_ADDRESS   0x1E80  //Output Endpoint 2 X Buffer Base-address
#define IEP3_Y_BUFFER_ADDRESS   0x1EC0  //Output Endpoint 2 Y Buffer Base-address

#define OEP4_X_BUFFER_ADDRESS   0x1F00  //Input  Endpoint 2 X Buffer Base-address
#define OEP4_Y_BUFFER_ADDRESS   0x1F40  //Input  Endpoint 2 Y Buffer Base-address
#define IEP4_X_BUFFER_ADDRESS   0x1F80  //Output Endpoint 2 X Buffer Base-address
#define IEP4_Y_BUFFER_ADDRESS   0x1FC0  //Output Endpoint 2 Y Buffer Base-address

#define OEP5_X_BUFFER_ADDRESS   0x2000  //Input  Endpoint 2 X Buffer Base-address
#define OEP5_Y_BUFFER_ADDRESS   0x2040  //Input  Endpoint 2 Y Buffer Base-address
#define IEP5_X_BUFFER_ADDRESS   0x2080  //Output Endpoint 2 X Buffer Base-address
#define IEP5_Y_BUFFER_ADDRESS   0x20C0  //Output Endpoint 2 Y Buffer Base-address

#define OEP6_X_BUFFER_ADDRESS   0x2100  //Input  Endpoint 2 X Buffer Base-address
#define OEP6_Y_BUFFER_ADDRESS   0x2140  //Input  Endpoint 2 Y Buffer Base-address
#define IEP6_X_BUFFER_ADDRESS   0x2180  //Output Endpoint 2 X Buffer Base-address
#define IEP6_Y_BUFFER_ADDRESS   0x21C0  //Output Endpoint 2 Y Buffer Base-address

#define OEP7_X_BUFFER_ADDRESS   0x2200  //Input  Endpoint 2 X Buffer Base-address
#define OEP7_Y_BUFFER_ADDRESS   0x2240  //Input  Endpoint 2 Y Buffer Base-address
#define IEP7_X_BUFFER_ADDRESS   0x2280  //Output Endpoint 2 X Buffer Base-address
#define IEP7_Y_BUFFER_ADDRESS   0x22C0  //Output Endpoint 2 Y Buffer Base-address

#define X_BUFFER 0
#define Y_BUFFER 1

//Macros for end point numbers
#define EP1 1
#define EP2 2
#define EP3 3
#define EP4 4
#define EP5 5
#define EP6 6
#define EP7 7

//addresses of pipes for endpoints
#define EP1_OUT_ADDR          0x01      //address for endpoint 1
#define EP2_OUT_ADDR          0x02      //address for endpoint 2
#define EP3_OUT_ADDR          0x03      //address for endpoint 3
#define EP4_OUT_ADDR          0x04      //address for endpoint 4
#define EP5_OUT_ADDR          0x05      //address for endpoint 5
#define EP6_OUT_ADDR          0x06      //address for endpoint 6
#define EP7_OUT_ADDR          0x07      //address for endpoint 7

//Input end points
#define EP1_IN_ADDR          0x81       //address for endpoint 1
#define EP2_IN_ADDR          0x82       //address for endpoint 2
#define EP3_IN_ADDR          0x83       //address for endpoint 3
#define EP4_IN_ADDR          0x84       //address for endpoint 4
#define EP5_IN_ADDR          0x85       //address for endpoint 5
#define EP6_IN_ADDR          0x86       //address for endpoint 6
#define EP7_IN_ADDR          0x87       //address for endpoint 7


//EDB Data Structure
typedef struct _tEDB {
    BYTE bEPCNF;                        //Endpoint Configuration
    BYTE bEPBBAX;                       //Endpoint X Buffer Base Address
    BYTE bEPBCTX;                       //Endpoint X Buffer byte Count
    BYTE bSPARE0;                       //no used
    BYTE bSPARE1;                       //no used
    BYTE bEPBBAY;                       //Endpoint Y Buffer Base Address
    BYTE bEPBCTY;                       //Endpoint Y Buffer byte Count
    BYTE bEPSIZXY;                      //Endpoint XY Buffer Size
} tEDB, *tpEDB;

typedef struct _tEDB0 {
    BYTE bIEPCNFG;                      //Input Endpoint 0 Configuration Register
    BYTE bIEPBCNT;                      //Input Endpoint 0 Buffer Byte Count
    BYTE bOEPCNFG;                      //Output Endpoint 0 Configuration Register
    BYTE bOEPBCNT;                      //Output Endpoint 0 Buffer Byte Count
} tEDB0, *tpEDB0;

//EndPoint Desciptor Block Bits
#define EPCNF_USBIE     0x04            //USB Interrupt on Transaction Completion. Set By MCU
                                        //0:No Interrupt, 1:Interrupt on completion
#define EPCNF_STALL     0x08            //USB Stall Condition Indication. Set by UBM
                                        //0: No Stall, 1:USB Install Condition
#define EPCNF_DBUF      0x10            //Double Buffer Enable. Set by MCU
                                        //0: Primary Buffer Only(x-buffer only), 1:Toggle Bit Selects Buffer

#define EPCNF_TOGGLE     0x20           //USB Toggle bit. This bit reflects the toggle sequence bit of DATA0 and DATA1.

#define EPCNF_UBME      0x80            //UBM Enable or Disable bit. Set or Clear by MCU.
                                        //0:UBM can't use this endpoint
                                        //1:UBM can use this endpoint
#define EPBCNT_BYTECNT_MASK 0x7F        //MASK for Buffer Byte Count
#define EPBCNT_NAK       0x80           //NAK, 0:No Valid in buffer, 1:Valid packet in buffer

//definitions for MSP430 USB-module
#define START_OF_USB_BUFFER   0x1C00

//input and output buffers for EP0
#define USBIEP0BUF 0x2378
#define USBOEP0BUF 0x2370









/*----------------------------------------------------------------------------+
 | Constant Definition                                                         |
 +----------------------------------------------------------------------------*/
#define USB_RETURN_DATA_LENGTH  8
#define SIZEOF_DEVICE_REQUEST   0x08

//Bit definitions for DEVICE_REQUEST.bmRequestType
//Bit 7:   Data direction
#define USB_REQ_TYPE_OUTPUT     0x00    //0 = Host sending data to device
#define USB_REQ_TYPE_INPUT      0x80    //1 = Device sending data to host

//Bit 6-5: Type
#define USB_REQ_TYPE_MASK       0x60    //Mask value for bits 6-5
#define USB_REQ_TYPE_STANDARD   0x00    //00 = Standard USB request
#define USB_REQ_TYPE_CLASS      0x20    //01 = Class specific
#define USB_REQ_TYPE_VENDOR     0x40    //10 = Vendor specific

//Bit 4-0: Recipient
#define USB_REQ_TYPE_RECIP_MASK 0x1F    //Mask value for bits 4-0
#define USB_REQ_TYPE_DEVICE     0x00    //00000 = Device
#define USB_REQ_TYPE_INTERFACE  0x01    //00001 = Interface
#define USB_REQ_TYPE_ENDPOINT   0x02    //00010 = Endpoint
#define USB_REQ_TYPE_OTHER      0x03    //00011 = Other

//Values for DEVICE_REQUEST.bRequest
//Standard Device Requests
#define USB_REQ_GET_STATUS              0
#define USB_REQ_CLEAR_FEATURE           1
#define USB_REQ_SET_FEATURE             3
#define USB_REQ_SET_ADDRESS             5
#define USB_REQ_GET_DESCRIPTOR          6
#define USB_REQ_SET_DESCRIPTOR          7
#define USB_REQ_GET_CONFIGURATION       8
#define USB_REQ_SET_CONFIGURATION       9
#define USB_REQ_GET_INTERFACE           10
#define USB_REQ_SET_INTERFACE           11
#define USB_REQ_SYNCH_FRAME             12

//CDC CLASS Requests
#define USB_CDC_GET_LINE_CODING         0x21
#define USB_CDC_SET_LINE_CODING         0x20
#define USB_CDC_SET_CONTROL_LINE_STATE  0x22

//HID CLASS Requests
#define USB_HID_REQ                     0x81
#define USB_REQ_GET_REPORT              0x01
#define USB_REQ_GET_IDLE                0x02
#define USB_REQ_SET_REPORT              0x09
#define USB_REQ_SET_IDLE                0x0A
#define USB_REQ_SET_PROTOCOL            0x0B
#define USB_REQ_GET_PROTOCOL            0x03

//MSC CLASS Requests
#define USB_MSC_RESET_BULK              0xFF
#define USB_MSC_GET_MAX_LUN             0xFE

// PHDC CLASS Requests
#define USB_PHDC_GET_STATUS             0x00

//HID Values for HID Report Types (tSetup.bValueH)
#define USB_REQ_HID_INPUT               0x01
#define USB_REQ_HID_OUTPUT              0x02
#define USB_REQ_HID_FEATURE             0x03

#define USB_REQ_HID_BOOT_PROTOCOL       0x00
#define USB_REQ_HID_REPORT_PROTOCOL     0x01


//Descriptor Type Values
#define DESC_TYPE_DEVICE                1       //Device Descriptor (Type 1)
#define DESC_TYPE_CONFIG                2       //Configuration Descriptor (Type 2)
#define DESC_TYPE_STRING                3       //String Descriptor (Type 3)
#define DESC_TYPE_INTERFACE             4       //Interface Descriptor (Type 4)
#define DESC_TYPE_ENDPOINT              5       //Endpoint Descriptor (Type 5)
#define DESC_TYPE_DEVICE_QUALIFIER      6       //Endpoint Descriptor (Type 6)
#define DESC_TYPE_IAD                   0x0B
#define DESC_TYPE_HUB                   0x29    //Hub Descriptor (Type 6)
#define DESC_TYPE_HID                   0x21    //HID Descriptor
#define DESC_TYPE_REPORT                0x22    //Report Descriptor
#define DESC_TYPE_PHYSICAL              0x23    //Physical Descriptor

//Feature Selector Values
#define FEATURE_REMOTE_WAKEUP           1       //Remote wakeup (Type 1)
#define FEATURE_ENDPOINT_STALL          0       //Endpoint stall (Type 0)

//Device Status Values
#define DEVICE_STATUS_REMOTE_WAKEUP     0x02
#define DEVICE_STATUS_SELF_POWER        0x01

//Maximum descriptor size
#define MAX_DESC_SIZE                   256

//DEVICE_DESCRIPTOR structure
#define SIZEOF_DEVICE_DESCRIPTOR        0x12
#define OFFSET_DEVICE_DESCRIPTOR_VID_L  0x08
#define OFFSET_DEVICE_DESCRIPTOR_VID_H  0x09
#define OFFSET_DEVICE_DESCRIPTOR_PID_L  0x0A
#define OFFSET_DEVICE_DESCRIPTOR_PID_H  0x0B
#define OFFSET_CONFIG_DESCRIPTOR_POWER  0x07
#define OFFSET_CONFIG_DESCRIPTOR_CURT   0x08

//CONFIG_DESCRIPTOR structure
#define SIZEOF_CONFIG_DESCRIPTOR 0x09

//HID DESCRIPTOR structure
//#define SIZEOF_HID_DESCRIPTOR 0x09

//Bit definitions for CONFIG_DESCRIPTOR.bmAttributes
#define CFG_DESC_ATTR_SELF_POWERED  0x40    //Bit 6: If set, device is self powered
#define CFG_DESC_ATTR_BUS_POWERED   0x80    //Bit 7: If set, device is bus powered
#define CFG_DESC_ATTR_REMOTE_WAKE   0x20    //Bit 5: If set, device supports remote wakeup

//INTERFACE_DESCRIPTOR structure
#define SIZEOF_INTERFACE_DESCRIPTOR 0x09

//ENDPOINT_DESCRIPTOR structure
#define SIZEOF_ENDPOINT_DESCRIPTOR 0x07

//Bit definitions for EndpointDescriptor.EndpointAddr
#define EP_DESC_ADDR_EP_NUM     0x0F        //Bit 3-0: Endpoint number
#define EP_DESC_ADDR_DIR_IN     0x80        //Bit 7: Direction of endpoint, 1/0 = In/Out

//Bit definitions for EndpointDescriptor.EndpointFlags
#define EP_DESC_ATTR_TYPE_MASK  0x03        //Mask value for bits 1-0
#define EP_DESC_ATTR_TYPE_CONT  0x00        //Bit 1-0: 00 = Endpoint does control transfers
#define EP_DESC_ATTR_TYPE_ISOC  0x01        //Bit 1-0: 01 = Endpoint does isochronous transfers
#define EP_DESC_ATTR_TYPE_BULK  0x02        //Bit 1-0: 10 = Endpoint does bulk transfers
#define EP_DESC_ATTR_TYPE_INT   0x03        //Bit 1-0: 11 = Endpoint does interrupt transfers

//Definition to indicate valid/invalid data
#define DATA_VALID      1
#define DATA_INVALID    0

//Function return values
#define kUSB_succeed        0x00
#define kUSB_generalError   0x01
#define kUSB_notEnabled     0x02
//#define kUSB_VbusNotPresent 0x03

//return values USB_connectionInfo(), USB_connect()
#define kUSB_vbusPresent    0x01
#define kUSB_busActive      0x02    //frame sync packets are being received
#define kUSB_ConnectNoVBUS  0x04
#define kUSB_suspended      0x08
#define kUSB_NotSuspended   0x10
#define kUSB_Enumerated     0x20
#define kUSB_purHigh        0x40

//Parameters for function USB_setEnabledEvents()
#define kUSB_clockFaultEvent        0x0001
#define kUSB_VbusOnEvent            0x0002
#define kUSB_VbusOffEvent           0x0004
#define kUSB_UsbResetEvent          0x0008
#define kUSB_UsbSuspendEvent        0x0010
#define kUSB_UsbResumeEvent         0x0020
#define kUSB_dataReceivedEvent      0x0040
#define kUSB_sendCompletedEvent     0x0080
#define kUSB_receiveCompletedEvent  0x0100
#define kUSB_allUsbEvents           0x01FF

//USB connection states
#define ST_USB_DISCONNECTED         0x80
#define ST_USB_CONNECTED_NO_ENUM    0x81
#define ST_ENUM_IN_PROGRESS         0x82
#define ST_ENUM_ACTIVE              0x83
#define ST_ENUM_SUSPENDED           0x84
//#define ST_FAILED_ENUM              0x85
#define ST_ERROR                    0x86
#define ST_NOENUM_SUSPENDED         0x87

#define ENUMERATION_COMPLETE 0x01








//***********************************************************************************************
// CDC or HID - Define both for composite support
//***********************************************************************************************
#define _CDC_          // Needed for CDC inteface
//***********************************************************************************************
// CONFIGURATION CONSTANTS
//***********************************************************************************************
// These constants configure the API stack and help define the USB descriptors.
// Refer to Sec. 6 of the MSP430 USB CDC API Programmer's Guide for descriptions of these constants.

// Configuration Constants that can change
// #define that relates to Device Descriptor
#define USB_VID               0x2047        // Vendor ID (VID)

#ifdef eZ_FET
  #define USB_PID               0x0013        // Product ID for eZ-FET
#endif
#ifdef MSP_FET
  #define USB_PID               0x0014        // Product ID for MSP-FET
#endif
/*----------------------------------------------------------------------------+
| Firmware Version                                                            |
| How to detect version number of the FW running on MSP430?                   |
| on Windows Open ControlPanel->Systems->Hardware->DeviceManager->Ports->     |
|         Msp430->ApplicationUART->Details                                    |
+----------------------------------------------------------------------------*/
#define VER_FW_H              0x02          // Device release number, in binary-coded decimal
#define VER_FW_L              0x00          // Device release number, in binary-coded decimal
// If a serial number is to be reported, set this to the index within the string descriptor
//of the dummy serial number string.  It will then be automatically handled by the API.
// If no serial number is to be reported, set this to 0.
#define USB_STR_INDEX_SERNUM  3
 #define PHDC_ENDPOINTS_NUMBER               2  // bulk in, bulk out


#define DESCRIPTOR_TOTAL_LENGTH             141    // wTotalLength, This is the sum of configuration descriptor length  + CDC descriptor length  + HID descriptor length
#define USB_NUM_INTERFACES                  4    // Number of implemented interfaces.

#define CDC0_COMM_INTERFACE                0              // Comm interface number of CDC0
#define CDC0_DATA_INTERFACE                1              // Data interface number of CDC0
#define CDC0_INTEP_ADDR                    0x81           // Interrupt Endpoint Address of CDC0
#define CDC0_OUTEP_ADDR                    0x02           // Output Endpoint Address of CDC0
#define CDC0_INEP_ADDR                     0x82           // Input Endpoint Address of CDC0

#define CDC1_COMM_INTERFACE                2              // Comm interface number of CDC1
#define CDC1_DATA_INTERFACE                3              // Data interface number of CDC1
#define CDC1_INTEP_ADDR                    0x83           // Interrupt Endpoint Address of CDC1
#define CDC1_OUTEP_ADDR                    0x04           // Output Endpoint Address of CDC1
#define CDC1_INEP_ADDR                     0x84           // Input Endpoint Address of CDC1

#define CDC_NUM_INTERFACES                   2           //  Total Number of CDCs implemented. should set to 0 if there are no CDCs implemented.
#define HID_NUM_INTERFACES                   0           //  Total Number of HIDs implemented. should set to 0 if there are no HIDs implemented.
#define MSC_NUM_INTERFACES                   0           //  Total Number of MSCs implemented. should set to 0 if there are no MSCs implemented.
#define PHDC_NUM_INTERFACES                  0           //  Total Number of PHDCs implemented. should set to 0 if there are no PHDCs implemented.
// Interface numbers for the implemented CDSs and HIDs, This is to use in the Application(main.c) and in the interupt file(UsbIsr.c).
#define CDC0_INTFNUM                0
#define CDC1_INTFNUM                1
#define MSC_MAX_LUN_NUMBER                   1           // Maximum number of LUNs supported

#define PUTWORD(x)      ((x)&0xFF),((x)>>8)

#define USB_OUTEP_INT_EN BIT0 | BIT2 | BIT4
#define USB_INEP_INT_EN BIT0 | BIT1 | BIT2 | BIT3 | BIT4
// MCLK frequency of MCU, in Hz
// For running higher frequencies the Vcore voltage adjustment may required.
// Please refer to Data Sheet of the MSP430 device you use
#ifdef eZ_FET
  #define USB_MCLK_FREQ 25000000                // MCLK frequency in Hz(eZ-FET, 5528)
#endif
#ifdef MSP_FET
  #define USB_MCLK_FREQ 22000000                // MCLK frequency in Hz(MSP-FET, 6638)
#endif

#define USB_PLL_XT        2                  // Defines which XT is used by the PLL (1=XT1, 2=XT2)
#define USB_XT_FREQ       USBPLL_SETCLK_4_0  // Indicates the freq of the crystal on the oscillator indicated by USB_PLL_XT
#define USB_DISABLE_XT_SUSPEND 0             // If non-zero, then USB_suspend() will disable the oscillator
                                             // that is designated by USB_PLL_XT; if zero, USB_suspend won't
                                             // affect the oscillator
#define USB_DMA_CHAN             0x00        // Set to 0xFF if no DMA channel will be used 0..7 for selected DMA channel



// Controls whether the remote wakeup feature is supported by this device.
// A value of 0x20 indicates that is it supported (this value is the mask for
// the bmAttributes field in the configuration descriptor).
// A value of zero indicates remote wakeup is not supported.
// Other values are undefined, as they will interfere with bmAttributes.
#define USB_SUPPORT_REM_WAKE 0x00

// Controls whether the application is self-powered to any degree.  Should be
// set to 0x40, unless the USB device is fully supplied by the bus.
#define USB_SUPPORT_SELF_POWERED 0x80

// Controls what the device reports to the host regarding how much power it will
// consume from VBUS.  Expressed in 2mA units; that is, the number of mA
// communicated is twice the value of this field.
#define USB_MAX_POWER 0x32
//Configuration constants that can not change ( Fixed Values)
#define CDC_CLASS  2
#define HID_CLASS  3
#define MSC_CLASS  4
#define PHDC_CLASS 5

    #define MAX_PACKET_SIZE   0x40              // Max size of the USB packets.

//***********************************************************************************************
// DESCRIPTOR CONSTANTS
//***********************************************************************************************
#define SIZEOF_DEVICE_DESCRIPTOR  0x12
//#define SIZEOF_REPORT_DESCRIPTOR  36
//#define USBHID_REPORT_LENGTH      64  // length of whole HID report (including Report ID)
#define CONFIG_STRING_INDEX       4
#define INTF_STRING_INDEX         5
#define USB_CONFIG_VALUE          0x01
//***********************************************************************************************
// OUTWARD DECLARATIONS
//***********************************************************************************************

//Calculates the endpoint descriptor block number from given address
#define EDB(addr) ((addr&0x07)-1)

/* Structure for generic part of configuration descriptor */
struct abromConfigurationDescriptorGenric
{
    BYTE sizeof_config_descriptor;            // bLength
    BYTE desc_type_config;                    // bDescriptorType: 2
    BYTE sizeof_configuration_descriptor1;    // wTotalLength
    BYTE sizeof_configuration_descriptor2;
    BYTE usb_num_configurations;              // bNumInterfaces
    BYTE bconfigurationvalue;                 // bConfigurationValue
    BYTE  config_string_index;                // iConfiguration Description offset
    BYTE mattributes;                         // bmAttributes, bus power, remote wakeup
    BYTE usb_max_power;                       // Max. Power Consumption at 2mA unit
};

/************************************************CDC Descriptor**************************/
struct abromConfigurationDescriptorCdc
{
//Interface Association Descriptor
    BYTE bLength;                             // Size of this Descriptor in Bytes
    BYTE bDescriptorType;                     // Descriptor Type (=11)
    BYTE bFirstInterface;                     // Interface number of the first one associated with this function
    BYTE bInterfaceCount;                     // Numver of contiguous interface associated with this function
    BYTE bFunctionClass;                      // The class triad of this interface,
    BYTE bFunctionSubClass;                   // usually same as the triad of the first interface
    BYTE bFunctionProcotol;
    BYTE iInterface;                          // Index of String Desc for this function
// interface descriptor (9 bytes)
    BYTE blength_intf;                        // blength: interface descriptor size
    BYTE desc_type_interface;                 // bdescriptortype: interface
    BYTE interface_number_cdc;                // binterfacenumber
    BYTE balternatesetting;                   // balternatesetting: alternate setting
    BYTE bnumendpoints;                       // bnumendpoints: three endpoints used
    BYTE binterfaceclass;                     // binterfaceclass: communication interface class
    BYTE binterfacesubclass;                  // binterfacesubclass: abstract control model
    BYTE binterfaceprotocol;                  // binterfaceprotocol: common at commands
    BYTE intf_string_index;                   // interface:
//header functional descriptor
    BYTE blength_header;                      // blength: endpoint descriptor size
    BYTE bdescriptortype_header;              // bdescriptortype: cs_interface
    BYTE bdescriptorsubtype_header;           // bdescriptorsubtype: header func desc
    BYTE bcdcdc1;
    BYTE bcdcdc2;                             // bcdcdc: spec release number

//call managment functional descriptor
    BYTE bfunctionlength;                     // bfunctionlength
    BYTE bdescriptortype_c;                   // bdescriptortype: cs_interface
    BYTE bdescriptorsubtype_c;                // bdescriptorsubtype: call management func desc
    BYTE bmcapabilities;                      // bmcapabilities: d0+d1
    BYTE intf_number_cdc;                     // bdatainterface: 0

//acm functional descriptor
    BYTE bfunctionlength_acm;                 // bfunctionlength
    BYTE bdescriptortype_acm;                 // bdescriptortype: cs_interface
    BYTE bdescriptorsubtype_acm;              // bdescriptorsubtype: abstract control management desc
    BYTE bmcapabilities_acm;                  // bmcapabilities

// Union Functional Descriptor
    BYTE bLength_ufd;                         // Size, in bytes
    BYTE bdescriptortype_ufd;                 // bDescriptorType: CS_INTERFACE
    BYTE bdescriptorsubtype_ufd;              // bDescriptorSubtype: Union Functional Desc
    BYTE bmasterinterface_ufd;                // bMasterInterface -- the controlling intf for the union
    BYTE bslaveinterface_ufd;                 // bSlaveInterface -- the controlled intf for the union

//Interrupt end point related fields
    BYTE sizeof_epintep_descriptor;           // blength: endpoint descriptor size
    BYTE desc_type_epintep;                   // bdescriptortype: endpoint
    BYTE cdc_intep_addr;                      // bendpointaddress: (in2)
    BYTE epintep_desc_attr_type_int;          // bmattributes: interrupt
    BYTE epintep_wmaxpacketsize1;
    BYTE epintep_wmaxpacketsize;              // wmaxpacketsize, 64 bytes
    BYTE epintep_binterval;                   // binterval

// Data interface descriptor (9 bytes)
    BYTE blength_slaveintf;                   // blength: interface descriptor size
    BYTE desc_type_slaveinterface;            // bdescriptortype: interface
    BYTE interface_number_slavecdc;           // binterfacenumber
    BYTE balternatesetting_slave;             // balternatesetting: alternate setting
    BYTE bnumendpoints_slave;                 // bnumendpoints: three endpoints used
    BYTE binterfaceclass_slave;               // binterfaceclass: data interface class
    BYTE binterfacesubclass_slave;            // binterfacesubclass: abstract control model
    BYTE binterfaceprotocol_slave;            // binterfaceprotocol: common at commands
    BYTE intf_string_index_slave;             // interface:

// Bulk out end point related fields
    BYTE sizeof_outep_descriptor;             // blength: endpoint descriptor size
    BYTE desc_type_outep;                     // bdescriptortype: endpoint
    BYTE cdc_outep_addr;                      // bendpointaddress: (out3)
    BYTE outep_desc_attr_type_bulk;           // bmattributes: bulk
    BYTE outep_wmaxpacketsize1;
    BYTE outep_wmaxpacketsize2;               // wmaxpacketsize, 64 bytes
    BYTE outep_binterval;                     // binterval: ignored for bulk transfer

// Bulk in related fields
    BYTE sizeof_inep_descriptor;              // blength: endpoint descriptor size
    BYTE desc_type_inep;                      // bdescriptortype: endpoint
    BYTE cdc_inep_addr;                       // bendpointaddress: (in3)
    BYTE inep_desc_attr_type_bulk;            // bmattributes: bulk
    BYTE inep_wmaxpacketsize1;
    BYTE inep_wmaxpacketsize2;                // wmaxpacketsize, 64 bytes
    BYTE inep_binterval;                      // binterval: ignored for bulk transfer
};

/**************************************HID descriptor structure *************************/
struct abromConfigurationDescriptorHid
{
//INTERFACE DESCRIPTOR (9 bytes)
    BYTE sizeof_interface_descriptor;        // Desc Length
    BYTE desc_type_interface;                // DescriptorType
    BYTE interface_number_hid;               // Interface number
    BYTE balternatesetting;                  // Any alternate settings if supported
    BYTE bnumendpoints;                      // Number of end points required
    BYTE binterfaceclass;                    // Class ID
    BYTE binterfacesubclass;                 // Sub class ID
    BYTE binterfaceprotocol;                 // Protocol
    BYTE intf_string_index;                  // String Index

//hid descriptor (9 bytes)
    BYTE blength_hid_descriptor;             // HID Desc length
    BYTE hid_descriptor_type;                // HID Desc Type
    BYTE hidrevno1;                          // Rev no
    BYTE hidrevno2;                          // Rev no - 2nd part
    BYTE tcountry;                           // Country code
    BYTE numhidclasses;                      // Number of HID classes to follow
    BYTE report_descriptor_type;             // Report desc type
    BYTE tlength;                            // Total length of report descriptor
    BYTE size_rep_desc;

//input end point descriptor (7 bytes)
    BYTE size_inp_endpoint_descriptor;       // End point desc size
    BYTE desc_type_inp_endpoint;             // Desc type
    BYTE hid_inep_addr;                      // Input end point address
    BYTE ep_desc_attr_type_inp_int;          // Type of end point
    BYTE  inp_wmaxpacketsize1;               // Max packet size
    BYTE  inp_wmaxpacketsize2;
    BYTE inp_binterval;                      // bInterval in ms

 // Output end point descriptor; (7 bytes)
    BYTE size_out_endpoint_descriptor;       // Output endpoint desc size
    BYTE desc_type_out_endpoint;             // Desc type
    BYTE hid_outep_addr;                     // Output end point address
    BYTE ep_desc_attr_type_out_int;          // End point type
    BYTE out_wmaxpacketsize1;                // Max packet size
    BYTE out_wmaxpacketsize2;
    BYTE out_binterval;                      // bInterval in ms
};

/**************************************MSC descriptor structure *************************/
struct abromConfigurationDescriptorMsc
{
// INTERFACE DESCRIPTOR (9 bytes)
    BYTE sizeof_interface_descriptor;         // Desc Length
    BYTE desc_type_interface;                 // DescriptorType
    BYTE interface_number_hid;                // Interface number
    BYTE balternatesetting;                   // Any alternate settings if supported
    BYTE bnumendpoints;                       // Number of end points required
    BYTE binterfaceclass;                     // Class ID
    BYTE binterfacesubclass;                  // Sub class ID
    BYTE binterfaceprotocol;                  // Protocol
    BYTE intf_string_index;                   // String Index

// input end point descriptor (7 bytes)
    BYTE size_inp_endpoint_descriptor;        // End point desc size
    BYTE desc_type_inp_endpoint;              // Desc type
    BYTE hid_inep_addr;                       // Input end point address
    BYTE ep_desc_attr_type_inp_int;           // Type of end point
    BYTE  inp_wmaxpacketsize1;                // Max packet size
    BYTE  inp_wmaxpacketsize2;
    BYTE inp_binterval;                       // bInterval in ms

// Output end point descriptor; (7 bytes)
    BYTE size_out_endpoint_descriptor;        // Output endpoint desc size
    BYTE desc_type_out_endpoint;              // Desc type
    BYTE hid_outep_addr;                      // Output end point address
    BYTE ep_desc_attr_type_out_int;           // End point type
    BYTE out_wmaxpacketsize1;                 // Max packet size
    BYTE out_wmaxpacketsize2;
    BYTE out_binterval;                       // bInterval in ms
};

/* Global structure having Generic,CDC,HID, MSC structures */
struct  abromConfigurationDescriptorGroup
{
    /* Generic part of config descriptor */
    const struct abromConfigurationDescriptorGenric abromConfigurationDescriptorGenric;
#ifdef _MSC_
    /* MSC descriptor structure */
    const struct abromConfigurationDescriptorMsc stMsc[MSC_NUM_INTERFACES];
#endif
#ifdef _CDC_
    /* CDC descriptor structure */
    const struct abromConfigurationDescriptorCdc stCdc[CDC_NUM_INTERFACES];
#endif
#ifdef _HID_
    /* HID descriptor structure */
    const struct abromConfigurationDescriptorHid stHid[HID_NUM_INTERFACES];
#endif
#ifdef _PHDC_
/* PDC descriptor structure */
    const struct abromConfigurationDescriptorPhdc stPhdc[PHDC_NUM_INTERFACES];
#endif
};

//extern const struct  abromConfigurationDescriptorGroup abromConfigurationDescriptorGroup;
//extern BYTE const abromDeviceDescriptor[SIZEOF_DEVICE_DESCRIPTOR];
//extern BYTE const abromStringDescriptor[];
//extern BYTE const abromReportDescriptor[SIZEOF_REPORT_DESCRIPTOR];

/* Handle Structure - Will be populated in descriptors.c based on number of CDC,HID interfaces */
struct tUsbHandle
{
    BYTE ep_In_Addr;               // Input EP Addr
    BYTE ep_Out_Addr;              // Output EP Addr
    BYTE edb_Index;                // The EDB index
    BYTE dev_Class;                // Device Class- 2 for CDC, 3 for HID
    WORD intepEP_X_Buffer;         // Interupt X Buffer Addr
    WORD intepEP_Y_Buffer;         // Interupt Y Buffer Addr
    WORD oep_X_Buffer;             // Output X buffer Addr
    WORD oep_Y_Buffer;             // Output Y buffer Addr
    WORD iep_X_Buffer;             // Input X Buffer Addr
    WORD iep_Y_Buffer;             // Input  Y Buffer Addr
};

#define kUSBCDC_waitingForSend      0x01
#define kUSBCDC_waitingForReceive   0x02
#define kUSBCDC_dataWaiting         0x04
#define kUSBCDC_busNotAvailable     0x08
#define kUSB_allCdcEvents           0xFF

#define kUSBCDC_sendStarted         0x01
#define kUSBCDC_sendComplete        0x02
#define kUSBCDC_intfBusyError       0x03
#define kUSBCDC_receiveStarted      0x04
#define kUSBCDC_receiveCompleted    0x05
#define kUSBCDC_receiveInProgress   0x06
#define kUSBCDC_generalError        0x07
#define kUSBCDC_busNotAvailable     0x08

//extern const struct tUsbHandle stUsbHandle[CDC_NUM_INTERFACES + HID_NUM_INTERFACES + MSC_NUM_INTERFACES + PHDC_NUM_INTERFACES];
//extern const tDEVICE_REQUEST_COMPARE tUsbRequestList[];

BYTE USBCDC_intfStatus (BYTE intfNum, WORD* bytesSent, WORD* bytesReceived) {
    return 0;
}

BYTE USBCDC_abortSend (WORD* size, BYTE intfNum) {
    // Unimplemented
    abort();
    return 0;
}

BYTE cdcSendDataInBackground (BYTE* dataBuf, WORD size, BYTE intfNum, ULONG ulTimeout) {
    ULONG sendCounter = 0;
    WORD bytesSent, bytesReceived;

    while (USBCDC_intfStatus(intfNum,&bytesSent, &bytesReceived) & kUSBCDC_waitingForSend)
    {
        if (ulTimeout && ((sendCounter++) > ulTimeout))
        {    /* A send operation is underway; incr counter & try again */
            return ( 1) ;                                   /* Timed out                */
        }
    }

    /* The interface is now clear.  Call sendData().   */
    switch (USBCDC_sendData(dataBuf,size,intfNum))
    {
        case kUSBCDC_sendStarted:
            return ( 0) ;
        case kUSBCDC_busNotAvailable:
            return ( 2) ;
        default:
            return ( 4) ;
    }
}

uint8_t USBCDC_bytesInUSBBuffer(BYTE intfNum) {
    assert(intfNum == DEBUG_CHANNEL);
    assert(!_msgs.empty());
    _Msg& msg = _msgs.front();
    assert(msg.len <= std::numeric_limits<uint8_t>::max());
    return msg.len;
}

BYTE USBCDC_sendData (const BYTE* data, WORD size, BYTE intfNum) {
    printf("[FW] SENDING DATA LEN: %zu\n", (size_t)size);
    assert(intfNum == DEBUG_CHANNEL);
    _writeReply(data, size);
    return (kUSBCDC_sendStarted);
}

BYTE USBCDC_receiveData(BYTE* data, WORD size, BYTE intfNum) {
    assert(intfNum == DEBUG_CHANNEL);
    assert(!_msgs.empty());
    _Msg& msg = _msgs.front();
    assert(size <= msg.len);
    memcpy(data, msg.data, size);
    _msgs.pop_front();
    return kUSBCDC_receiveCompleted;
}

BYTE USBCDC_handleDataReceived (BYTE intfNum)
{
    if (intfNum == DEBUG_CHANNEL) //this is our debug channel
    {
        if(BIOS_UsbRxData() == RESPTYP_EXCEPTION )
        {
            V3OP_KillLoop(0);
            BIOS_UsbRxClear();
            BIOS_UsbTxClear();
            BIOS_InitCom();
         }
    }
    return (TRUE);
}



VOID USB_init(VOID)
{
//    //initialize RAM variables
//    bHostAskMoreDataThanAvailable = 0;
//    bFunctionSuspended = FALSE;
//
//    // configuration of USB module
//    USBKEYPID  = 0x9628;            // set KEY and PID to 0x9628 -> access to configuration registers enabled
//    USBPWRCTL = 0;                  // Workaround for USB9
//    __no_operation();               // for workaround USB9
//    USBPWRCTL  = VUSBEN + SLDOEN + SLDOAON /*+ VBOFFIE + VBONIE*/; // keep primary and secondary LDO (3.3 and 1.8 V) enabled
//    USBPHYCTL  = PUSEL;             // use DP and DM as USB terminals (not needed because an external PHY is connected to port 9)
//    IntDelay();
//    bEnumerationStatus = 0x00;      // Device not enumerated yet
}

//WORD wUsbEventMask = 0;                 //used by USB_getEnabledEvents() and USB_setEnabledEvents()
//
//BYTE USB_setEnabledEvents (WORD events)
//{
//    wUsbEventMask = events;
//    return (kUSB_succeed);
//}
