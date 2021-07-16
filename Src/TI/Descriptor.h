#pragma once
#include "USBTypes.h"

namespace TI::Descriptor {
using namespace USBTypes;

#define SIZEOF_DEVICE_DESCRIPTOR        0x12

/*-----------------------------------------------------------------------------+
| Device Descriptor                                                            |
|-----------------------------------------------------------------------------*/
BYTE const abromDeviceDescriptor[SIZEOF_DEVICE_DESCRIPTOR] = {
    SIZEOF_DEVICE_DESCRIPTOR,               // Length of this descriptor
    DESC_TYPE_DEVICE,                       // Type code of this descriptor
    0x00, 0x02,                             // Release of USB spec
    0xef,                                   // Device's base class code
    0x02,                                   // Device's sub class code
    0x01,                                   // Device's protocol type code
    EP0_PACKET_SIZE,                        // End point 0's packet size
    USB_VID&0xFF, USB_VID>>8,               // Vendor ID for device, TI=0x0451
                                            // You can order your own VID at www.usb.org
    USB_PID&0xFF, USB_PID>>8,               // Product ID for device,
                                            // this ID is to only with this example
    VER_FW_L, VER_FW_H,                     // Revision level of device
    1,                                      // Index of manufacturer name string desc
    2,                                      // Index of product name string desc
    USB_STR_INDEX_SERNUM,                   // Index of serial number string desc
    1                                       //  Number of configurations supported
};

/*-----------------------------------------------------------------------------+
| Configuration Descriptor                                                     |
|-----------------------------------------------------------------------------*/
const struct abromConfigurationDescriptorGroup abromConfigurationDescriptorGroup=
{
    /* Generic part */
    {
        // CONFIGURATION DESCRIPTOR (9 bytes)
        SIZEOF_CONFIG_DESCRIPTOR,                          // bLength
        DESC_TYPE_CONFIG,                                  // bDescriptorType
        DESCRIPTOR_TOTAL_LENGTH, 0x00,                     // wTotalLength
        USB_NUM_INTERFACES,                 	           // bNumInterfaces
        USB_CONFIG_VALUE,                                  // bConfigurationvalue
        CONFIG_STRING_INDEX,                               // iConfiguration Description offset
        USB_SUPPORT_SELF_POWERED | USB_SUPPORT_REM_WAKE,   // bmAttributes, bus power, remote wakeup
        USB_MAX_POWER                                      // Max. Power Consumption
    },

    /******************************************************* start of CDC*************************************/

    {
        /* start CDC[0] */
        {

           //Interface Association Descriptor
            0X08,                              // bLength
            DESC_TYPE_IAD,                     // bDescriptorType = 11
            CDC0_COMM_INTERFACE,               // bFirstInterface
            0x02,                              // bInterfaceCount
            0x02,                              // bFunctionClass (Communication Class)
            0x02,                              // bFunctionSubClass (Abstract Control Model)
            0x01,                              // bFunctionProcotol (V.25ter, Common AT commands)
            INTF_STRING_INDEX + 0,             // iInterface

            //INTERFACE DESCRIPTOR (9 bytes)
            0x09,                              // bLength: Interface Descriptor size
            DESC_TYPE_INTERFACE,               // bDescriptorType: Interface
            CDC0_COMM_INTERFACE,               // bInterfaceNumber
            0x00,                              // bAlternateSetting: Alternate setting
            0x01,                              // bNumEndpoints: Three endpoints used
            0x02,                              // bInterfaceClass: Communication Interface Class
            0x02,                              // bInterfaceSubClass: Abstract Control Model
            0x01,                              // bInterfaceProtocol: Common AT commands
            INTF_STRING_INDEX + 0,             // iInterface:

            //Header Functional Descriptor
            0x05,	                            // bLength: Endpoint Descriptor size
            0x24,	                            // bDescriptorType: CS_INTERFACE
            0x00,	                            // bDescriptorSubtype: Header Func Desc
            0x10,	                            // bcdCDC: spec release number
            0x01,

            //Call Managment Functional Descriptor
            0x05,	                            // bFunctionLength
            0x24,	                            // bDescriptorType: CS_INTERFACE
            0x01,	                            // bDescriptorSubtype: Call Management Func Desc
            0x00,	                            // bmCapabilities: D0+D1
            CDC0_DATA_INTERFACE,                // bDataInterface: 0

            //ACM Functional Descriptor
            0x04,	                            // bFunctionLength
            0x24,	                            // bDescriptorType: CS_INTERFACE
            0x02,	                            // bDescriptorSubtype: Abstract Control Management desc
            0x02,	                            // bmCapabilities

            // Union Functional Descriptor
            0x05,                               // Size, in bytes
            0x24,                               // bDescriptorType: CS_INTERFACE
            0x06,	                            // bDescriptorSubtype: Union Functional Desc
            CDC0_COMM_INTERFACE,                // bMasterInterface -- the controlling intf for the union
            CDC0_DATA_INTERFACE,                // bSlaveInterface -- the controlled intf for the union

            //EndPoint Descriptor for Interrupt endpoint
            SIZEOF_ENDPOINT_DESCRIPTOR,         // bLength: Endpoint Descriptor size
            DESC_TYPE_ENDPOINT,                 // bDescriptorType: Endpoint
            CDC0_INTEP_ADDR,                    // bEndpointAddress: (IN2)
            EP_DESC_ATTR_TYPE_INT,	            // bmAttributes: Interrupt
            0x40, 0x00,                         // wMaxPacketSize, 64 bytes
            0xFF,	                            // bInterval

            //DATA INTERFACE DESCRIPTOR (9 bytes)
            0x09,	                            // bLength: Interface Descriptor size
            DESC_TYPE_INTERFACE,	            // bDescriptorType: Interface
            CDC0_DATA_INTERFACE,                // bInterfaceNumber
            0x00,                               // bAlternateSetting: Alternate setting
            0x02,                               // bNumEndpoints: Three endpoints used
            0x0A,                               // bInterfaceClass: Data Interface Class
            0x00,                               // bInterfaceSubClass:
            0x00,                               // bInterfaceProtocol: No class specific protocol required
            0x00,	                            // iInterface:

            //EndPoint Descriptor for Output endpoint
            SIZEOF_ENDPOINT_DESCRIPTOR,         // bLength: Endpoint Descriptor size
            DESC_TYPE_ENDPOINT,	                // bDescriptorType: Endpoint
            CDC0_OUTEP_ADDR,	                // bEndpointAddress: (OUT3)
            EP_DESC_ATTR_TYPE_BULK,	            // bmAttributes: Bulk
            0x40, 0x00,                         // wMaxPacketSize, 64 bytes
            0xFF, 	                            // bInterval: ignored for Bulk transfer

            //EndPoint Descriptor for Input endpoint
            SIZEOF_ENDPOINT_DESCRIPTOR,         // bLength: Endpoint Descriptor size
            DESC_TYPE_ENDPOINT,	                // bDescriptorType: Endpoint
            CDC0_INEP_ADDR,	                    // bEndpointAddress: (IN3)
            EP_DESC_ATTR_TYPE_BULK,	            // bmAttributes: Bulk
            0x40, 0x00,                         // wMaxPacketSize, 64 bytes
            0xFF                                // bInterval: ignored for bulk transfer
        },

        /* end CDC[0]*/

        /* start CDC[1] */
        {

           //Interface Association Descriptor
            0X08,                              // bLength
            DESC_TYPE_IAD,                     // bDescriptorType = 11
            CDC1_COMM_INTERFACE,               // bFirstInterface
            0x02,                              // bInterfaceCount
            0x02,                              // bFunctionClass (Communication Class)
            0x02,                              // bFunctionSubClass (Abstract Control Model)
            0x01,                              // bFunctionProcotol (V.25ter, Common AT commands)
            INTF_STRING_INDEX + 1,             // iInterface

            //INTERFACE DESCRIPTOR (9 bytes)
            0x09,                              // bLength: Interface Descriptor size
            DESC_TYPE_INTERFACE,               // bDescriptorType: Interface
            CDC1_COMM_INTERFACE,               // bInterfaceNumber
            0x00,                              // bAlternateSetting: Alternate setting
            0x01,                              // bNumEndpoints: Three endpoints used
            0x02,                              // bInterfaceClass: Communication Interface Class
            0x02,                              // bInterfaceSubClass: Abstract Control Model
            0x01,                              // bInterfaceProtocol: Common AT commands
            INTF_STRING_INDEX + 1,             // iInterface:

            //Header Functional Descriptor
            0x05,	                            // bLength: Endpoint Descriptor size
            0x24,	                            // bDescriptorType: CS_INTERFACE
            0x00,	                            // bDescriptorSubtype: Header Func Desc
            0x10,	                            // bcdCDC: spec release number
            0x01,

            //Call Managment Functional Descriptor
            0x05,	                            // bFunctionLength
            0x24,	                            // bDescriptorType: CS_INTERFACE
            0x01,	                            // bDescriptorSubtype: Call Management Func Desc
            0x00,	                            // bmCapabilities: D0+D1
            CDC1_DATA_INTERFACE,                // bDataInterface: 0

            //ACM Functional Descriptor
            0x04,	                            // bFunctionLength
            0x24,	                            // bDescriptorType: CS_INTERFACE
            0x02,	                            // bDescriptorSubtype: Abstract Control Management desc
            0x02,	                            // bmCapabilities

            // Union Functional Descriptor
            0x05,                               // Size, in bytes
            0x24,                               // bDescriptorType: CS_INTERFACE
            0x06,	                            // bDescriptorSubtype: Union Functional Desc
            CDC1_COMM_INTERFACE,                // bMasterInterface -- the controlling intf for the union
            CDC1_DATA_INTERFACE,                // bSlaveInterface -- the controlled intf for the union

            //EndPoint Descriptor for Interrupt endpoint
            SIZEOF_ENDPOINT_DESCRIPTOR,         // bLength: Endpoint Descriptor size
            DESC_TYPE_ENDPOINT,                 // bDescriptorType: Endpoint
            CDC1_INTEP_ADDR,                    // bEndpointAddress: (IN2)
            EP_DESC_ATTR_TYPE_INT,	            // bmAttributes: Interrupt
            0x40, 0x00,                         // wMaxPacketSize, 64 bytes
            0xFF,	                            // bInterval

            //DATA INTERFACE DESCRIPTOR (9 bytes)
            0x09,	                            // bLength: Interface Descriptor size
            DESC_TYPE_INTERFACE,	            // bDescriptorType: Interface
            CDC1_DATA_INTERFACE,                // bInterfaceNumber
            0x00,                               // bAlternateSetting: Alternate setting
            0x02,                               // bNumEndpoints: Three endpoints used
            0x0A,                               // bInterfaceClass: Data Interface Class
            0x00,                               // bInterfaceSubClass:
            0x00,                               // bInterfaceProtocol: No class specific protocol required
            0x00,	                            // iInterface:

            //EndPoint Descriptor for Output endpoint
            SIZEOF_ENDPOINT_DESCRIPTOR,         // bLength: Endpoint Descriptor size
            DESC_TYPE_ENDPOINT,	                // bDescriptorType: Endpoint
            CDC1_OUTEP_ADDR,	                // bEndpointAddress: (OUT3)
            EP_DESC_ATTR_TYPE_BULK,	            // bmAttributes: Bulk
            0x40, 0x00,                         // wMaxPacketSize, 64 bytes
            0xFF, 	                            // bInterval: ignored for Bulk transfer

            //EndPoint Descriptor for Input endpoint
            SIZEOF_ENDPOINT_DESCRIPTOR,         // bLength: Endpoint Descriptor size
            DESC_TYPE_ENDPOINT,	                // bDescriptorType: Endpoint
            CDC1_INEP_ADDR,	                    // bEndpointAddress: (IN3)
            EP_DESC_ATTR_TYPE_BULK,	            // bmAttributes: Bulk
            0x40, 0x00,                         // wMaxPacketSize, 64 bytes
            0xFF                                // bInterval: ignored for bulk transfer
        }

        /* end CDC[1]*/

    }
    /******************************************************* end of CDC**************************************/

};

/*-----------------------------------------------------------------------------+
| String Descriptor                                                            |
|-----------------------------------------------------------------------------*/
constexpr auto String0 = USB::SupportedLanguagesDescriptorMake({0x0409});
constexpr auto String1 = USB::StringDescriptorMake("Texas Instruments");        // Manufacturer
constexpr auto String2 = USB::StringDescriptorMake("MSP Tools Driver");         // Product
constexpr auto String3 = USB::StringDescriptorMake("MSPProbeSim");              // Serial Number
constexpr auto String4 = USB::StringDescriptorMake("MSP430 USB");               // Configuration String
constexpr auto String5 = USB::StringDescriptorMake("MSP Debug Interface");      // Interface String
constexpr auto String6 = USB::StringDescriptorMake("MSP Application UART");     // Interface String

const USB::StringDescriptor* Strings[] = {
    &String0,
    &String1,
    &String2,
    &String3,
    &String4,
    &String5,
    &String6,
};

} // TI::Descriptor
