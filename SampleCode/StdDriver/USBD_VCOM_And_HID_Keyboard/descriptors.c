/****************************************************************************//**
 * @file     descriptors.c
 * @version  V0.10
 * @brief    USBD descriptors
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include "NuMicro.h"
#include "VCOM_and_HID_keyboard.h"

/*!<USB HID Report Descriptor */
const uint8_t HID_KeyboardReportDescriptor[] =
{
    0x05, 0x01,         /* Usage Page(Generic Desktop Controls) */
    0x09, 0x06,         /* Usage(Keyboard) */
    0xA1, 0x01,         /* Collection(Application) */
    0x05, 0x07,         /* Usage Page(Keyboard/Keypad) */
    0x19, 0xE0,         /* Usage Minimum(0xE0) */
    0x29, 0xE7,         /* Usage Maximum(0xE7) */
    0x15, 0x00,         /* Logical Minimum(0x0) */
    0x25, 0x01,         /* Logical Maximum(0x1) */
    0x75, 0x01,         /* Report Size(0x1) */
    0x95, 0x08,         /* Report Count(0x8) */
    0x81, 0x02,         /* Input (Data) => Modifier byte */
    0x95, 0x01,         /* Report Count(0x1) */
    0x75, 0x08,         /* Report Size(0x8) */
    0x81, 0x01,         /* Input (Constant) => Reserved byte */
    0x95, 0x05,         /* Report Count(0x5) */
    0x75, 0x01,         /* Report Size(0x1) */
    0x05, 0x08,         /* Usage Page(LEDs) */
    0x19, 0x01,         /* Usage Minimum(0x1) */
    0x29, 0x05,         /* Usage Maximum(0x5) */
    0x91, 0x02,         /* Output (Data) => LED report */
    0x95, 0x01,         /* Report Count(0x1) */
    0x75, 0x03,         /* Report Size(0x3) */
    0x91, 0x01,         /* Output (Constant) => LED report padding */
    0x95, 0x06,         /* Report Count(0x6) */
    0x75, 0x08,         /* Report Size(0x8) */
    0x15, 0x00,         /* Logical Minimum(0x0) */
    0x25, 0x65,         /* Logical Maximum(0x65) */
    0x05, 0x07,         /* Usage Page(Keyboard/Keypad) */
    0x19, 0x00,         /* Usage Minimum(0x0) */
    0x29, 0x65,         /* Usage Maximum(0x65) */
    0x81, 0x00,         /* Input (Data) */
    0xC0,               /* End Collection */
};

/*----------------------------------------------------------------------------*/
/*!<USB Device Descriptor */
const uint8_t gu8DeviceDescriptor[] = {
    LEN_DEVICE,  /* bLength */
    DESC_DEVICE, /* bDescriptorType */
    0x10, 0x01, /* bcdUSB */
    0xEF,             /* bDeviceClass: IAD */
    0x02,             /* bDeviceSubClass */
    0x01,             /* bDeviceProtocol */
    EP0_MAX_PKT_SIZE, /* bMaxPacketSize0 */
    /* idVendor */
    USBD_VID & 0x00FF, (USBD_VID & 0xFF00) >> 8,
    /* idProduct */
    USBD_PID & 0x00FF, (USBD_PID & 0xFF00) >> 8,
    0x00, 0x00, /* bcdDevice */
    0x01,       /* iManufacture */
    0x02,       /* iProduct */
    0x03,       /* iSerialNumber */
    0x01 /* bNumConfigurations */
};

/*!<USB Configure Descriptor */
const uint8_t gu8ConfigDescriptor[] = {
    LEN_CONFIG,  /* bLength              */
    DESC_CONFIG, /* bDescriptorType      */
    0x64, 0x00,  /* wTotalLength         */
    0x03,        /* bNumInterfaces       */
    0x01,        /* bConfigurationValue  */
    0x00,        /* iConfiguration       */
    0xC0,        /* bmAttributes         */
    0x32,        /* MaxPower             */

    // IAD
    0x08, // bLength: Interface Descriptor size
    0x0B, // bDescriptorType: IAD
    0x00, // bFirstInterface
    0x02, // bInterfaceCount
    0x02, // bFunctionClass: CDC
    0x02, // bFunctionSubClass
    0x01, // bFunctionProtocol
    0x02, // iFunction

    /* VCOM */
    /* INTERFACE descriptor */
    LEN_INTERFACE,  /* bLength              */
    DESC_INTERFACE, /* bDescriptorType      */
    0x00,           /* bInterfaceNumber     */
    0x00,           /* bAlternateSetting    */
    0x01,           /* bNumEndpoints        */
    0x02,           /* bInterfaceClass      */
    0x02,           /* bInterfaceSubClass   */
    0x01,           /* bInterfaceProtocol   */
    0x00,           /* iInterface           */

    /* Communication Class Specified INTERFACE descriptor */
    0x05,       /* Size of the descriptor, in bytes */
    0x24,       /* CS_INTERFACE descriptor type */
    0x00,       /* Header functional descriptor subtype */
    0x10, 0x01, /* Communication device compliant to the communication spec.
                   ver. 1.10 */

    /* Communication Class Specified INTERFACE descriptor */
    0x05, /* Size of the descriptor, in bytes */
    0x24, /* CS_INTERFACE descriptor type */
    0x01, /* Call management functional descriptor */
    0x00, /* BIT0: Whether device handle call management itself. */
    /* BIT1: Whether device can send/receive call management information over a
       Data Class Interface 0 */
    0x01, /* Interface number of data class interface optionally used for call
             management */

    /* Communication Class Specified INTERFACE descriptor */
    0x04, /* Size of the descriptor, in bytes */
    0x24, /* CS_INTERFACE descriptor type */
    0x02, /* Abstract control management functional descriptor subtype */
    0x00, /* bmCapabilities       */

    /* Communication Class Specified INTERFACE descriptor */
    0x05, /* bLength              */
    0x24, /* bDescriptorType: CS_INTERFACE descriptor type */
    0x06, /* bDescriptorSubType   */
    0x00, /* bMasterInterface     */
    0x01, /* bSlaveInterface0     */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,               /* bLength          */
    DESC_ENDPOINT,              /* bDescriptorType  */
    (EP_INPUT | INT_IN_EP_NUM), /* bEndpointAddress */
    EP_INT,                     /* bmAttributes     */
    EP4_MAX_PKT_SIZE, 0x00,     /* wMaxPacketSize   */
    0x01,                       /* bInterval        */

    /* INTERFACE descriptor */
    LEN_INTERFACE,  /* bLength              */
    DESC_INTERFACE, /* bDescriptorType      */
    0x01,           /* bInterfaceNumber     */
    0x00,           /* bAlternateSetting    */
    0x02,           /* bNumEndpoints        */
    0x0A,           /* bInterfaceClass      */
    0x00,           /* bInterfaceSubClass   */
    0x00,           /* bInterfaceProtocol   */
    0x00,           /* iInterface           */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                /* bLength          */
    DESC_ENDPOINT,               /* bDescriptorType  */
    (EP_INPUT | BULK_IN_EP_NUM), /* bEndpointAddress */
    EP_BULK,                     /* bmAttributes     */
    EP2_MAX_PKT_SIZE, 0x00,      /* wMaxPacketSize   */
    0x00,                        /* bInterval        */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                  /* bLength          */
    DESC_ENDPOINT,                 /* bDescriptorType  */
    (EP_OUTPUT | BULK_OUT_EP_NUM), /* bEndpointAddress */
    EP_BULK,                       /* bmAttributes     */
    EP3_MAX_PKT_SIZE, 0x00,        /* wMaxPacketSize   */
    0x00,                          /* bInterval        */

    /* HID class device */
    /* I/F descr: HID keyboard*/
    LEN_INTERFACE,  /* bLength */
    DESC_INTERFACE, /* bDescriptorType */
    0x02,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x01,           /* bNumEndpoints */
    0x03,           /* bInterfaceClass */

    // Note: set report protocol(0),Set_Protocol / Get_protocol request is
    // options. CV3.0 Test pass
    0x00,         /* bInterfaceSubClass */
    HID_KEYBOARD, /* bInterfaceProtocol */
    0x00,         /* iInterface */

    /* HID Descriptor */
    LEN_HID,      /* Size of this descriptor in UINT8s. */
    DESC_HID,     /* HID descriptor type. */
    0x10, 0x01,   /* HID Class Spec. release number. */
    0x00,         /* H/W target country. */
    0x01,         /* Number of HID class descriptors to follow. */
    DESC_HID_RPT, /* Descriptor type. */
    /* Total length of report descriptor. */
    sizeof(HID_KeyboardReportDescriptor) & 0x00FF,
    (sizeof(HID_KeyboardReportDescriptor) & 0xFF00) >> 8,

    /* EP Descriptor: interrupt in. */
    LEN_ENDPOINT,                 /* bLength */
    DESC_ENDPOINT,                /* bDescriptorType */
    (INT_IN_EP_NUM_1 | EP_INPUT), /* bEndpointAddress */
    EP_INT,                       /* bmAttributes */
    /* wMaxPacketSize */
    EP5_MAX_PKT_SIZE & 0x00FF, (EP5_MAX_PKT_SIZE & 0xFF00) >> 8,
    HID_DEFAULT_INT_IN_INTERVAL /* bInterval */
};

/*!<USB Language String Descriptor */
const uint8_t gu8StringLang[4] =
{
    4,              /* bLength */
    DESC_STRING,    /* bDescriptorType */
    0x09, 0x04
};

/*!<USB Vendor String Descriptor */
const uint8_t gu8VendorStringDesc[] =
{
    16,
    DESC_STRING,
    'N', 0, 'u', 0, 'v', 0, 'o', 0, 't', 0, 'o', 0, 'n', 0
};

/*!<USB Product String Descriptor */
const uint8_t gu8ProductStringDesc[] =
{
    22,             /* bLength          */
    DESC_STRING,    /* bDescriptorType  */
    'U', 0, 'S', 0, 'B', 0, ' ', 0, 'D', 0, 'e', 0, 'v', 0, 'i', 0, 'c', 0, 'e', 0
};

const uint8_t gu8StringSerial[26] =
{
    26,             // bLength
    DESC_STRING,    // bDescriptorType
    'A', 0, '0', 0, '2', 0, '0', 0, '1', 0, '5', 0, '0', 0, '8', 0, '1', 0, '8', 0, '0', 0, '1', 0
};

const uint8_t *gpu8UsbString[4] =
{
    gu8StringLang,
    gu8VendorStringDesc,
    gu8ProductStringDesc,
    gu8StringSerial
};

const uint8_t *gpu8UsbHidReport[3] =
{
    NULL,
    NULL,
    HID_KeyboardReportDescriptor
};

const uint32_t gu32UsbHidReportLen[3] =
{
    0,
    0,
    sizeof(HID_KeyboardReportDescriptor)
};

const uint32_t gu32ConfigHidDescIdx[3] =
{
    0,
    0,
    (sizeof(gu8ConfigDescriptor) - LEN_HID - LEN_ENDPOINT)
};

const S_USBD_INFO_T gsInfo =
{
    (uint8_t *) gu8DeviceDescriptor,
    (uint8_t *) gu8ConfigDescriptor,
    (uint8_t **)gpu8UsbString,
    (uint8_t **)gpu8UsbHidReport,
    NULL,
    (uint32_t *)gu32UsbHidReportLen,
    (uint32_t *)gu32ConfigHidDescIdx,
};

