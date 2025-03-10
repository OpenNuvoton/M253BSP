/****************************************************************************//**
 * @file     descriptors.c
 * @version  V0.10
 * @brief    USBD descriptor file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
/*!<Includes */
#include "NuMicro.h"
#include "micro_printer.h"

/*----------------------------------------------------------------------------*/
/*!<USB Device Descriptor */
const uint8_t gu8DeviceDescriptor[] = {
    LEN_DEVICE,  /* bLength */
    DESC_DEVICE, /* bDescriptorType */
    0x10, 0x01, /* bcdUSB */
    0x00,             /* bDeviceClass */
    0x00,             /* bDeviceSubClass */
    0x00,             /* bDeviceProtocol */
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
const uint8_t gu8ConfigDescriptor[] =
{
    LEN_CONFIG,     /* bLength              */
    DESC_CONFIG,    /* bDescriptorType      */
    0x4E, 0x00,     /* wTotalLength         */
    0x01,           /* bNumInterfaces       */
    0x01,           /* bConfigurationValue  */
    0x00,           /* iConfiguration       */
    0xC0,           /* bmAttributes         */
    0x32,           /* MaxPower             */

    /* INTERFACE descriptor */
    LEN_INTERFACE,  /* bLength                        */
    DESC_INTERFACE, /* bDescriptorType                */
    0x00,           /* bInterfaceNumber               */
    0x00,           /* bAlternateSetting              */
    0x01,           /* bNumEndpoints                  */
    0x07,           /* bInterfaceClass: printer class */
    0x01,           /* bInterfaceSubClass             */
    0x01,           /* bInterfaceProtocol             */
    0x00,           /* iInterface                     */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                   /* bLength          */
    DESC_ENDPOINT,                  /* bDescriptorType  */
    (EP_OUTPUT | BULK_OUT_EP_NUM),  /* bEndpointAddress */
    EP_BULK,                        /* bmAttributes     */
    EP3_MAX_PKT_SIZE, 0x00,         /* wMaxPacketSize   */
    0x00,                           /* bInterval        */

    /* INTERFACE descriptor */
    LEN_INTERFACE,  /* bLength                        */
    DESC_INTERFACE, /* bDescriptorType                */
    0x00,           /* bInterfaceNumber               */
    0x01,           /* bAlternateSetting              */
    0x02,           /* bNumEndpoints                  */
    0x07,           /* bInterfaceClass: printer class */
    0x01,           /* bInterfaceSubClass             */
    0x02,           /* bInterfaceProtocol             */
    0x00,           /* iInterface                     */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                   /* bLength          */
    DESC_ENDPOINT,                  /* bDescriptorType  */
    (EP_OUTPUT | BULK_OUT_EP_NUM),  /* bEndpointAddress */
    EP_BULK,                        /* bmAttributes     */
    EP3_MAX_PKT_SIZE, 0x00,         /* wMaxPacketSize   */
    0x00,                           /* bInterval        */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                   /* bLength          */
    DESC_ENDPOINT,                  /* bDescriptorType  */
    (EP_INPUT | BULK_IN_EP_NUM),    /* bEndpointAddress */
    EP_BULK,                        /* bmAttributes     */
    EP2_MAX_PKT_SIZE, 0x00,         /* wMaxPacketSize   */
    0x00,                           /* bInterval        */

    /* INTERFACE descriptor */
    LEN_INTERFACE,  /* bLength              */
    DESC_INTERFACE, /* bDescriptorType      */
    0x00,           /* bInterfaceNumber     */
    0x02,           /* bAlternateSetting    */
    0x03,           /* bNumEndpoints        */
    0xFF,           /* bInterfaceClass      */
    0x00,           /* bInterfaceSubClass   */
    0xFF,           /* bInterfaceProtocol   */
    0x00,           /* iInterface           */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                   /* bLength          */
    DESC_ENDPOINT,                  /* bDescriptorType  */
    (EP_OUTPUT | BULK_OUT_EP_NUM),  /* bEndpointAddress */
    EP_BULK,                        /* bmAttributes     */
    EP3_MAX_PKT_SIZE, 0x00,         /* wMaxPacketSize   */
    0x00,                           /* bInterval        */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                   /* bLength          */
    DESC_ENDPOINT,                  /* bDescriptorType  */
    (EP_INPUT | BULK_IN_EP_NUM),    /* bEndpointAddress */
    EP_BULK,                        /* bmAttributes     */
    EP2_MAX_PKT_SIZE, 0x00,         /* wMaxPacketSize   */
    0x00,                           /* bInterval        */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                   /* bLength          */
    DESC_ENDPOINT,                  /* bDescriptorType  */
    (EP_INPUT | INT_IN_EP_NUM),     /* bEndpointAddress */
    EP_INT,                         /* bmAttributes     */
    EP4_MAX_PKT_SIZE, 0x00,         /* wMaxPacketSize   */
    0x01                            /* bInterval        */
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
    28,             /* bLength          */
    DESC_STRING,    /* bDescriptorType  */
    'M', 0, 'i', 0, 'c', 0, 'r', 0, 'o', 0, ' ', 0, 'P', 0, 'r', 0, 'i', 0, 'n', 0, 't', 0, 'e', 0, 'r', 0
};


const uint8_t gu8StringSerial[26] =
{
    26,             // bLength
    DESC_STRING,    // bDescriptorType
    'A', 0, '0', 0, '2', 0, '0', 0, '1', 0, '5', 0, '0', 0, '8', 0, '0', 0, '1', 0, '0', 0, '3', 0
};

const uint8_t *gpu8UsbString[4] =
{
    gu8StringLang,
    gu8VendorStringDesc,
    gu8ProductStringDesc,
    gu8StringSerial
};

const S_USBD_INFO_T gsInfo =
{
    (uint8_t *) gu8DeviceDescriptor,
    (uint8_t *) gu8ConfigDescriptor,
    (uint8_t **)gpu8UsbString,
    NULL,
    NULL,
    NULL,
    NULL,
};


