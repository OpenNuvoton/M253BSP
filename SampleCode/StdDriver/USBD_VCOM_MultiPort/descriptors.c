/****************************************************************************/ /**
 * @file     descriptors.c
 * @brief    USBD descriptors
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include "NuMicro.h"
#include "vcom_multi_port.h"

/*----------------------------------------------------------------------------*/
/*!<USB Device Descriptor */
const uint8_t gu8DeviceDescriptor[] = {
    LEN_DEVICE,  /* bLength */
    DESC_DEVICE, /* bDescriptorType */
    0x10, 0x01, /* bcdUSB */
    0xEF,             /* bDeviceClass: miscellaneous device class */
    0x02,             /* bDeviceSubClass: common class */
    0x01,             /* bDeviceProtocol: IAD */
    EP0_MAX_PKT_SIZE, /* bMaxPacketSize0 */
    /* idVendor */
    USBD_VID & 0x00FF, (USBD_VID & 0xFF00) >> 8,
    /* idProduct */
    USBD_PID & 0x00FF, (USBD_PID & 0xFF00) >> 8,
    0x00, 0x03, /* bcdDevice */
    0x01,       /* iManufacture */
    0x02,       /* iProduct */
    0x03,       /* iSerialNumber */
    0x01        /* bNumConfigurations */
};

/*!<USB Configure Descriptor */
const uint8_t gu8ConfigDescriptor[] = {
    LEN_CONFIG,  /* bLength              */
    DESC_CONFIG, /* bDescriptorType      */
    GET_BYTE0(LEN_CONFIGURATION_TOTAL),
    GET_BYTE1(LEN_CONFIGURATION_TOTAL), /* wTotalLength         */
    VCOM_CNT * 2,                       /* bNumInterfaces       */
    0x01,                               /* bConfigurationValue  */
    0x00,                               /* iConfiguration       */
    0xC0,                               /* bmAttributes         */
    0xC8,                               /* MaxPower             */

#if (VCOM_CNT >= 1)
    // IAD
    LEN_IAD,        // bLength        : Interface Descriptor size
    DESC_IAD,       // bDescriptorType: IAD
    INTF_NUM_VCOM0, // bFirstInterface
    0x02,           // bInterfaceCount
    0x02,           // bFunctionClass     : CDC
    0x02,           // bFunctionSubClass
    0x01,           // bFunctionProtocol
    0x02,           // iFunction

    /* VCOM - 0 */
    /* INTERFACE descriptor */
    LEN_INTERFACE,                          /* bLength              */
    DESC_INTERFACE,                         /* bDescriptorType      */
    INTF_NUM_VCOM0,                         /* bInterfaceNumber     */
    0x00,                                   /* bAlternateSetting    */
    0x01,                                   /* bNumEndpoints        */
    DESC_CDC_COMMUNICATION_INTERFACE_CLASS, /* bInterfaceClass      */
    DESC_CDC_ABSTRACT_CONTROL_MODEL,        /* bInterfaceSubClass   */
    0x01,                                   /* bInterfaceProtocol   */
    0x00,                                   /* iInterface           */

    /* Header Functional descriptor */
    0x05,                  /* Size of the descriptor, in bytes */
    DESC_CDC_CS_INTERFACE, /* CS_INTERFACE descriptor type */
    DESC_CDC_HEADER,       /* Header functional descriptor subtype */
    0x10, 0x01, /* Communication device compliant to the communication spec.
                   ver. 1.10 */

    /* Call Management Functional descriptor */
    0x05,                     /* Size of the descriptor, in bytes */
    DESC_CDC_CS_INTERFACE,    /* CS_INTERFACE descriptor type */
    DESC_CDC_CALL_MANAGEMENT, /* Call management functional descriptor */
    0x00, /* BIT0: Whether device handle call management itself. */
    /* BIT1: Whether device can send/receive call management information over a
       Data Class Interface 0 */

    INTF_NUM_VCOM0 + 1, /* Interface number of data class interface optionally
             used for call management */

    /* Abstract Control Management Functional Descriptor  */
    0x04,                            /* Size of the descriptor, in bytes */
    DESC_CDC_CS_INTERFACE,           /* CS_INTERFACE descriptor type */
    DESC_CDC_ABSTRACT_CONTROL_MODEL, /* Abstract control management functional
                                        descriptor subtype */
    0x00,                            /* bmCapabilities       */

    /* Union Functional descriptor */
    0x05,                  /* bLength              */
    DESC_CDC_CS_INTERFACE, /* bDescriptorType: CS_INTERFACE descriptor type */
    DESC_CDC_UNION,        /* bDescriptorSubType   */
    INTF_NUM_VCOM0,        /* bMasterInterface     */
    INTF_NUM_VCOM0 + 1,    /* bSlaveInterface0     */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                 /* bLength          */
    DESC_ENDPOINT,                /* bDescriptorType  */
    (EP_INPUT | INT_IN_EP_NUM_0), /* bEndpointAddress */
    EP_INT,                       /* bmAttributes     */
    GET_BYTE0(EP2_MAX_PKT_SIZE),
    GET_BYTE1(EP2_MAX_PKT_SIZE), /* wMaxPacketSize   */
    0x01,                        /* bInterval        */

    /* INTERFACE descriptor */
    LEN_INTERFACE,      /* bLength              */
    DESC_INTERFACE,     /* bDescriptorType      */
    INTF_NUM_VCOM0 + 1, /* bInterfaceNumber     */
    0x00,               /* bAlternateSetting    */
    0x02,               /* bNumEndpoints        */
    0x0A,               /* bInterfaceClass      */
    0x00,               /* bInterfaceSubClass   */
    0x00,               /* bInterfaceProtocol   */
    0x00,               /* iInterface           */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                  /* bLength          */
    DESC_ENDPOINT,                 /* bDescriptorType  */
    (EP_INPUT | BULK_IN_EP_NUM_0), /* bEndpointAddress */
    EP_BULK,                       /* bmAttributes     */
    GET_BYTE0(EP3_MAX_PKT_SIZE),
    GET_BYTE1(EP3_MAX_PKT_SIZE), /* wMaxPacketSize   */
    0x00,                        /* bInterval        */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                    /* bLength          */
    DESC_ENDPOINT,                   /* bDescriptorType  */
    (EP_OUTPUT | BULK_OUT_EP_NUM_0), /* bEndpointAddress */
    EP_BULK,                         /* bmAttributes     */
    GET_BYTE0(EP4_MAX_PKT_SIZE),
    GET_BYTE1(EP4_MAX_PKT_SIZE), /* wMaxPacketSize   */
    0x00,                        /* bInterval        */
#endif
#if (VCOM_CNT >= 2)
    // IAD
    LEN_IAD,        // bLength: Interface Descriptor size
    DESC_IAD,       // bDescriptorType: IAD
    INTF_NUM_VCOM1, // bFirstInterface
    0x02,           // bInterfaceCount
    0x02,           // bFunctionClass: CDC
    0x02,           // bFunctionSubClass
    0x01,           // bFunctionProtocol
    0x02,           // iFunction

    /* VCOM - 1 */
    /* INTERFACE descriptor */
    LEN_INTERFACE,  /* bLength              */
    DESC_INTERFACE, /* bDescriptorType      */
    INTF_NUM_VCOM1, /* bInterfaceNumber     */
    0x00,           /* bAlternateSetting    */
    0x01,           /* bNumEndpoints        */
    0x02,           /* bInterfaceClass      */
    0x02,           /* bInterfaceSubClass   */
    0x01,           /* bInterfaceProtocol   */
    0x00,           /* iInterface           */

    /* Header Functional descriptor */
    0x05,                  /* Size of the descriptor, in bytes */
    DESC_CDC_CS_INTERFACE, /* CS_INTERFACE descriptor type */
    DESC_CDC_HEADER,       /* Header functional descriptor subtype */
    0x10, 0x01, /* Communication device compliant to the communication spec.
                   ver. 1.10 */

    /* Call Management Functional descriptor */
    0x05,                     /* Size of the descriptor, in bytes */
    DESC_CDC_CS_INTERFACE,    /* CS_INTERFACE descriptor type */
    DESC_CDC_CALL_MANAGEMENT, /* Call management functional descriptor */
    0x00, /* BIT0: Whether device handle call management itself. */
    /* BIT1: Whether device can send/receive call management information over a
       Data Class Interface 0 */
    INTF_NUM_VCOM1 + 1, /* Interface number of data class interface optionally
             used for call management */

    /* Abstract Control Management Functional Descriptor  */
    0x04,                                 /* Size of the descriptor, in bytes */
    DESC_CDC_CS_INTERFACE,                /* CS_INTERFACE descriptor type */
    DESC_CDC_ABSTRACT_CONTROL_MANAGEMENT, /* Abstract control management
                                             functional descriptor subtype */
    0x00,                                 /* bmCapabilities       */

    /* Union Functional descriptor */
    0x05,                  /* bLength              */
    DESC_CDC_CS_INTERFACE, /* bDescriptorType: CS_INTERFACE descriptor type */
    DESC_CDC_UNION,        /* bDescriptorSubType   */
    INTF_NUM_VCOM1,        /* bMasterInterface     */
    INTF_NUM_VCOM1 + 1,    /* bSlaveInterface0     */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                 /* bLength          */
    DESC_ENDPOINT,                /* bDescriptorType  */
    (EP_INPUT | INT_IN_EP_NUM_1), /* bEndpointAddress */
    EP_INT,                       /* bmAttributes     */
    GET_BYTE0(EP5_MAX_PKT_SIZE),
    GET_BYTE1(EP5_MAX_PKT_SIZE), /* wMaxPacketSize   */
    0x01,                        /* bInterval        */

    /* INTERFACE descriptor */
    LEN_INTERFACE,      /* bLength              */
    DESC_INTERFACE,     /* bDescriptorType      */
    INTF_NUM_VCOM1 + 1, /* bInterfaceNumber     */
    0x00,               /* bAlternateSetting    */
    0x02,               /* bNumEndpoints        */
    0x0A,               /* bInterfaceClass      */
    0x00,               /* bInterfaceSubClass   */
    0x00,               /* bInterfaceProtocol   */
    0x00,               /* iInterface           */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                  /* bLength          */
    DESC_ENDPOINT,                 /* bDescriptorType  */
    (EP_INPUT | BULK_IN_EP_NUM_1), /* bEndpointAddress */
    EP_BULK,                       /* bmAttributes     */
    GET_BYTE0(EP6_MAX_PKT_SIZE),
    GET_BYTE1(EP6_MAX_PKT_SIZE), /* wMaxPacketSize   */
    0x00,                        /* bInterval        */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                    /* bLength          */
    DESC_ENDPOINT,                   /* bDescriptorType  */
    (EP_OUTPUT | BULK_OUT_EP_NUM_1), /* bEndpointAddress */
    EP_BULK,                         /* bmAttributes     */
    GET_BYTE0(EP7_MAX_PKT_SIZE),
    GET_BYTE1(EP7_MAX_PKT_SIZE), /* wMaxPacketSize   */
    0x00,                        /* bInterval        */
#endif
#if (VCOM_CNT >= 3)
    // IAD
    LEN_IAD,        // bLength: Interface Descriptor size
    DESC_IAD,       // bDescriptorType: IAD
    INTF_NUM_VCOM2, // bFirstInterface
    0x02,           // bInterfaceCount
    0x02,           // bFunctionClass: CDC
    0x02,           // bFunctionSubClass
    0x01,           // bFunctionProtocol
    0x02,           // iFunction

    /* VCOM - 2 */
    /* INTERFACE descriptor */
    LEN_INTERFACE,  /* bLength              */
    DESC_INTERFACE, /* bDescriptorType      */
    INTF_NUM_VCOM2, /* bInterfaceNumber     */
    0x00,           /* bAlternateSetting    */
    0x01,           /* bNumEndpoints        */
    0x02,           /* bInterfaceClass      */
    0x02,           /* bInterfaceSubClass   */
    0x01,           /* bInterfaceProtocol   */
    0x00,           /* iInterface           */

    /* Header Functional descriptor */
    0x05,                  /* Size of the descriptor, in bytes */
    DESC_CDC_CS_INTERFACE, /* CS_INTERFACE descriptor type */
    DESC_CDC_HEADER,       /* Header functional descriptor subtype */
    0x10, 0x01, /* Communication device compliant to the communication spec.
                   ver. 1.10 */

    /* Call Management Functional descriptor */
    0x05,                     /* Size of the descriptor, in bytes */
    DESC_CDC_CS_INTERFACE,    /* CS_INTERFACE descriptor type */
    DESC_CDC_CALL_MANAGEMENT, /* Call management functional descriptor */
    0x00, /* BIT0: Whether device handle call management itself. */
    /* BIT1: Whether device can send/receive call management information over a
       Data Class Interface 0 */
    INTF_NUM_VCOM2 + 1, /* Interface number of data class interface optionally
             used for call management */

    /* Abstract Control Management Functional Descriptor  */
    0x04,                                 /* Size of the descriptor, in bytes */
    DESC_CDC_CS_INTERFACE,                /* CS_INTERFACE descriptor type */
    DESC_CDC_ABSTRACT_CONTROL_MANAGEMENT, /* Abstract control management
                                             functional descriptor subtype */
    0x00,                                 /* bmCapabilities       */

    /* Union Functional descriptor */
    0x05,                  /* bLength              */
    DESC_CDC_CS_INTERFACE, /* bDescriptorType: CS_INTERFACE descriptor type */
    DESC_CDC_UNION,        /* bDescriptorSubType   */
    INTF_NUM_VCOM2,        /* bMasterInterface     */
    INTF_NUM_VCOM2 + 1,    /* bSlaveInterface0     */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                 /* bLength          */
    DESC_ENDPOINT,                /* bDescriptorType  */
    (EP_INPUT | INT_IN_EP_NUM_2), /* bEndpointAddress */
    EP_INT,                       /* bmAttributes     */
    GET_BYTE0(EP8_MAX_PKT_SIZE),
    GET_BYTE1(EP8_MAX_PKT_SIZE), /* wMaxPacketSize   */
    0x01,                        /* bInterval        */

    /* INTERFACE descriptor */
    LEN_INTERFACE,      /* bLength              */
    DESC_INTERFACE,     /* bDescriptorType      */
    INTF_NUM_VCOM2 + 1, /* bInterfaceNumber     */
    0x00,               /* bAlternateSetting    */
    0x02,               /* bNumEndpoints        */
    0x0A,               /* bInterfaceClass      */
    0x00,               /* bInterfaceSubClass   */
    0x00,               /* bInterfaceProtocol   */
    0x00,               /* iInterface           */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                  /* bLength          */
    DESC_ENDPOINT,                 /* bDescriptorType  */
    (EP_INPUT | BULK_IN_EP_NUM_2), /* bEndpointAddress */
    EP_BULK,                       /* bmAttributes     */
    GET_BYTE0(EP9_MAX_PKT_SIZE),
    GET_BYTE1(EP9_MAX_PKT_SIZE), /* wMaxPacketSize   */
    0x00,                        /* bInterval        */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                    /* bLength          */
    DESC_ENDPOINT,                   /* bDescriptorType  */
    (EP_OUTPUT | BULK_OUT_EP_NUM_2), /* bEndpointAddress */
    EP_BULK,                         /* bmAttributes     */
    GET_BYTE0(EP10_MAX_PKT_SIZE),
    GET_BYTE1(EP10_MAX_PKT_SIZE), /* wMaxPacketSize   */
    0x00,                         /* bInterval        */
#endif
#if (VCOM_CNT >= 4)
    // IAD
    LEN_IAD,        // bLength: Interface Descriptor size
    DESC_IAD,       // bDescriptorType: IAD
    INTF_NUM_VCOM3, // bFirstInterface
    0x02,           // bInterfaceCount
    0x02,           // bFunctionClass: CDC
    0x02,           // bFunctionSubClass
    0x01,           // bFunctionProtocol
    0x02,           // iFunction

    /* VCOM - 3 */
    /* INTERFACE descriptor */
    LEN_INTERFACE,                          /* bLength              */
    DESC_INTERFACE,                         /* bDescriptorType      */
    INTF_NUM_VCOM3,                         /* bInterfaceNumber     */
    0x00,                                   /* bAlternateSetting    */
    0x01,                                   /* bNumEndpoints        */
    DESC_CDC_COMMUNICATION_INTERFACE_CLASS, /* bInterfaceClass      */
    DESC_CDC_ABSTRACT_CONTROL_MODEL,        /* bInterfaceSubClass   */
    0x01,                                   /* bInterfaceProtocol   */
    0x00,                                   /* iInterface           */

    /* Header Functional descriptor */
    0x05,                  /* Size of the descriptor, in bytes */
    DESC_CDC_CS_INTERFACE, /* CS_INTERFACE descriptor type */
    DESC_CDC_HEADER,       /* Header functional descriptor subtype */
    0x10, 0x01, /* Communication device compliant to the communication spec.
                   ver. 1.10 */

    /* Call Management Functional descriptor */
    0x05,                     /* Size of the descriptor, in bytes */
    DESC_CDC_CS_INTERFACE,    /* CS_INTERFACE descriptor type */
    DESC_CDC_CALL_MANAGEMENT, /* Call management functional descriptor */
    0x00, /* BIT0: Whether device handle call management itself. */
    /* BIT1 : Whether device can send/receive call management information over a
       Data Class Interface 0 */
    INTF_NUM_VCOM3 + 1, /* Interface number of data class interface optionally
             used for call management */

    /* Abstract Control Management Functional Descriptor  */
    0x04,                            /* Size of the descriptor, in bytes */
    DESC_CDC_CS_INTERFACE,           /* CS_INTERFACE descriptor type */
    DESC_CDC_ABSTRACT_CONTROL_MODEL, /* Abstract control management functional
                                        descriptor subtype */
    0x00,                            /* bmCapabilities       */

    /* Union Functional descriptor */
    0x05,                  /* bLength              */
    DESC_CDC_CS_INTERFACE, /* bDescriptorType: CS_INTERFACE descriptor type */
    DESC_CDC_UNION,        /* bDescriptorSubType   */
    INTF_NUM_VCOM3,        /* bMasterInterface     */
    INTF_NUM_VCOM3 + 1,    /* bSlaveInterface0     */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                 /* bLength          */
    DESC_ENDPOINT,                /* bDescriptorType  */
    (EP_INPUT | INT_IN_EP_NUM_3), /* bEndpointAddress */
    EP_INT,                       /* bmAttributes     */
    GET_BYTE0(EP11_MAX_PKT_SIZE),
    GET_BYTE1(EP11_MAX_PKT_SIZE), /* wMaxPacketSize   */
    0x01,                         /* bInterval        */

    /* INTERFACE descriptor */
    LEN_INTERFACE,      /* bLength              */
    DESC_INTERFACE,     /* bDescriptorType      */
    INTF_NUM_VCOM3 + 1, /* bInterfaceNumber     */
    0x00,               /* bAlternateSetting    */
    0x02,               /* bNumEndpoints        */
    0x0A,               /* bInterfaceClass      */
    0x00,               /* bInterfaceSubClass   */
    0x00,               /* bInterfaceProtocol   */
    0x00,               /* iInterface           */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                  /* bLength          */
    DESC_ENDPOINT,                 /* bDescriptorType  */
    (EP_INPUT | BULK_IN_EP_NUM_3), /* bEndpointAddress */
    EP_BULK,                       /* bmAttributes     */
    GET_BYTE0(EP12_MAX_PKT_SIZE),
    GET_BYTE1(EP12_MAX_PKT_SIZE), /* wMaxPacketSize   */
    0x00,                         /* bInterval        */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                    /* bLength          */
    DESC_ENDPOINT,                   /* bDescriptorType  */
    (EP_OUTPUT | BULK_OUT_EP_NUM_3), /* bEndpointAddress */
    EP_BULK,                         /* bmAttributes     */
    GET_BYTE0(EP13_MAX_PKT_SIZE),
    GET_BYTE1(EP13_MAX_PKT_SIZE), /* wMaxPacketSize   */
    0x00,                         /* bInterval        */
#endif
#if (VCOM_CNT >= 5)
    // IAD
    LEN_IAD,        // bLength: Interface Descriptor size
    DESC_IAD,       // bDescriptorType: IAD
    INTF_NUM_VCOM4, // bFirstInterface
    0x02,           // bInterfaceCount
    0x02,           // bFunctionClass: CDC
    0x02,           // bFunctionSubClass
    0x01,           // bFunctionProtocol
    0x02,           // iFunction

    /* VCOM - 4 */
    /* INTERFACE descriptor */
    LEN_INTERFACE,  /* bLength              */
    DESC_INTERFACE, /* bDescriptorType      */
    INTF_NUM_VCOM4, /* bInterfaceNumber     */
    0x00,           /* bAlternateSetting    */
    0x01,           /* bNumEndpoints        */
    0x02,           /* bInterfaceClass      */
    0x02,           /* bInterfaceSubClass   */
    0x01,           /* bInterfaceProtocol   */
    0x00,           /* iInterface           */

    /* Header Functional descriptor */
    0x05,                  /* Size of the descriptor, in bytes */
    DESC_CDC_CS_INTERFACE, /* CS_INTERFACE descriptor type */
    DESC_CDC_HEADER,       /* Header functional descriptor subtype */
    0x10, 0x01, /* Communication device compliant to the communication spec.
                   ver. 1.10 */

    /* Call Management Functional descriptor */
    0x05,                     /* Size of the descriptor, in bytes */
    DESC_CDC_CS_INTERFACE,    /* CS_INTERFACE descriptor type */
    DESC_CDC_CALL_MANAGEMENT, /* Call management functional descriptor */
    0x00, /* BIT0: Whether device handle call management itself. */
    /* BIT1: Whether device can send/receive call management information over a
       Data Class Interface 0 */
    INTF_NUM_VCOM4 + 1, /* Interface number of data class interface optionally
             used for call management */

    /* Abstract Control Management Functional Descriptor  */
    0x04,                                 /* Size of the descriptor, in bytes */
    DESC_CDC_CS_INTERFACE,                /* CS_INTERFACE descriptor type */
    DESC_CDC_ABSTRACT_CONTROL_MANAGEMENT, /* Abstract control management
                                             functional descriptor subtype */
    0x00,                                 /* bmCapabilities       */

    /* Union Functional descriptor */
    0x05,                  /* bLength              */
    DESC_CDC_CS_INTERFACE, /* bDescriptorType: CS_INTERFACE descriptor type */
    DESC_CDC_UNION,        /* bDescriptorSubType   */
    INTF_NUM_VCOM4,        /* bMasterInterface     */
    INTF_NUM_VCOM4 + 1,    /* bSlaveInterface0     */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                 /* bLength          */
    DESC_ENDPOINT,                /* bDescriptorType  */
    (EP_INPUT | INT_IN_EP_NUM_4), /* bEndpointAddress */
    EP_INT,                       /* bmAttributes     */
    GET_BYTE0(EP14_MAX_PKT_SIZE),
    GET_BYTE1(EP14_MAX_PKT_SIZE), /* wMaxPacketSize   */
    0x01,                         /* bInterval        */

    /* INTERFACE descriptor */
    LEN_INTERFACE,      /* bLength              */
    DESC_INTERFACE,     /* bDescriptorType      */
    INTF_NUM_VCOM4 + 1, /* bInterfaceNumber     */
    0x00,               /* bAlternateSetting    */
    0x02,               /* bNumEndpoints        */
    0x0A,               /* bInterfaceClass      */
    0x00,               /* bInterfaceSubClass   */
    0x00,               /* bInterfaceProtocol   */
    0x00,               /* iInterface           */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                  /* bLength          */
    DESC_ENDPOINT,                 /* bDescriptorType  */
    (EP_INPUT | BULK_IN_EP_NUM_4), /* bEndpointAddress */
    EP_BULK,                       /* bmAttributes     */
    GET_BYTE0(EP15_MAX_PKT_SIZE),
    GET_BYTE1(EP15_MAX_PKT_SIZE), /* wMaxPacketSize   */
    0x00,                         /* bInterval        */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                    /* bLength          */
    DESC_ENDPOINT,                   /* bDescriptorType  */
    (EP_OUTPUT | BULK_OUT_EP_NUM_4), /* bEndpointAddress */
    EP_BULK,                         /* bmAttributes     */
    GET_BYTE0(EP16_MAX_PKT_SIZE),
    GET_BYTE1(EP16_MAX_PKT_SIZE), /* wMaxPacketSize   */
    0x00,                         /* bInterval        */
#endif
};

/*!<USB Language String Descriptor */
const uint8_t gu8StringLang[4] =
    {
        4,           /* bLength */
        DESC_STRING, /* bDescriptorType */
        0x09, 0x04};

/*!<USB Vendor String Descriptor */
const uint8_t gu8VendorStringDesc[] = {
    16,
    DESC_STRING,
    'N', 0, 'u', 0, 'v', 0, 'o', 0, 't', 0, 'o', 0, 'n', 0
};

/*!<USB Product String Descriptor */
const uint8_t gu8ProductStringDesc[] = {
    32,          /* bLength          */
    DESC_STRING, /* bDescriptorType  */
    'U', 0, 'S', 0, 'B', 0, ' ', 0, 'V', 0, 'i', 0, 'r', 0, 't', 0, 'u', 0, 'a', 0, 'l', 0, ' ', 0, 'C', 0, 'O', 0, 'M', 0
};

const uint8_t gu8StringSerial[26] = {
    26,          // bLength
    DESC_STRING, // bDescriptorType
    'A', 0, '0', 0, '2', 0, '0', 0, '1', 0, '5', 0, '0', 0, '8', 0, '1', 0, '3', 0, '0', 0, '1', 0
};

const uint8_t *gpu8UsbString[4] = {
    gu8StringLang,
    gu8VendorStringDesc,
    gu8ProductStringDesc,
    gu8StringSerial
};

const S_USBD_INFO_T gsInfo = {
    (uint8_t *)gu8DeviceDescriptor,
    (uint8_t *)gu8ConfigDescriptor,
    (uint8_t **)gpu8UsbString,
    (uint8_t **)NULL,
    NULL,
    (uint32_t *)NULL,
    (uint32_t *)NULL,
};
