/****************************************************************************//**
 * @file     cdc_serial.h
 * @version  V0.10
 * @brief    M253 series USB VCOM header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __USBD_CDC_H__
#define __USBD_CDC_H__

/* Define the vendor id and product id */
#define USBD_VID        0x0416
#define USBD_PID        0xDC01

/*!<Define CDC Class Specific Request */
#define SET_LINE_CODE           0x20
#define GET_LINE_CODE           0x21
#define SET_CONTROL_LINE_STATE  0x22

/*-------------------------------------------------------------*/
/* Define EP maximum packet size */
#define EP0_MAX_PKT_SIZE    64
#define EP1_MAX_PKT_SIZE    EP0_MAX_PKT_SIZE
#define EP2_MAX_PKT_SIZE    64
#define EP3_MAX_PKT_SIZE    64
#define EP4_MAX_PKT_SIZE    8

#define EP6_MAX_PKT_SIZE    64
#define EP7_MAX_PKT_SIZE    64

#define SETUP_BUF_BASE      0
#define SETUP_BUF_LEN       8
#define EP0_BUF_BASE        (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP0_BUF_LEN         EP0_MAX_PKT_SIZE
#define EP1_BUF_BASE        (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP1_BUF_LEN         EP1_MAX_PKT_SIZE
#define EP2_BUF_BASE        (EP1_BUF_BASE + EP1_BUF_LEN)
#define EP2_BUF_LEN         EP2_MAX_PKT_SIZE
#define EP3_BUF_BASE        (EP2_BUF_BASE + EP2_BUF_LEN)
#define EP3_BUF_LEN         EP3_MAX_PKT_SIZE
#define EP4_BUF_BASE        (EP3_BUF_BASE + EP3_BUF_LEN)
#define EP4_BUF_LEN         EP4_MAX_PKT_SIZE
#define EP7_BUF_BASE        (EP4_BUF_BASE + EP4_BUF_LEN)
#define EP7_BUF_LEN         EP7_MAX_PKT_SIZE
#define EP6_BUF_BASE        (EP7_BUF_BASE + EP7_BUF_LEN)
#define EP6_BUF_LEN         EP6_MAX_PKT_SIZE

/* Define the interrupt In EP number */
#define BULK_IN_EP_NUM      0x01
#define BULK_OUT_EP_NUM     0x02
#define INT_IN_EP_NUM       0x03
#define BULK_IN_EP_NUM_1    0x06
#define BULK_OUT_EP_NUM_1   0x05


/* Define Descriptor information */
#define USBD_SELF_POWERED               0
#define USBD_REMOTE_WAKEUP              0
#define USBD_MAX_POWER                  50  /* The unit is in 2mA. ex: 50 * 2mA = 100mA */

/************************************************/
/* for CDC class */
/* Line coding structure
  0-3 dwDTERate    Data terminal rate (baudrate), in bits per second
  4   bCharFormat  Stop bits: 0 - 1 Stop bit, 1 - 1.5 Stop bits, 2 - 2 Stop bits
  5   bParityType  Parity:    0 - None, 1 - Odd, 2 - Even, 3 - Mark, 4 - Space
  6   bDataBits    Data bits: 5, 6, 7, 8, 16  */

typedef struct
{
    uint32_t  u32DTERate;     /* Baud rate    */
    uint8_t   u8CharFormat;   /* stop bit     */
    uint8_t   u8ParityType;   /* parity       */
    uint8_t   u8DataBits;     /* data bits    */
} STR_VCOM_LINE_CODING;

/*-------------------------------------------------------------*/
extern volatile int8_t g_i8BulkOutReady;
extern STR_VCOM_LINE_CODING g_sLineCoding;
extern uint16_t g_u16CtrlSignal;
extern volatile uint16_t g_u16ComRbytes;
extern volatile uint16_t g_u16ComRhead;
extern volatile uint16_t g_u16ComRtail;
extern volatile uint16_t g_u16ComTbytes;
extern volatile uint16_t g_u16ComThead;
extern volatile uint16_t g_u16ComTtail;
extern volatile uint8_t *g_pu8RxBuf;
extern volatile uint32_t g_u32RxSize;
extern volatile uint32_t g_u32TxSize;

/*-------------------------------------------------------------*/
void VCOM_MSC_Init(void);
void VCOM_MSC_ClassRequest(void);

void EP2_Handler(void);
void EP3_Handler(void);
void EP7_Handler(void);
void EP6_Handler(void);
void VCOM_LineCoding(void);
void VCOM_TransferData(void);

#endif  /* __USBD_CDC_H_ */
