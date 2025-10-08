/****************************************************************************//**
 * @file     vcom_multi_port.h
 * @version  V0.10
 * @brief    M253 series USB VCOM header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __USBD_VCOM_MULTI_PORT_H__
#define __USBD_VCOM_MULTI_PORT_H__

/* Multi Port Configuration*/
#define VCOM_CNT (5)
#define TUNE_UART_TIMTOUT (80UL)
#define UART_FIFO_DEPTH (UART_FIFO_RFITL_8BYTES)

/**
 * | -------- | --------- | ------------- | -------- | ------------ | ------- | --------- | -------- |
 * | VCOM_CNT | UART port | USB Interface | IDX_VCOM | USB INTR NUM | Address | Endpoints | Function |
 * | -------- | --------- | ------------- | -------- | ------------ | ------- | --------- | -------- |
 * |          |           |               |          |              | 0x00    | EP0       | CTRL IN  |
 * |          |           |               |          |              | 0x00    | EP1       | CTRL OUT |
 * | 1        | UART0     | VCOM0         | 0        | 0            | 0x82    | EP2       | INT IN   |
 * |          |           |               |          | 1            | 0x83    | EP3       | BULK IN  |
 * |          |           |               |          | 1            | 0x03    | EP4       | BULK OUT |
 * | 2        | UART1     | VCOM1         | 1        | 2            | 0x84    | EP5       | INT IN   |
 * |          |           |               |          | 3            | 0x85    | EP6       | BULK IN  |
 * |          |           |               |          | 3            | 0x05    | EP7       | BULK OUT |
 * | 3        | UART2     | VCOM2         | 2        | 4            | 0x86    | EP8       | INT IN   |
 * |          |           |               |          | 5            | 0x87    | EP9       | BULK IN  |
 * |          |           |               |          | 5            | 0x07    | EP10      | BULK OUT |
 * | 4        | UART3     | VCOM3         | 3        | 6            | 0x88    | EP11      | INT IN   |
 * |          |           |               |          | 7            | 0x89    | EP12      | BULK IN  |
 * |          |           |               |          | 7            | 0x09    | EP13      | BULK OUT |
 * | 5        | UART4     | VCOM4         | 4        | 8            | 0x8A    | EP14      | INT IN   |
 * |          |           |               |          | 9            | 0x8B    | EP15      | BULK IN  |
 * |          |           |               |          | 9            | 0x0B    | EP16      | BULK OUT |
 * | -------- | --------- | ------------- | -------- | ------------ | ------- | --------- | -------- |
 */


#if (VCOM_CNT>=1)
    #define IDX_VCOM0 0x0
    #define INTF_NUM_VCOM0 (IDX_VCOM0*2)
#endif
#if (VCOM_CNT>=2)
    #define IDX_VCOM1 0x1
    #define INTF_NUM_VCOM1 (IDX_VCOM1*2)
#endif
#if (VCOM_CNT>=3)
    #define IDX_VCOM2 0x2
    #define INTF_NUM_VCOM2 (IDX_VCOM2*2)
#endif
#if (VCOM_CNT>=4)
    #define IDX_VCOM3 0x3
    #define INTF_NUM_VCOM3 (IDX_VCOM3*2)
#endif
#if (VCOM_CNT>=5)
    #define IDX_VCOM4 0x4
    #define INTF_NUM_VCOM4 (IDX_VCOM4*2)
#endif

#if ((VCOM_CNT>=6) || (VCOM_CNT<=0))
    #error VCOM number error, VCOM_CNT must be 1 ~ 5
#endif

#define LEN_VCOM_COMMAND_INTR (0x5UL + 0x5UL + 0x4UL + 0x5UL)
#define VCOM_LEN ( LEN_IAD + (LEN_INTERFACE * 2) + LEN_VCOM_COMMAND_INTR + (LEN_ENDPOINT * 3))
#define LEN_CONFIGURATION_TOTAL (LEN_CONFIG+ (VCOM_LEN * VCOM_CNT))


/* Define the vendor id and product id */
#define USBD_VID        0x0416
#define USBD_PID        0x50A1

/*!<Define CDC Class Specific Descriptor */
#define DESC_CDC_COMMUNICATION_INTERFACE_CLASS       0x02
#define DESC_CDC_DATA_INTERFACE_CLASS                0x0A
#define DESC_CDC_ABSTRACT_CONTROL_MODEL              0x02
#define DESC_CDC_CS_INTERFACE                        0x24
#define DESC_CDC_HEADER                              0x00
#define DESC_CDC_CALL_MANAGEMENT                     0x01
#define DESC_CDC_ABSTRACT_CONTROL_MANAGEMENT         0x02
#define DESC_CDC_UNION                               0x06

/*!<Define CDC Class Specific Request */
#define SET_LINE_CODE           0x20
#define GET_LINE_CODE           0x21
#define SET_CONTROL_LINE_STATE  0x22

/*-------------------------------------------------------------*/
/* Define EP maximum packet size */
#define DATA_EP_MAX_PKT_SIZE    64
#define EP0_MAX_PKT_SIZE        DATA_EP_MAX_PKT_SIZE
#define EP1_MAX_PKT_SIZE        EP0_MAX_PKT_SIZE

#if (VCOM_CNT>=1)
    #define EP2_MAX_PKT_SIZE    8
    #define EP3_MAX_PKT_SIZE    DATA_EP_MAX_PKT_SIZE
    #define EP4_MAX_PKT_SIZE    DATA_EP_MAX_PKT_SIZE
#endif
#if (VCOM_CNT>=2)
    #define EP5_MAX_PKT_SIZE    8
    #define EP6_MAX_PKT_SIZE    DATA_EP_MAX_PKT_SIZE
    #define EP7_MAX_PKT_SIZE    DATA_EP_MAX_PKT_SIZE
#endif

#if (VCOM_CNT>=3)
    #define EP8_MAX_PKT_SIZE    8
    #define EP9_MAX_PKT_SIZE    DATA_EP_MAX_PKT_SIZE
    #define EP10_MAX_PKT_SIZE   DATA_EP_MAX_PKT_SIZE
#endif

#if (VCOM_CNT>=4)
    #define EP11_MAX_PKT_SIZE    8
    #define EP12_MAX_PKT_SIZE    DATA_EP_MAX_PKT_SIZE
    #define EP13_MAX_PKT_SIZE    DATA_EP_MAX_PKT_SIZE
#endif

#if (VCOM_CNT>=5)
    #define EP14_MAX_PKT_SIZE    8
    #define EP15_MAX_PKT_SIZE    DATA_EP_MAX_PKT_SIZE
    #define EP16_MAX_PKT_SIZE    DATA_EP_MAX_PKT_SIZE
#endif

#define SETUP_BUF_BASE      0
#define SETUP_BUF_LEN       8
#define EP0_BUF_BASE        (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP0_BUF_LEN         EP0_MAX_PKT_SIZE
#define EP1_BUF_BASE        (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP1_BUF_LEN         EP1_MAX_PKT_SIZE

#if (VCOM_CNT>=1)
    #define EP2_BUF_BASE        (EP1_BUF_BASE + EP1_BUF_LEN)
    #define EP2_BUF_LEN         EP2_MAX_PKT_SIZE
    #define EP3_BUF_BASE        (EP2_BUF_BASE + EP2_BUF_LEN)
    #define EP3_BUF_LEN         EP3_MAX_PKT_SIZE
    #define EP4_BUF_BASE        (EP3_BUF_BASE + EP3_BUF_LEN)
    #define EP4_BUF_LEN         EP4_MAX_PKT_SIZE
#endif

#if (VCOM_CNT>=2)
    #define EP5_BUF_BASE        (EP4_BUF_BASE + EP4_BUF_LEN)
    #define EP5_BUF_LEN         EP5_MAX_PKT_SIZE
    #define EP6_BUF_BASE        (EP5_BUF_BASE + EP5_BUF_LEN)
    #define EP6_BUF_LEN         EP6_MAX_PKT_SIZE
    #define EP7_BUF_BASE        (EP6_BUF_BASE + EP6_BUF_LEN)
    #define EP7_BUF_LEN         EP7_MAX_PKT_SIZE
#endif

#if (VCOM_CNT>=3)
    #define EP8_BUF_BASE        (EP7_BUF_BASE + EP7_BUF_LEN)
    #define EP8_BUF_LEN         EP8_MAX_PKT_SIZE
    #define EP9_BUF_BASE        (EP8_BUF_BASE + EP8_BUF_LEN)
    #define EP9_BUF_LEN         EP9_MAX_PKT_SIZE
    #define EP10_BUF_BASE        (EP9_BUF_BASE + EP9_BUF_LEN)
    #define EP10_BUF_LEN         EP10_MAX_PKT_SIZE
#endif

#if (VCOM_CNT>=4)
    #define EP11_BUF_BASE        (EP10_BUF_BASE + EP10_BUF_LEN)
    #define EP11_BUF_LEN         EP11_MAX_PKT_SIZE
    #define EP12_BUF_BASE        (EP11_BUF_BASE + EP11_BUF_LEN)
    #define EP12_BUF_LEN         EP12_MAX_PKT_SIZE
    #define EP13_BUF_BASE        (EP12_BUF_BASE + EP12_BUF_LEN)
    #define EP13_BUF_LEN         EP13_MAX_PKT_SIZE
#endif

#if (VCOM_CNT>=5)
    #define EP14_BUF_BASE        (EP13_BUF_BASE + EP13_BUF_LEN)
    #define EP14_BUF_LEN         EP14_MAX_PKT_SIZE
    #define EP15_BUF_BASE        (EP14_BUF_BASE + EP14_BUF_LEN)
    #define EP15_BUF_LEN         EP15_MAX_PKT_SIZE
    #define EP16_BUF_BASE        (EP15_BUF_BASE + EP15_BUF_LEN)
    #define EP16_BUF_LEN         EP16_MAX_PKT_SIZE
#endif

/* Define the interrupt In EP number */
#if (VCOM_CNT>=1)
    #define INT_IN_EP_NUM_0     0x02
    #define BULK_IN_EP_NUM_0    0x03
    #define BULK_OUT_EP_NUM_0   0x03
#endif

#if (VCOM_CNT>=2)
    #define INT_IN_EP_NUM_1     0x04
    #define BULK_IN_EP_NUM_1    0x05
    #define BULK_OUT_EP_NUM_1   0x05
#endif
#if (VCOM_CNT>=3)
    #define INT_IN_EP_NUM_2     0x06
    #define BULK_IN_EP_NUM_2    0x07
    #define BULK_OUT_EP_NUM_2   0x07
#endif
#if (VCOM_CNT>=4)
    #define INT_IN_EP_NUM_3     0x08
    #define BULK_IN_EP_NUM_3    0x09
    #define BULK_OUT_EP_NUM_3   0x09
#endif
#if (VCOM_CNT>=5)
    #define INT_IN_EP_NUM_4     0x0A
    #define BULK_IN_EP_NUM_4    0x0B
    #define BULK_OUT_EP_NUM_4   0x0B
#endif

/* Define Descriptor information */
#define USBD_SELF_POWERED               0
#define USBD_REMOTE_WAKEUP              0
#define USBD_MAX_POWER                  50  /* The unit is in 2mA. ex: 50 * 2mA = 100mA */

/*--------------------------------------------------------------------------*/
#define RXBUFSIZE           512 /* RX buffer size */
#define TXBUFSIZE           512 /* TX buffer size */

#define TX_FIFO_SIZE_0      16  /* TX Hardware FIFO size */
#define TX_FIFO_SIZE_1      16  /* TX Hardware FIFO size */
#define TX_FIFO_SIZE_2      16  /* TX Hardware FIFO size */
#define TX_FIFO_SIZE_3      16  /* TX Hardware FIFO size */
#define TX_FIFO_SIZE_4      1  /* TX Hardware FIFO size */

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

typedef struct
{
    UART_T *const UART_BASE;
    const IRQn_Type eIRQn;
    const uint32_t u32TX_FIFO_SIZE;
} VCOM_CONTROL_CONST_t;

typedef struct
{
    volatile uint8_t au8ComRbuf[RXBUFSIZE];
    volatile uint16_t u16ComRbytes;
    volatile uint16_t u16ComRhead;
    volatile uint16_t u16ComRtail;

    volatile uint8_t au8ComTbuf[TXBUFSIZE];
    volatile uint16_t u16ComTbytes;
    volatile uint16_t u16ComThead;
    volatile uint16_t u16ComTtail;

    uint8_t au8RxBuf[DATA_EP_MAX_PKT_SIZE]; /* Bulk IN Buffer */
    volatile uint8_t *pu8RxBuf;
    volatile uint32_t u32RxSize;
    volatile uint32_t u32TxSize;

    volatile int8_t i8BulkOutReady;

    STR_VCOM_LINE_CODING sLineCoding; /* Baud rate: 115200, Stop bit, parity, data bits    */

    uint16_t u16CtrlSignal;           /* BIT0     : DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

    uint32_t volatile u32OutToggle;

} VCOM_CONTROL_VAR_t;

typedef struct
{
    VCOM_CONTROL_VAR_t       *psCtrlVar;
    VCOM_CONTROL_CONST_t  *psPeripheralInfo;
} VCOM_CONTROL_BLOCK_t;

#define GET_BULK_IN_EP(i) ((i+1)*3)
#define GET_BULK_OUT_EP(i) ((i+1)*3+1)

/*-------------------------------------------------------------*/

extern VCOM_CONTROL_BLOCK_t tVCOM[];

/*-------------------------------------------------------------*/
void VCOM_Init(void);
void VCOM_ClassRequest(void);
void VCOM_LineCoding(uint8_t port);
void VCOM_TransferData(void);

#if (VCOM_CNT>=1)
    void EP3_Handler(void);
    void EP4_Handler(void);
#endif

#if (VCOM_CNT>=2)
    void EP6_Handler(void);
    void EP7_Handler(void);
#endif

#if (VCOM_CNT>=3)
    void EP9_Handler(void);
    void EP10_Handler(void);
#endif

#if (VCOM_CNT>=4)
    void EP12_Handler(void);
    void EP13_Handler(void);
#endif

#if (VCOM_CNT>=5)
    void EP15_Handler(void);
    void EP16_Handler(void);
#endif

extern uint8_t volatile g_u8Suspend;

#endif  /* __USBD_VCOM_MULTI_PORT_H__ */
