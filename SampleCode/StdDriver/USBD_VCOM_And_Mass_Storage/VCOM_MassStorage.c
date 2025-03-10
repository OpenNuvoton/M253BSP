/****************************************************************************//**
 * @file     VCOM_MassStorage.c
 * @version  V0.10
 * @brief    M253 series USBD VCOM and Mass-storage sample file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "cdc_serial.h"
#include "massstorage.h"

#if 0
    #define DBG_PRINTF      printf
#else
    #define DBG_PRINTF(...)
#endif

/*--------------------------------------------------------------------------*/
/* Global variables for Control Pipe */
int32_t g_TotalSectors = 0;

uint8_t volatile g_u8EP7Ready   = 0;
uint8_t volatile g_u8EP6Ready   = 0;
uint8_t volatile g_u8Remove     = 0;
uint8_t volatile g_u8Suspend    = 0;

uint32_t volatile g_u32OutToggle0   = 0;
uint32_t volatile g_u32OutToggle    = 0, g_u32OutSkip = 0;
uint32_t volatile g_u32CbwStall     = 1;

/* USB flow control variables */
uint8_t g_u8BulkState;
uint8_t g_u8Prevent = 0;
uint8_t g_u8Size;

uint8_t g_au8SenseKey[4];

uint32_t g_u32DataFlashStartAddr;
uint32_t g_u32Address;
uint32_t g_u32Length;
uint32_t g_u32LbaAddress;
uint32_t g_u32BytesInStorageBuf;

uint32_t g_u32BulkBuf0, g_u32BulkBuf1;

/* CBW/CSW variables */
struct CBW g_sCBW;
struct CSW g_sCSW;

uint32_t g_au32MassBlock[MASS_BUFFER_SIZE / 4];
uint32_t g_au32StorageBlock[STORAGE_BUFFER_SIZE / 4];

/*--------------------------------------------------------------------------*/
uint8_t g_au8InquiryID[36] =
{
    0x00,                   /* Peripheral Device Type */
    0x80,                   /* RMB */
    0x00,                   /* ISO/ECMA, ANSI Version */
    0x00,                   /* Response Data Format */
    0x1F, 0x00, 0x00, 0x00, /* Additional Length */

    /* Vendor Identification */
    'N', 'u', 'v', 'o', 't', 'o', 'n', ' ',

    /* Product Identification */
    'U', 'S', 'B', ' ', 'M', 'a', 's', 's', ' ', 'S', 't', 'o', 'r', 'a', 'g', 'e',

    /* Product Revision */
    '1', '.', '0', '0'
};

// code = 5Ah, Mode Sense
static uint8_t g_au8ModePage_01[12] =
{
    0x01, 0x0A, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,
    0x03, 0x00, 0x00, 0x00
};

static uint8_t g_au8ModePage_05[32] =
{
    0x05, 0x1E, 0x13, 0x88, 0x08, 0x20, 0x02, 0x00,
    0x01, 0xF4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x05, 0x1E, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x68, 0x00, 0x00
};

static uint8_t g_au8ModePage_1B[12] =
{
    0x1B, 0x0A, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
};

static uint8_t g_au8ModePage_1C[8] =
{
    0x1C, 0x06, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00
};


void USBD_IRQHandler(void)
{
    uint32_t u32IntSts = USBD_GET_INT_FLAG();
    uint32_t u32State = USBD_GET_BUS_STATE();


    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_FLDET)
    {
        // Floating detect
        USBD_CLR_INT_FLAG(USBD_INTSTS_FLDET);

        if (USBD_IS_ATTACHED())
        {
            /* USB Plug In */
            USBD_ENABLE_USB();
        }
        else
        {
            /* USB Un-plug */
            USBD_DISABLE_USB();
        }
    }

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_BUS)
    {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_BUS);

        if (u32State & USBD_STATE_USBRST)
        {
            /* Bus reset */
            USBD_ENABLE_USB();
            USBD_SwReset();
            g_u32OutToggle0 = 0;
            g_u32OutToggle  = 0;
            g_u32OutSkip    = 0;
            g_u8Remove      = 0;
            g_u8Suspend     = 0;

            DBG_PRINTF("Bus reset\n");
        }

        if (u32State & USBD_STATE_SUSPEND)
        {
            /* Enter power down to wait USB attached */
            g_u8Suspend = 1;

            /* Enable USB but disable PHY */
            USBD_DISABLE_PHY();
            DBG_PRINTF("Suspend\n");
        }

        if (u32State & USBD_STATE_RESUME)
        {
            /* Enable USB and enable PHY */
            USBD_ENABLE_USB();
            DBG_PRINTF("Resume\n");

            g_u8Suspend = 0;
        }

    }

    if (u32IntSts & USBD_INTSTS_NEVWKIF_Msk)
    {
        /*Clear no-event wake up interrupt */
        USBD_CLR_INT_FLAG(USBD_INTSTS_NEVWKIF_Msk);
        /*
           TODO: Implement the function that will be executed when device is woken by non-USB event.
        */

    }

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_USB)
    {
        extern uint8_t g_USBD_au8SetupPacket[];

        // USB event
        if (u32IntSts & USBD_INTSTS_SETUP)
        {
            // Setup packet
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SETUP);

            /* Clear the data IN/OUT ready flag of control end-points */
            USBD_STOP_TRANSACTION(EP0);
            USBD_STOP_TRANSACTION(EP1);

            USBD_ProcessSetupPacket();
        }

        // EP events
        if (u32IntSts & USBD_INTSTS_EP0)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP0);
            // control IN
            USBD_CtrlIn();
        }

        if (u32IntSts & USBD_INTSTS_EP1)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP1);

            // control OUT
            USBD_CtrlOut();

            // In ACK of SET_LINE_CODE
            if (g_USBD_au8SetupPacket[1] == SET_LINE_CODE)
            {
                if (g_USBD_au8SetupPacket[4] == 0) /* VCOM-1 */
                    VCOM_LineCoding(); /* Apply UART settings */
            }
        }

        if (u32IntSts & USBD_INTSTS_EP2)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP2);
            // Bulk IN
            EP2_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP3)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP3);
            // Bulk OUT
            EP3_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP4)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP4);
        }

        if (u32IntSts & USBD_INTSTS_EP5)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP5);
            // Bulk IN
            //EP5_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP6)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP6);
            // Bulk OUT
            EP6_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP7)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP7);
            // Bulk IN
            EP7_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP8)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP8);
        }

        if (u32IntSts & USBD_INTSTS_EP9)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP9);
        }

        if (u32IntSts & USBD_INTSTS_EP10)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP10);
        }

        if (u32IntSts & USBD_INTSTS_EP11)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP11);
        }
    }

}


void EP2_Handler(void)
{
    g_u32TxSize = 0;
}


void EP3_Handler(void)
{
    /* Bulk OUT */
    if (g_u32OutToggle0 == (USBD->EPSTS0 & USBD_EPSTS0_EPSTS3_Msk))
    {
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }
    else
    {
        g_u32RxSize = USBD_GET_PAYLOAD_LEN(EP3);
        g_pu8RxBuf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP3));

        g_u32OutToggle0 = USBD->EPSTS0 & USBD_EPSTS0_EPSTS3_Msk;
        /* Set a flag to indicate bulk out ready */
        g_i8BulkOutReady = 1;
    }
}

void EP7_Handler(void)
{
    g_u8EP7Ready = 1;
    MSC_AckCmd();
}


void EP6_Handler(void)
{
    /* Bulk OUT */
    if ((g_u32OutToggle == (USBD->EPSTS0 & USBD_EPSTS0_EPSTS6_Msk)) && !g_u32CbwStall)
    {
        g_u32OutSkip = 1;
        USBD_SET_PAYLOAD_LEN(EP6, EP6_MAX_PKT_SIZE);
    }
    else
    {
        g_u8EP6Ready    = 1;
        g_u32OutToggle  = USBD->EPSTS0 & USBD_EPSTS0_EPSTS6_Msk;
        g_u32OutSkip    = 0;

        g_u32CbwStall   = 0;
    }
}


void VCOM_MSC_Init(void)
{
    /* Init setup packet buffer */
    /* Buffer range for SETUP packet -> [0 ~ 0x7] */
    USBD->STBUFSEG = SETUP_BUF_BASE;

    /*****************************************************/
    /* EP0 ==> control IN endpoint, address 0 */
    USBD_CONFIG_EP(EP0, USBD_CFG_CSTALL | USBD_CFG_EPMODE_IN | 0);
    /* Buffer range for EP0 */
    USBD_SET_EP_BUF_ADDR(EP0, EP0_BUF_BASE);

    /* EP1 ==> control OUT endpoint, address 0 */
    USBD_CONFIG_EP(EP1, USBD_CFG_CSTALL | USBD_CFG_EPMODE_OUT | 0);
    /* Buffer range for EP1 */
    USBD_SET_EP_BUF_ADDR(EP1, EP1_BUF_BASE);

    /*****************************************************/
    /* EP2 ==> Bulk IN endpoint, address 1 */
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM);
    /* Buffer range for EP2 */
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);

    /* EP3 ==> Bulk Out endpoint, address 2 */
    USBD_CONFIG_EP(EP3, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM);
    /* Buffer range for EP3 */
    USBD_SET_EP_BUF_ADDR(EP3, EP3_BUF_BASE);
    /* trigger to receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);

    /* EP4 ==> Interrupt IN endpoint, address 3 */
    USBD_CONFIG_EP(EP4, USBD_CFG_EPMODE_IN | INT_IN_EP_NUM);
    /* Buffer range for EP4 ->  */
    USBD_SET_EP_BUF_ADDR(EP4, EP4_BUF_BASE);

    /*****************************************************/
    /* EP7 ==> Bulk IN endpoint, address 6 */
    USBD_CONFIG_EP(EP7, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM_1);
    /* Buffer range for EP7 */
    USBD_SET_EP_BUF_ADDR(EP7, EP7_BUF_BASE);

    /* EP6 ==> Bulk Out endpoint, address 5 */
    USBD_CONFIG_EP(EP6, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM_1);
    /* Buffer range for EP6 */
    USBD_SET_EP_BUF_ADDR(EP6, EP6_BUF_BASE);
    /* trigger to receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP6, EP6_MAX_PKT_SIZE);

    /*****************************************************/
    g_u32BulkBuf0 = EP6_BUF_BASE;
    g_u32BulkBuf1 = EP7_BUF_BASE;

    g_sCSW.dCSWSignature = CSW_SIGNATURE;
    g_TotalSectors = DATA_FLASH_STORAGE_SIZE / UDC_SECTOR_SIZE;

}

void VCOM_MSC_ClassRequest(void)
{
    uint8_t buf[8];

    USBD_GetSetupPacket(buf);

    if (buf[0] & EP_INPUT)   /* request data transfer direction */
    {
        // Device to host
        switch (buf[1])
        {
            case GET_LINE_CODE:
            {
                if (buf[4] == 0)   /* VCOM-1 */
                {
                    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)), (uint8_t *)&g_sLineCoding, 7);
                }

                /* Data stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 7);
                /* Status stage */
                USBD_PrepareCtrlOut(0, 0);
                break;
            }

            case GET_MAX_LUN:
            {
                /* Check interface number with cfg descriptor and check wValue = 0, wLength = 1 */
                if ((buf[4] == gsInfo.gu8ConfigDesc[LEN_CONFIG + 2]) && (buf[2] + buf[3] + buf[6] + buf[7] == 1))
                {
                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = 0;
                    /* Data stage */
                    USBD_SET_DATA1(EP0);
                    USBD_SET_PAYLOAD_LEN(EP0, 1);
                    /* Status stage */
                    USBD_PrepareCtrlOut(0, 0);
                }
                else
                {
                    USBD_SET_EP_STALL(EP1); // Stall when wrong parameter
                }

                g_u32OutToggle = 0;
                USBD_SET_DATA0(EP7);

                break;
            }

            default:
            {
                /* Setup error, stall the device */
                USBD_SetStall(0);
                DBG_PRINTF("Unknow MSC req(0x%x). stall ctrl pipe\n", buf[1]);
                break;
            }
        }
    }
    else
    {
        // Host to device
        switch (buf[1])
        {
            case SET_CONTROL_LINE_STATE:
            {
                if (buf[4] == 0)   /* VCOM-1 */
                {
                    g_u16CtrlSignal = buf[3];
                    g_u16CtrlSignal = (g_u16CtrlSignal << 8) | buf[2];
                    //printf("RTS=%d  DTR=%d\n", (gCtrlSignal0 >> 1) & 1, gCtrlSignal0 & 1);
                }

                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                break;
            }

            case SET_LINE_CODE:
            {
                if (buf[4] == 0) /* VCOM-1 */
                    USBD_PrepareCtrlOut((uint8_t *)&g_sLineCoding, 7);

                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);

                break;
            }

            case BULK_ONLY_MASS_STORAGE_RESET:
            {
                /* Check interface number with cfg descriptor and check wValue = 0, wLength = 0 */
                //if ((buf[4] == gsInfo.gu8ConfigDesc[LEN_CONFIG + 2]) && (buf[2] + buf[3] + buf[6] + buf[7] == 0))
                if ((buf[4] == 0x2) && (buf[2] + buf[3] + buf[6] + buf[7] == 0))
                {

                    g_u32Length = 0; // Reset all read/write data transfer
                    USBD_LockEpStall(0);

                    /* Clear ready */
                    USBD->EP[EP7].CFGP |= USBD_CFGP_CLRRDY_Msk;
                    USBD->EP[EP6].CFGP |= USBD_CFGP_CLRRDY_Msk;

                    /* Prepare to receive the CBW */

                    g_u8EP6Ready = 0;
                    g_u8BulkState = BULK_CBW;

                    USBD_SET_DATA1(EP6);
                    USBD_SET_EP_BUF_ADDR(EP6, g_u32BulkBuf0);
                    USBD_SET_PAYLOAD_LEN(EP6, 31);

                    /* Status stage */
                    USBD_SET_DATA1(EP0);
                    USBD_SET_PAYLOAD_LEN(EP0, 0);

                }
                else
                {
                    /* Stall when wrong parameter */
                    USBD_SET_EP_STALL(EP0);
                    USBD_SET_EP_STALL(EP1);
                }

                break;
            }

            default:
            {
                // Stall
                /* Setup error, stall the device */
                USBD_SetStall(0);
                DBG_PRINTF("Unknow MSC req (0x%x). stall ctrl pipe\n", buf[1]);
                break;
            }
        }
    }
}

void VCOM_LineCoding(void)
{
    uint32_t u32Reg;
    uint32_t u32Baud_Div;

    NVIC_DisableIRQ(UART4_IRQn);
    // Reset software FIFO
    g_u16ComRbytes = 0;
    g_u16ComRhead = 0;
    g_u16ComRtail = 0;

    g_u16ComTbytes = 0;
    g_u16ComThead = 0;
    g_u16ComTtail = 0;

    // Reset hardware FIFO
    UART4->FIFO = UART_FIFO_RXRST_Msk | UART_FIFO_TXRST_Msk;

    // Set baudrate
    u32Baud_Div = UART_BAUD_MODE2_DIVIDER(__HIRC, g_sLineCoding.u32DTERate);

    if (u32Baud_Div > 0xFFFF)
        UART4->BAUD = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(__HIRC, g_sLineCoding.u32DTERate));
    else
        UART4->BAUD = (UART_BAUD_MODE2 | u32Baud_Div);

    // Set parity
    if (g_sLineCoding.u8ParityType == 0)
        u32Reg = 0; // none parity
    else if (g_sLineCoding.u8ParityType == 1)
        u32Reg = 0x08; // odd parity
    else if (g_sLineCoding.u8ParityType == 2)
        u32Reg = 0x18; // even parity
    else
        u32Reg = 0;

    // bit width
    switch (g_sLineCoding.u8DataBits)
    {
        case 5:
            u32Reg |= 0;
            break;

        case 6:
            u32Reg |= 1;
            break;

        case 7:
            u32Reg |= 2;
            break;

        case 8:
            u32Reg |= 3;
            break;

        default:
            break;
    }

    // stop bit
    if (g_sLineCoding.u8CharFormat > 0)
        u32Reg |= 0x4; // 2 or 1.5 bits

    UART4->LINE = u32Reg;

    // Re-enable UART interrupt
    NVIC_EnableIRQ(UART4_IRQn);
}

void MSC_RequestSense(void)
{
    uint8_t tmp[20];

    memset(tmp, 0, 18);

    if (g_u8Prevent)
    {
        g_u8Prevent = 0;
        tmp[0] = 0x70;
    }
    else
        tmp[0] = 0xf0;

    tmp[2] = g_au8SenseKey[0];
    tmp[7] = 0x0a;
    tmp[12] = g_au8SenseKey[1];
    tmp[13] = g_au8SenseKey[2];
    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP7)), tmp, 20);

    g_au8SenseKey[0] = 0;
    g_au8SenseKey[1] = 0;
    g_au8SenseKey[2] = 0;
}

void MSC_ReadFormatCapacity(void)
{
    uint8_t *pu8Desc;

    pu8Desc = (uint8_t *)MassCMD_BUF;
    memset(pu8Desc, 0, 36);

    /*---------- Capacity List Header ----------*/
    // Capacity List Length
    pu8Desc[3] = 0x10;

    /*---------- Current/Maximum Capacity Descriptor ----------*/
    // Number of blocks (MSB first)
    pu8Desc[4] = GET_BYTE3(g_TotalSectors);
    pu8Desc[5] = GET_BYTE2(g_TotalSectors);
    pu8Desc[6] = GET_BYTE1(g_TotalSectors);
    pu8Desc[7] = GET_BYTE0(g_TotalSectors);

    // Descriptor Code:
    // 01b = Unformatted Media - Maximum formattable capacity for this cartridge
    // 10b = Formatted Media - Current media capacity
    // 11b = No Cartridge in Drive - Maximum formattable capacity for any cartridge
    pu8Desc[8] = 0x02;


    // Block Length. Fixed to be 512 (MSB first)
    pu8Desc[ 9] = GET_BYTE2(512);
    pu8Desc[10] = GET_BYTE1(512);
    pu8Desc[11] = GET_BYTE0(512);

    /*---------- Formattable Capacity Descriptor ----------*/
    // Number of Blocks
    pu8Desc[12] = GET_BYTE3(g_TotalSectors);
    pu8Desc[13] = GET_BYTE2(g_TotalSectors);
    pu8Desc[14] = GET_BYTE1(g_TotalSectors);
    pu8Desc[15] = GET_BYTE0(g_TotalSectors);

    // Block Length. Fixed to be 512 (MSB first)
    pu8Desc[17] = GET_BYTE2(512);
    pu8Desc[18] = GET_BYTE1(512);
    pu8Desc[19] = GET_BYTE0(512);

}

void MSC_Read(void)
{
    if (USBD_GET_EP_BUF_ADDR(EP7) == g_u32BulkBuf1)
        USBD_SET_EP_BUF_ADDR(EP7, g_u32BulkBuf0);
    else
        USBD_SET_EP_BUF_ADDR(EP7, g_u32BulkBuf1);

    /* Trigger to send out the data packet */
    USBD_SET_PAYLOAD_LEN(EP7, g_u8Size);

    g_u32Length -= g_u8Size;
    g_u32BytesInStorageBuf -= g_u8Size;

    if (g_u32Length)
    {
        if (g_u32BytesInStorageBuf)
        {
            /* Prepare next data packet */
            g_u8Size = EP7_MAX_PKT_SIZE;

            if (g_u8Size > g_u32Length)
                g_u8Size = g_u32Length;

            if (USBD_GET_EP_BUF_ADDR(EP7) == g_u32BulkBuf1)
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf0), (uint8_t *)g_u32Address, g_u8Size);
            else
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_u32Address, g_u8Size);

            g_u32Address += g_u8Size;
        }
        else
        {
            uint32_t u32Len = g_u32Length;

            if (u32Len > STORAGE_BUFFER_SIZE)
                u32Len = STORAGE_BUFFER_SIZE;

            MSC_ReadMedia(g_u32LbaAddress, u32Len, (uint8_t *)STORAGE_DATA_BUF);
            g_u32BytesInStorageBuf = u32Len;
            g_u32LbaAddress += u32Len;
            g_u32Address = STORAGE_DATA_BUF;

            /* Prepare next data packet */
            g_u8Size = EP7_MAX_PKT_SIZE;

            if (g_u8Size > g_u32Length)
                g_u8Size = g_u32Length;

            if (USBD_GET_EP_BUF_ADDR(EP7) == g_u32BulkBuf1)
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf0), (uint8_t *)g_u32Address, g_u8Size);
            else
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_u32Address, g_u8Size);

            g_u32Address += g_u8Size;
        }
    }
}

void MSC_ReadTrig(void)
{
    if (g_u32Length)
    {
        if (g_u32BytesInStorageBuf)
        {
            /* Prepare next data packet */
            g_u8Size = EP7_MAX_PKT_SIZE;

            if (g_u8Size > g_u32Length)
                g_u8Size = g_u32Length;

            if (USBD_GET_EP_BUF_ADDR(EP7) == g_u32BulkBuf1)
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf0), (uint8_t *)g_u32Address, g_u8Size);
            else
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_u32Address, g_u8Size);

            g_u32Address += g_u8Size;
        }
        else
        {
            uint32_t u32Len = g_u32Length;

            if (u32Len > STORAGE_BUFFER_SIZE)
                u32Len = STORAGE_BUFFER_SIZE;

            MSC_ReadMedia(g_u32LbaAddress, u32Len, (uint8_t *)STORAGE_DATA_BUF);
            g_u32BytesInStorageBuf = u32Len;
            g_u32LbaAddress += u32Len;
            g_u32Address = STORAGE_DATA_BUF;

            /* Prepare next data packet */
            g_u8Size = EP7_MAX_PKT_SIZE;

            if (g_u8Size > g_u32Length)
                g_u8Size = g_u32Length;

            if (USBD_GET_EP_BUF_ADDR(EP7) == g_u32BulkBuf1)
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf0), (uint8_t *)g_u32Address, g_u8Size);
            else
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_u32Address, g_u8Size);

            g_u32Address += g_u8Size;
        }

        /* DATA0/DATA1 Toggle */
        if (USBD_GET_EP_BUF_ADDR(EP7) == g_u32BulkBuf1)
            USBD_SET_EP_BUF_ADDR(EP7, g_u32BulkBuf0);
        else
            USBD_SET_EP_BUF_ADDR(EP7, g_u32BulkBuf1);

        /* Trigger to send out the data packet */
        USBD_SET_PAYLOAD_LEN(EP7, g_u8Size);

        g_u32Length -= g_u8Size;
        g_u32BytesInStorageBuf -= g_u8Size;

    }
    else
        USBD_SET_PAYLOAD_LEN(EP7, 0);
}


void MSC_ReadCapacity(void)
{
    uint32_t tmp;

    memset((uint8_t *)MassCMD_BUF, 0, 36);

    tmp = g_TotalSectors - 1;
    *((uint8_t *)(MassCMD_BUF + 0)) = *((uint8_t *)&tmp + 3);
    *((uint8_t *)(MassCMD_BUF + 1)) = *((uint8_t *)&tmp + 2);
    *((uint8_t *)(MassCMD_BUF + 2)) = *((uint8_t *)&tmp + 1);
    *((uint8_t *)(MassCMD_BUF + 3)) = *((uint8_t *)&tmp + 0);
    *((uint8_t *)(MassCMD_BUF + 6)) = 0x02;
}

void MSC_ReadCapacity16(void)
{
    uint32_t tmp;

    memset((uint8_t *)MassCMD_BUF, 0, 36);

    tmp = g_TotalSectors - 1;
    *((uint8_t *)(MassCMD_BUF + 0)) = 0;
    *((uint8_t *)(MassCMD_BUF + 1)) = 0;
    *((uint8_t *)(MassCMD_BUF + 2)) = 0;
    *((uint8_t *)(MassCMD_BUF + 3)) = 0;
    *((uint8_t *)(MassCMD_BUF + 4)) = *((uint8_t *)&tmp + 3);
    *((uint8_t *)(MassCMD_BUF + 5)) = *((uint8_t *)&tmp + 2);
    *((uint8_t *)(MassCMD_BUF + 6)) = *((uint8_t *)&tmp + 1);
    *((uint8_t *)(MassCMD_BUF + 7)) = *((uint8_t *)&tmp + 0);
    *((uint8_t *)(MassCMD_BUF + 10)) = 0x02;
}


void MSC_ModeSense10(void)
{
    uint8_t i, j;
    uint8_t NumHead, NumSector;
    uint16_t NumCyl = 0;

    /* Clear the command buffer */
    *((uint32_t *)MassCMD_BUF) = 0;
    *((uint32_t *)MassCMD_BUF + 1) = 0;

    switch (g_sCBW.au8Data[0])
    {
        case 0x01:
            *((uint8_t *)MassCMD_BUF) = 19;
            i = 8;

            for (j = 0; j < 12; j++, i++)
                *((uint8_t *)(MassCMD_BUF + i)) = g_au8ModePage_01[j];

            break;

        case 0x05:
            *((uint8_t *)MassCMD_BUF) = 39;
            i = 8;

            for (j = 0; j < 32; j++, i++)
                *((uint8_t *)(MassCMD_BUF + i)) = g_au8ModePage_05[j];

            NumHead = 2;
            NumSector = 64;
            NumCyl = g_TotalSectors / 128;

            *((uint8_t *)(MassCMD_BUF + 12)) = NumHead;
            *((uint8_t *)(MassCMD_BUF + 13)) = NumSector;
            *((uint8_t *)(MassCMD_BUF + 16)) = (uint8_t)(NumCyl >> 8);
            *((uint8_t *)(MassCMD_BUF + 17)) = (uint8_t)(NumCyl & 0x00ff);
            break;

        case 0x1B:
            *((uint8_t *)MassCMD_BUF) = 19;
            i = 8;

            for (j = 0; j < 12; j++, i++)
                *((uint8_t *)(MassCMD_BUF + i)) = g_au8ModePage_1B[j];

            break;

        case 0x1C:
            *((uint8_t *)MassCMD_BUF) = 15;
            i = 8;

            for (j = 0; j < 8; j++, i++)
                *((uint8_t *)(MassCMD_BUF + i)) = g_au8ModePage_1C[j];

            break;

        case 0x3F:
            *((uint8_t *)MassCMD_BUF) = 0x47;
            i = 8;

            for (j = 0; j < 12; j++, i++)
                *((uint8_t *)(MassCMD_BUF + i)) = g_au8ModePage_01[j];

            for (j = 0; j < 32; j++, i++)
                *((uint8_t *)(MassCMD_BUF + i)) = g_au8ModePage_05[j];

            for (j = 0; j < 12; j++, i++)
                *((uint8_t *)(MassCMD_BUF + i)) = g_au8ModePage_1B[j];

            for (j = 0; j < 8; j++, i++)
                *((uint8_t *)(MassCMD_BUF + i)) = g_au8ModePage_1C[j];

            NumHead = 2;
            NumSector = 64;
            NumCyl = g_TotalSectors / 128;

            *((uint8_t *)(MassCMD_BUF + 24)) = NumHead;
            *((uint8_t *)(MassCMD_BUF + 25)) = NumSector;
            *((uint8_t *)(MassCMD_BUF + 28)) = (uint8_t)(NumCyl >> 8);
            *((uint8_t *)(MassCMD_BUF + 29)) = (uint8_t)(NumCyl & 0x00ff);
            break;

        default:
            g_au8SenseKey[0] = 0x05;
            g_au8SenseKey[1] = 0x24;
            g_au8SenseKey[2] = 0x00;
    }
}

void MSC_Write(void)
{

    if (g_u32OutSkip == 0)
    {
        if (g_u32Length > EP6_MAX_PKT_SIZE)
        {
            if (USBD_GET_EP_BUF_ADDR(EP6) == g_u32BulkBuf0)
            {
                USBD_SET_EP_BUF_ADDR(EP6, g_u32BulkBuf1);
                USBD_SET_PAYLOAD_LEN(EP6, EP6_MAX_PKT_SIZE);
                USBD_MemCopy((uint8_t *)g_u32Address, (uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf0), EP6_MAX_PKT_SIZE);
            }
            else
            {
                USBD_SET_EP_BUF_ADDR(EP6, g_u32BulkBuf0);
                USBD_SET_PAYLOAD_LEN(EP6, EP6_MAX_PKT_SIZE);
                USBD_MemCopy((uint8_t *)g_u32Address, (uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), EP6_MAX_PKT_SIZE);
            }

            g_u32Address += EP6_MAX_PKT_SIZE;
            g_u32Length -= EP6_MAX_PKT_SIZE;

            /* Buffer full. Writer it to storage first. */
            if (g_u32Address >= (STORAGE_DATA_BUF + STORAGE_BUFFER_SIZE))
            {
                DataFlashWrite(g_u32DataFlashStartAddr, STORAGE_BUFFER_SIZE, (uint32_t)STORAGE_DATA_BUF);

                g_u32Address = STORAGE_DATA_BUF;
                g_u32DataFlashStartAddr += STORAGE_BUFFER_SIZE;
            }
        }
        else
        {
            if (USBD_GET_EP_BUF_ADDR(EP6) == g_u32BulkBuf0)
                USBD_MemCopy((uint8_t *)g_u32Address, (uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf0), g_u32Length);
            else
                USBD_MemCopy((uint8_t *)g_u32Address, (uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), g_u32Length);

            g_u32Address += g_u32Length;
            g_u32Length = 0;


            if ((g_sCBW.u8OPCode == UFI_WRITE_10) || (g_sCBW.u8OPCode == UFI_WRITE_12))
            {
                uint32_t lba = get_be32(&g_sCBW.au8Data[0]);

                uint32_t len = lba * UDC_SECTOR_SIZE + g_sCBW.dCBWDataTransferLength - g_u32DataFlashStartAddr;

                if (len)
                {
                    DataFlashWrite(g_u32DataFlashStartAddr, len, (uint32_t)STORAGE_DATA_BUF);
                }
            }

            g_u8BulkState = BULK_IN;
            MSC_AckCmd();
        }
    }
}

void MSC_ProcessCmd(void)
{
    if (g_u8EP6Ready)
    {
        g_u8EP6Ready = 0;

        if (g_u8BulkState == BULK_CBW)
        {
            uint32_t Hcount, Dcount;
            int32_t i;
            uint8_t u8Len = USBD_GET_PAYLOAD_LEN(EP6);

            if (u8Len > 31) u8Len = 31;

            /* Check Signature & length of CBW */
            /* Bulk Out buffer */
            if ((*(uint32_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf0) != CBW_SIGNATURE)/* || (u8Len != 31)*/)
            {

                USBD_SET_EP_STALL(EP7);
                USBD_SET_EP_STALL(EP6);
                USBD_LockEpStall((1 << EP6) | (1 << EP7));
                g_u32CbwStall = 1;
                g_u8BulkState = BULK_CBW;
                DBG_PRINTF("CBW signature fail. stall bulk out pipe\n");
                return;

            }

            /* Get the CBW */
            for (i = 0; i < u8Len; i++)
                *((uint8_t *)(&g_sCBW.dCBWSignature) + i) = *(uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf0 + i);


            /* Prepare to echo the tag from CBW to CSW */
            g_sCSW.dCSWTag = g_sCBW.dCBWTag;
            Hcount = g_sCBW.dCBWDataTransferLength;

            /* Parse Op-Code of CBW */
            switch (g_sCBW.u8OPCode)
            {
                case UFI_PREVENT_ALLOW_MEDIUM_REMOVAL:
                {
                    if (g_sCBW.au8Data[2] & 0x01)
                    {
                        g_au8SenseKey[0] = 0x05;  //INVALID COMMAND
                        g_au8SenseKey[1] = 0x24;
                        g_au8SenseKey[2] = 0;
                        g_u8Prevent = 1;
                    }
                    else
                        g_u8Prevent = 0;

                    g_u8BulkState = BULK_IN;
                    MSC_AckCmd();
                    return;
                }


                case UFI_START_STOP:
                {
                    if ((g_sCBW.au8Data[2] & 0x03) == 0x2)
                    {
                        g_u8Remove = 1;
                    }

                    g_u8BulkState = BULK_IN;
                    MSC_AckCmd();
                    return;
                }

                case UFI_VERIFY_10:
                case UFI_TEST_UNIT_READY:
                {
                    DBG_PRINTF("Test Unit\n");

                    if (Hcount != 0)
                    {
                        if (g_sCBW.bmCBWFlags == 0)     /* Ho > Dn (Case 9) */
                        {
                            g_u8Prevent = 1;
                            g_u32CbwStall = 1;
                            USBD_SET_EP_STALL(EP6);
                            g_sCSW.bCSWStatus = 0x1;
                            g_sCSW.dCSWDataResidue = Hcount;
                        }
                    }
                    else     /* Hn == Dn (Case 1) */
                    {
                        if (g_u8Remove)
                        {
                            g_sCSW.dCSWDataResidue = 0;
                            g_sCSW.bCSWStatus = 1;
                            g_au8SenseKey[0] = 0x02;    /* Not ready */
                            g_au8SenseKey[1] = 0x3A;
                            g_au8SenseKey[2] = 0;
                            g_u8Prevent = 1;
                        }
                        else
                        {
                            g_sCSW.dCSWDataResidue = 0;
                            g_sCSW.bCSWStatus = 0;
                        }
                    }

                    g_u8BulkState = BULK_IN;
                    MSC_AckCmd();
                    return;
                }

                case UFI_REQUEST_SENSE:
                {
                    uint32_t u32Len;

                    u32Len = g_sCBW.dCBWDataTransferLength;

                    if (u32Len > 18) u32Len = 18;

                    if (u32Len)
                    {
                        if (g_sCBW.dCBWDataTransferLength > u32Len)
                        {
                            /* Expecting a STALL after data phase completes with a zero-length or short packet */
                            USBD_SET_EP_STALL(EP6);
                            USBD_SET_EP_STALL(EP7);
                            USBD_LockEpStall((1 << EP6) | (1 << EP7));
                            return;
                        }

                        MSC_RequestSense();
                        g_u8BulkState = BULK_IN;
                        USBD_SET_PAYLOAD_LEN(EP7, u32Len);
                    }
                    else
                    {
                        /* Just skip data phase if zero data transfer length */
                        g_u8BulkState = BULK_IN;
                        MSC_AckCmd();
                    }

                    return;
                }

                case UFI_READ_FORMAT_CAPACITY:
                {
                    g_u32Length = g_sCBW.dCBWDataTransferLength;

                    /* format capacity descriptor length is fixed to be 12 bytes */
                    if (g_u32Length > 20) g_u32Length = 20;

                    g_u32Address = MassCMD_BUF;
                    MSC_ReadFormatCapacity();
                    g_u8BulkState = BULK_IN;

                    if (g_u32Length > 0)
                    {
                        if (g_u32Length > EP7_MAX_PKT_SIZE)
                            g_u8Size = EP7_MAX_PKT_SIZE;
                        else
                            g_u8Size = g_u32Length;

                        /* Bulk IN buffer */
                        USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_u32Address, g_u8Size);
                        g_u32BytesInStorageBuf = g_u8Size;

                        g_u32Address += g_u8Size;
                        USBD_SET_EP_BUF_ADDR(EP7, g_u32BulkBuf0);
                        MSC_Read();
                    }

                    return;
                }

                case UFI_READ_CAPACITY:
                case UFI_READ_CAPACITY_16:
                {
                    g_u32Length = g_sCBW.dCBWDataTransferLength;

                    if (g_u32Length > 36) g_u32Length = 36;

                    g_u32Address = MassCMD_BUF;

                    if (g_sCBW.u8OPCode == UFI_READ_CAPACITY)
                        MSC_ReadCapacity();
                    else
                        MSC_ReadCapacity16();

                    g_u8BulkState = BULK_IN;

                    if (g_u32Length > 0)
                    {
                        if (g_u32Length > EP7_MAX_PKT_SIZE)
                            g_u8Size = EP7_MAX_PKT_SIZE;
                        else
                            g_u8Size = g_u32Length;

                        /* Bulk IN buffer */
                        USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_u32Address, g_u8Size);
                        g_u32BytesInStorageBuf = g_u8Size;

                        g_u32Address += g_u8Size;
                        USBD_SET_EP_BUF_ADDR(EP7, g_u32BulkBuf0);
                        MSC_Read();
                    }

                    return;
                }

                case UFI_MODE_SENSE_10:
                {
                    if (g_u32Length == 0)
                    {
                        g_u32Length = g_sCBW.dCBWDataTransferLength;
                        g_u32Address = MassCMD_BUF;
                    }

                    MSC_ModeSense10();
                    g_u8BulkState = BULK_IN;

                    if (g_u32Length > 0)
                    {
                        if (g_u32Length > EP7_MAX_PKT_SIZE)
                            g_u8Size = EP7_MAX_PKT_SIZE;
                        else
                            g_u8Size = g_u32Length;

                        /* Bulk IN buffer */
                        USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_u32Address, g_u8Size);

                        g_u32Address += g_u8Size;

                        USBD_SET_EP_BUF_ADDR(EP7, g_u32BulkBuf0);
                        MSC_Read();
                    }

                    return;
                }

                case UFI_INQUIRY:
                {
                    if (Hcount > 36)
                    {
                        g_u8Prevent = 1;
                        g_sCSW.dCSWDataResidue = Hcount;
                        g_sCSW.bCSWStatus = 0x1;
                        USBD_SET_EP_STALL(EP7);
                        g_u8BulkState = BULK_IN;
                        USBD_SET_DATA0(EP7);
                        MSC_AckCmd();
                    }
                    else
                    {

                        uint8_t u8PageCode;

                        u8PageCode = g_sCBW.au8Data[0];

                        g_u8BulkState = BULK_IN;

                        if (Hcount)
                        {
                            /* u8PageCode should be zero */
                            if (u8PageCode)
                            {
                                /* Expecting a STALL after data phase completes with a zero-length or short packet */
                                USBD_SET_EP_STALL(EP6);
                                USBD_LockEpStall(1 << EP6);

                                DBG_PRINTF("INQUIRY page code = %d", u8PageCode);
                            }

                            /* Bulk IN buffer */
                            USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_au8InquiryID, Hcount);
                            USBD_SET_PAYLOAD_LEN(EP7, Hcount);

                            DBG_PRINTF("Inquiry, len %d\n", u32Len);

                        }
                        else
                        {
                            /* Next is status phase if zero data length in data phase */
                            MSC_AckCmd();
                        }
                    }

                    return;

                }

                case UFI_READ_10:
                case UFI_READ_12:
                {

                    /* Check if it is a new transfer */
                    if (g_u32Length == 0)
                    {
                        Dcount = (get_be32(&g_sCBW.au8Data[4]) >> 8) * 512;

                        if (g_sCBW.bmCBWFlags == 0x80)      /* IN */
                        {
                            if (Hcount == Dcount)   /* Hi == Di (Case 6)*/
                            {
                            }
                            else if (Hcount < Dcount)     /* Hn < Di (Case 2) || Hi < Di (Case 7) */
                            {
                                if (Hcount)     /* Hi < Di (Case 7) */
                                {
                                    g_u8Prevent = 1;
                                    g_sCSW.bCSWStatus = 0x01;
                                    g_sCSW.dCSWDataResidue = 0;
                                }
                                else     /* Hn < Di (Case 2) */
                                {
                                    g_u8Prevent = 1;
                                    g_sCSW.bCSWStatus = 0x01;
                                    g_sCSW.dCSWDataResidue = 0;
                                    g_u8BulkState = BULK_IN;
                                    MSC_AckCmd();
                                    return;
                                }
                            }
                            else if (Hcount > Dcount)     /* Hi > Dn (Case 4) || Hi > Di (Case 5) */
                            {
                                g_u8Prevent = 1;
                                g_sCSW.bCSWStatus = 0x01;
                                g_sCSW.dCSWDataResidue = 0;
                            }
                        }
                        else     /* Ho <> Di (Case 10) */
                        {
                            g_u8Prevent = 1;
                            g_u32CbwStall = 1;
                            USBD_SET_EP_STALL(EP6);
                            g_sCSW.bCSWStatus = 0x01;
                            g_sCSW.dCSWDataResidue = Hcount;
                            g_u8BulkState = BULK_IN;
                            MSC_AckCmd();
                            return;
                        }
                    }

                    /* Get LBA address */
                    g_u32Address = get_be32(&g_sCBW.au8Data[0]);
                    g_u32LbaAddress = g_u32Address * UDC_SECTOR_SIZE;
                    g_u32Length = g_sCBW.dCBWDataTransferLength;
                    g_u32BytesInStorageBuf = g_u32Length;

                    i = g_u32Length;

                    if (i > STORAGE_BUFFER_SIZE)
                        i = STORAGE_BUFFER_SIZE;

                    MSC_ReadMedia(g_u32Address * UDC_SECTOR_SIZE, i, (uint8_t *)STORAGE_DATA_BUF);
                    g_u32BytesInStorageBuf = i;
                    g_u32LbaAddress += i;

                    g_u32Address = STORAGE_DATA_BUF;

                    /* Indicate the next packet should be Bulk IN Data packet */
                    g_u8BulkState = BULK_IN;

                    if (g_u32BytesInStorageBuf > 0)
                    {
                        /* Set the packet size */
                        if (g_u32BytesInStorageBuf > EP7_MAX_PKT_SIZE)
                            g_u8Size = EP7_MAX_PKT_SIZE;
                        else
                            g_u8Size = g_u32BytesInStorageBuf;

                        /* Prepare the first data packet (DATA1) */
                        /* Bulk IN buffer */
                        USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_u32Address, g_u8Size);
                        g_u32Address += g_u8Size;

                        /* kick - start */
                        USBD_SET_EP_BUF_ADDR(EP7, g_u32BulkBuf1);
                        /* Trigger to send out the data packet */
                        USBD_SET_PAYLOAD_LEN(EP7, g_u8Size);
                        g_u32Length -= g_u8Size;
                        g_u32BytesInStorageBuf -= g_u8Size;
                    }

                    return;

                }

                case UFI_WRITE_10:
                case UFI_WRITE_12:
                {
                    if (g_u32Length == 0)
                    {
                        Dcount = (get_be32(&g_sCBW.au8Data[4]) >> 8) * 512;

                        if (g_sCBW.bmCBWFlags == 0x00)      /* OUT */
                        {
                            if (Hcount == Dcount)   /* Ho == Do (Case 12)*/
                            {
                                g_sCSW.dCSWDataResidue = 0;
                                g_sCSW.bCSWStatus = 0;
                            }
                            else if (Hcount < Dcount)     /* Hn < Do (Case 3) || Ho < Do (Case 13) */
                            {
                                g_u8Prevent = 1;
                                g_sCSW.dCSWDataResidue = 0;
                                g_sCSW.bCSWStatus = 0x1;

                                if (Hcount == 0)    /* Hn < Do (Case 3) */
                                {
                                    g_u8BulkState = BULK_IN;
                                    MSC_AckCmd();
                                    return;
                                }
                            }
                            else if (Hcount > Dcount)     /* Ho > Do (Case 11) */
                            {
                                g_u8Prevent = 1;
                                g_sCSW.dCSWDataResidue = 0;
                                g_sCSW.bCSWStatus = 0x1;
                            }

                            g_u32Length = g_sCBW.dCBWDataTransferLength;
                            g_u32Address = STORAGE_DATA_BUF;
                            g_u32DataFlashStartAddr = get_be32(&g_sCBW.au8Data[0]) * UDC_SECTOR_SIZE;
                        }
                        else     /* Hi <> Do (Case 8) */
                        {
                            g_u8Prevent = 1;
                            g_sCSW.dCSWDataResidue = Hcount;
                            g_sCSW.bCSWStatus = 0x1;
                            USBD_SET_EP_STALL(EP7);
                            g_u8BulkState = BULK_IN;
                            USBD_SET_DATA0(EP7);
                            MSC_AckCmd();
                            return;
                        }
                    }

                    DBG_PRINTF("Write 0x%x  0x%x\n", g_u32Address, g_u32Length);

                    if ((g_u32Length > 0))
                    {
                        USBD_SET_PAYLOAD_LEN(EP6, EP6_MAX_PKT_SIZE);
                        g_u8BulkState = BULK_OUT;
                    }

                    return;
                }

                case UFI_MODE_SENSE_6:
                {
                    uint32_t u32Data = 0x3;
                    g_u8BulkState = BULK_IN;
                    USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)u32Data, 4);
                    USBD_SET_PAYLOAD_LEN(EP7, 4);
                    return;
                }

                case UFI_MODE_SELECT_6:
                case UFI_MODE_SELECT_10:
                {
                    g_u32Length = g_sCBW.dCBWDataTransferLength;
                    g_u32Address = MassCMD_BUF;

                    if (g_u32Length > 0)
                    {
                        USBD_SET_PAYLOAD_LEN(EP6, EP6_MAX_PKT_SIZE);
                        g_u8BulkState = BULK_OUT;
                    }
                    else
                    {
                        g_u8BulkState = BULK_IN;
                        MSC_AckCmd();
                    }

                    return;
                }

                default:
                {
                    /* Just stall for unknown command */
                    DBG_PRINTF("Unknow cmd 0x%x\n", g_sCBW.u8OPCode);

                    if (g_sCBW.bmCBWFlags & 0x80)
                        USBD_SET_PAYLOAD_LEN(EP7, 4);

                    g_u8BulkState = BULK_IN;
                    MSC_AckCmd();
                    return;
                }
            }
        }
        else if (g_u8BulkState == BULK_OUT)
        {
            switch (g_sCBW.u8OPCode)
            {
                case UFI_WRITE_10:
                case UFI_WRITE_12:
                case UFI_MODE_SELECT_6:
                case UFI_MODE_SELECT_10:
                {
                    MSC_Write();
                    return;
                }

                default:
                {
                    /* Bulk-out of unkonwn command. Just dorp them. */
                    if (g_u32Length > EP6_MAX_PKT_SIZE)
                    {
                        USBD_SET_EP_BUF_ADDR(EP6, g_u32BulkBuf0);
                        USBD_SET_PAYLOAD_LEN(EP6, EP6_MAX_PKT_SIZE);
                        g_u32Length -= EP6_MAX_PKT_SIZE;
                    }
                    else
                    {
                        g_u32Length = 0;
                        g_u8BulkState = BULK_IN;
                        MSC_AckCmd();
                    }

                    break;
                }
            }
        }
    }
}

void MSC_AckCmd(void)
{
    /* Bulk IN */
    if (g_u8BulkState == BULK_CSW)
    {
        /* Prepare to receive the CBW */
        g_u8BulkState = BULK_CBW;

        USBD_SET_EP_BUF_ADDR(EP6, g_u32BulkBuf0);
        USBD_SET_PAYLOAD_LEN(EP6, 31);

        DBG_PRINTF("CSW ack\n");

    }
    else if (g_u8BulkState == BULK_IN)
    {
        switch (g_sCBW.u8OPCode)
        {
            case UFI_READ_FORMAT_CAPACITY:
            case UFI_READ_CAPACITY:
            case UFI_READ_CAPACITY_16:
            case UFI_MODE_SENSE_10:
            {
                if (g_u32Length > 0)
                {
                    MSC_Read();
                    return;
                }

                if (g_sCBW.dCBWDataTransferLength > 36)
                    g_sCSW.dCSWDataResidue = g_sCBW.dCBWDataTransferLength - 36;
                else
                    g_sCSW.dCSWDataResidue = 0;

                g_sCSW.bCSWStatus = 0;
                break;
            }

            case UFI_READ_10:
            case UFI_READ_12:
            {
                if (g_u32Length > 0)
                {
                    MSC_ReadTrig();
                    return;
                }

                break;
            }

            case UFI_REQUEST_SENSE:
            case UFI_INQUIRY:
            {
                if (g_sCBW.dCBWDataTransferLength > 36)
                {
                    // Stall EP2 after short packet
                    //USBD_SET_EP_STALL(EP2);

                    g_sCSW.dCSWDataResidue = g_sCBW.dCBWDataTransferLength - 36;
                    g_sCSW.bCSWStatus = 0;
                    DBG_PRINTF("Inquiry size > 36\n");
                }
                else
                {
                    g_sCSW.dCSWDataResidue = 0;
                    g_sCSW.bCSWStatus = 0;
                    DBG_PRINTF("Inquiry ack, %x\n", USBD->EP[5].CFGP);
                }

                break;
            }

            case UFI_WRITE_10:
            case UFI_WRITE_12:
            case UFI_TEST_UNIT_READY:
                break;

            case UFI_PREVENT_ALLOW_MEDIUM_REMOVAL:
            case UFI_VERIFY_10:
            case UFI_START_STOP:

            {
                int32_t tmp;

                tmp = g_sCBW.dCBWDataTransferLength - STORAGE_BUFFER_SIZE;

                if (tmp < 0)
                    tmp = 0;

                g_sCSW.dCSWDataResidue = tmp;
                g_sCSW.bCSWStatus = 0;
                break;
            }

            case UFI_MODE_SENSE_6:
            {
                g_sCSW.dCSWDataResidue = g_sCBW.dCBWDataTransferLength - 4;
                g_sCSW.bCSWStatus = 0;

                break;
            }

            default:
            {
                // Unknown command
                g_sCSW.dCSWDataResidue = g_sCBW.dCBWDataTransferLength;
                g_sCSW.bCSWStatus = 1; // return command failed

                break;

            }
        }

        /* Return the CSW */
        USBD_SET_EP_BUF_ADDR(EP7, g_u32BulkBuf1);

        /* Bulk IN buffer */
        USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)&g_sCSW.dCSWSignature, 16);

        g_u8BulkState = BULK_CSW;
        USBD_SET_PAYLOAD_LEN(EP7, 13);

        DBG_PRINTF("Prepare CSW\n");


    }
    else
    {
        // This should be a DATA phase error.
        USBD_SET_EP_STALL(EP7);
        USBD_SET_EP_STALL(EP6);
        USBD_LockEpStall((1 << EP7) | (1 << EP6));

        DBG_PRINTF("Unexpected IN ack\n");

    }
}

void MSC_ReadMedia(uint32_t addr, uint32_t size, uint8_t *buffer)
{
    DataFlashRead(addr, size, (uint32_t)buffer);
}

void MSC_SetConfig(void)
{
    // Clear stall status and ready
    USBD->EP[5].CFGP = 1;
    USBD->EP[6].CFGP = 1;
    /*****************************************************/
    /* EP7 ==> Bulk IN endpoint, address 6 */
    USBD_CONFIG_EP(EP7, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM_1);
    /* Buffer range for EP7 */
    USBD_SET_EP_BUF_ADDR(EP7, EP7_BUF_BASE);

    /* EP6 ==> Bulk Out endpoint, address 5 */
    USBD_CONFIG_EP(EP6, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM_1);
    /* Buffer range for EP6 */
    USBD_SET_EP_BUF_ADDR(EP6, EP6_BUF_BASE);

    /* trigger to receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP6, EP6_MAX_PKT_SIZE);


    USBD_LockEpStall(0);

    g_u8BulkState = BULK_CBW;


    DBG_PRINTF("Set config\n");

}

