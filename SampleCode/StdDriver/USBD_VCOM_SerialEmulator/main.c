/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief
 *           Demonstrate how to implement a USB virtual com port device.
 *           It supports one virtual comport.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "vcom_serial.h"

#define CRYSTAL_LESS    1 /* CRYSTAL_LESS must be 1 if USB clock source is HIRC */
#define TRIM_INIT           (SYS_BASE+0x118)

/*--------------------------------------------------------------------------*/
STR_VCOM_LINE_CODING g_sLineCoding = {115200, 0, 0, 8};   /* Baud rate: 115200     */
/* Stop bit : 1 Stops Bit*/
/* parity   : None       */
/* data bits: 8 Bits     */
uint16_t g_u16CtrlSignal = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

/*--------------------------------------------------------------------------*/
#define RXBUFSIZE           512 /* RX buffer size */
#define TXBUFSIZE           512 /* RX buffer size */

#define TX_FIFO_SIZE        1  /* TX Hardware FIFO size */


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
/* UART4 */
volatile uint8_t  g_au8ComRbuf[RXBUFSIZE];
volatile uint16_t g_u16ComRbytes = 0;
volatile uint16_t g_u16ComRhead = 0;
volatile uint16_t g_u16ComRtail = 0;

volatile uint8_t  g_au8ComTbuf[TXBUFSIZE];
volatile uint16_t g_u16ComTbytes = 0;
volatile uint16_t g_u16ComThead = 0;
volatile uint16_t g_u16ComTtail = 0;

uint8_t g_au8RxBuf[64] = {0};
volatile uint8_t *g_pu8RxBuf = 0;
volatile uint32_t g_u32RxSize = 0;
volatile uint32_t g_u32TxSize = 0;

volatile int8_t g_i8BulkOutReady = 0;



/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal HIRC 48 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal HIRC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable module clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /* Debug UART clock setting */
    UartDebugCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    UartDebugMFP();

}


/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART4_IRQHandler(void)
{
    uint32_t u32IntStatus;
    uint8_t bInChar;

    u32IntStatus = UART4->INTSTS;

    if ((u32IntStatus & UART_INTSTS_RDAIF_Msk) || (u32IntStatus & UART_INTSTS_RXTOIF_Msk))
    {
        /* Receiver FIFO threashold level is reached or RX time out */

        /* Get all the input characters */
        while ((UART4->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0)
        {
            /* Get the character from UART Buffer */
            bInChar = UART4->DAT;

            /* Check if buffer full */
            if (g_u16ComRbytes < RXBUFSIZE)
            {
                /* Enqueue the character */
                g_au8ComRbuf[g_u16ComRtail++] = bInChar;

                if (g_u16ComRtail >= RXBUFSIZE)
                    g_u16ComRtail = 0;

                g_u16ComRbytes++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if (u32IntStatus & UART_INTSTS_THREIF_Msk)
    {
        if (g_u16ComTbytes && (UART4->INTEN & UART_INTEN_THREIEN_Msk))
        {
            /* Fill the TX FIFO */
            int32_t size = g_u16ComTbytes;

            if (size >= TX_FIFO_SIZE)
            {
                size = TX_FIFO_SIZE;
            }

            while (size)
            {
                if (g_u16ComThead >= TXBUFSIZE)
                    g_u16ComThead = 0;

                bInChar = g_au8ComTbuf[g_u16ComThead++];
                UART4->DAT = bInChar;

                g_u16ComTbytes--;
                size--;
            }
        }
        else
        {
            /* No more data, just stop TX (Stop work) */
            UART4->INTEN &= (~UART_INTEN_THREIEN_Msk);
        }
    }

}

void VCOM_TransferData(void)
{
    int32_t i;

    /* Check wether USB is ready for next packet or not*/
    if (g_u32TxSize == 0)
    {
        int32_t i32Len;

        /* Check wether we have new COM Rx data to send to USB or not */
        if (g_u16ComRbytes)
        {
            i32Len = g_u16ComRbytes;

            if (i32Len > EP2_MAX_PKT_SIZE)
                i32Len = EP2_MAX_PKT_SIZE;

            for (i = 0; i < i32Len; i++)
            {
                if (g_u16ComRhead >= RXBUFSIZE)
                    g_u16ComRhead = 0;

                g_au8RxBuf[i] = g_au8ComRbuf[g_u16ComRhead++];
            }

            __set_PRIMASK(1);
            g_u16ComRbytes -= i32Len;
            __set_PRIMASK(0);

            g_u32TxSize = i32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), (uint8_t *)g_au8RxBuf, i32Len);
            USBD_SET_PAYLOAD_LEN(EP2, i32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EP2_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            i32Len = USBD_GET_PAYLOAD_LEN(EP2);

            if (i32Len == EP2_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(EP2, 0);
        }
    }

    /* Process the Bulk out data when bulk out data is ready. */
    if (g_i8BulkOutReady && (g_u32RxSize <= TXBUFSIZE - g_u16ComTbytes))
    {
        for (i = 0; i < g_u32RxSize; i++)
        {
            g_au8ComTbuf[g_u16ComTtail++] = g_pu8RxBuf[i];

            if (g_u16ComTtail >= TXBUFSIZE)
                g_u16ComTtail = 0;
        }

        __set_PRIMASK(1);
        g_u16ComTbytes += g_u32RxSize;
        __set_PRIMASK(0);

        g_u32RxSize = 0;
        g_i8BulkOutReady = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }

    /* Process the software TX FIFO */
    if (g_u16ComTbytes)
    {
        /* Check if TX is working */
        if ((UART4->INTEN & UART_INTEN_THREIEN_Msk) == 0)
        {
            if (g_u16ComThead >= TXBUFSIZE)
            {
                g_u16ComThead = 0;
            }

            /* Send one bytes out */
            UART4->DAT = g_au8ComTbuf[g_u16ComThead++];

            g_u16ComTbytes--;

            /* Enable TX Empty Interrupt. (Trigger first one) */
            UART4->INTEN |= UART_INTEN_THREIEN_Msk;
        }
    }
}



void PowerDown()
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Wakeup Enable */
    USBD_ENABLE_INT(USBD_INTEN_WKEN_Msk);

    CLK_PowerDown();

    /* Clear PWR_DOWN_EN if it is not clear by itself */
    if (CLK->PWRCTL & CLK_PWRCTL_PDEN_Msk)
        CLK->PWRCTL ^= CLK_PWRCTL_PDEN_Msk;

    /* Lock protected registers */
    SYS_LockReg();
}

void UART4_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART4_RST);

    /* Configure UART4 and set UART4 Baudrate */
    UART_Open(UART4, 115200);

    /* Enable UART4 RX Time-Out Interrupt and RX Data Available Interrupt */
    UART_EnableInt(UART4, UART_INTEN_TOCNTEN_Msk | UART_INTEN_RDAIEN_Msk);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
#if CRYSTAL_LESS
    uint32_t u32TrimInit;
#endif
    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();
    UART4_Init();

    printf("\n\n");
    printf("+------------------------------------------------------------+\n");
    printf("|          NuMicro USB Virtual COM Port Sample Code          |\n");
    printf("+------------------------------------------------------------+\n");

    /* Open USB controller */
    USBD_Open(&gsInfo, VCOM_ClassRequest, NULL);

    /* Endpoint configuration */
    VCOM_Init();
    /* Start USB device */
    USBD_Start();

    NVIC_EnableIRQ(USBD_IRQn);
    NVIC_EnableIRQ(UART4_IRQn);

#if CRYSTAL_LESS
    /* Backup default trim value */
    u32TrimInit = M32(TRIM_INIT);

    /* Clear SOF */
    USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);
#endif

    while (1)
    {
#if CRYSTAL_LESS

        /* Start USB trim function if it is not enabled. */
        if ((SYS->HIRCTRIMCTL & SYS_HIRCTRIMCTL_FREQSEL_Msk) != 0x1)
        {
            /* Start USB trim only when USB signal arrived */
            if (USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
            {
                /* Clear SOF */
                USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);

                /*
                    USB clock trim function:
                    HIRC Trimming with boundary function enhances robustility
                    and keeps HIRC in right frequency while receiving unstable USB signal
                */
                SYS->HIRCTRIMCTL = (0x1 << SYS_HIRCTRIMCTL_REFCKSEL_Pos)
                                   | (0x1 << SYS_HIRCTRIMCTL_FREQSEL_Pos)
                                   | (0x0 << SYS_HIRCTRIMCTL_LOOPSEL_Pos)
                                   | (0x1 << SYS_HIRCTRIMCTL_BOUNDEN_Pos)
                                   | (10  << SYS_HIRCTRIMCTL_BOUNDARY_Pos);
            }
        }

        /* Disable USB Trim when any error found */
        if (SYS->HIRCTRIMSTS & (SYS_HIRCTRIMSTS_CLKERIF_Msk | SYS_HIRCTRIMSTS_TFAILIF_Msk))
        {
            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;

            /* Disable USB clock trim function */
            SYS->HIRCTRIMCTL = 0;

            /* Clear trim error flags */
            SYS->HIRCTRIMSTS = SYS_HIRCTRIMSTS_CLKERIF_Msk | SYS_HIRCTRIMSTS_TFAILIF_Msk;

            /* Clear SOF */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);
        }

#endif

        /* Enter power down when USB suspend */
        if (g_u8Suspend)
            PowerDown();

        VCOM_TransferData();
    }
}
