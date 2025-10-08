/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Demonstrate how to implement a USB multi virtual COM ports device with
 *           a terminal echo port.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "vcom_multi_port_cmd.h"

#define CRYSTAL_LESS    1 /* CRYSTAL_LESS must be 1 if USB clock source is HIRC */
#define TRIM_INIT           (SYS_BASE+0x118)

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 48 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Switch HCLK clock source to Internal HIRC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable module clock */
#if (VCOM_CNT>=1)
    CLK_EnableModuleClock(UART0_MODULE);
#endif
#if (VCOM_CNT>=2)
    CLK_EnableModuleClock(UART1_MODULE);
#endif
#if (VCOM_CNT>=3)
    CLK_EnableModuleClock(UART2_MODULE);
#endif
#if (VCOM_CNT>=4)
    CLK_EnableModuleClock(UART3_MODULE);
#endif
#if (VCOM_CNT>=5)
    /*VCOM4 internal loopback*/
#endif

    UartDebugCLK();
    CLK_EnableModuleClock(USBD_MODULE);

    /* Select module clock source */
#if (VCOM_CNT>=1)
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
#endif
#if (VCOM_CNT>=2)
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));
#endif
#if (VCOM_CNT>=3)
    CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HIRC, CLK_CLKDIV4_UART2(1));
#endif
#if (VCOM_CNT>=4)
    CLK_SetModuleClock(UART3_MODULE, CLK_CLKSEL3_UART3SEL_HIRC, CLK_CLKDIV4_UART3(1));
#endif
#if (VCOM_CNT>=5)
    /*VCOM4 internal loopback*/
#endif
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
#if (VCOM_CNT>=1)
    /* Set PB.12 and PB.13 multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB12MFP_Msk) | SYS_GPB_MFPH_PB12MFP_UART0_RXD;
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB13MFP_Msk) | SYS_GPB_MFPH_PB13MFP_UART0_TXD;
#endif
#if (VCOM_CNT>=2)
    /* Set PB.2 and PB.3 multi-function pins for UART1 RXD and TXD */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB2MFP_Msk) | SYS_GPB_MFPL_PB2MFP_UART1_RXD;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB3MFP_Msk) | SYS_GPB_MFPL_PB3MFP_UART1_TXD;
#endif
#if (VCOM_CNT>=3)
    /* Set PB.4 and PB.5 multi-function pins for UART2 RXD and TXD */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB4MFP_Msk) | SYS_GPB_MFPL_PB4MFP_UART2_RXD;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB5MFP_Msk) | SYS_GPB_MFPL_PB5MFP_UART2_TXD;
#endif
#if (VCOM_CNT>=4)
    /* Set PC.2 and PC.3 multi-function pins for UART3 RXD and TXD */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~SYS_GPC_MFPL_PC2MFP_Msk) | SYS_GPC_MFPL_PC2MFP_UART3_RXD;
    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~SYS_GPC_MFPL_PC3MFP_Msk) | SYS_GPC_MFPL_PC3MFP_UART3_TXD;
#endif
#if (VCOM_CNT>=5)
    /*VCOM4 internal loopback*/
#endif
    UartDebugMFP();

    /* Set PB.14 as clock output pin */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB14MFP_Msk) | SYS_GPB_MFPH_PB14MFP_CLKO;

    /* Enable CLKO (PA.3) for monitor HCLK. CLKO = HCLK/8 Hz */
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HCLK, 2, 0);
}

#if (VCOM_CNT>=1)
void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
    UART0->FIFO = (UART0->FIFO & ~UART_FIFO_RFITL_Msk) | UART_FIFO_DEPTH;
    UART_SetTimeoutCnt(UART0, TUNE_UART_TIMTOUT);

    /* Enable UART0 RX Time-Out Interrupt and RX Data Available Interrupt */
    UART_EnableInt(UART0, UART_INTEN_RXTOIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RDAIEN_Msk);
}

#endif

#if (VCOM_CNT>=2)
void UART1_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART1_RST);

    /* Configure UART1 and set UART1 Baudrate */
    UART_Open(UART1, 115200);
    UART1->FIFO = (UART1->FIFO & ~UART_FIFO_RFITL_Msk) | UART_FIFO_DEPTH;
    UART_SetTimeoutCnt(UART1, TUNE_UART_TIMTOUT);

    /* Enable UART1 RX Time-Out Interrupt and RX Data Available Interrupt */
    UART_EnableInt(UART1, UART_INTEN_RXTOIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RDAIEN_Msk);
}

#endif

#if (VCOM_CNT>=3)
void UART2_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART2_RST);

    /* Configure UART2 and set UART2 Baudrate */
    UART_Open(UART2, 115200);
    UART2->FIFO = (UART2->FIFO & ~UART_FIFO_RFITL_Msk) | UART_FIFO_DEPTH;
    UART_SetTimeoutCnt(UART2, TUNE_UART_TIMTOUT);

    /* Enable UART2 RX Time-Out Interrupt and RX Data Available Interrupt */
    UART_EnableInt(UART2, UART_INTEN_RXTOIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RDAIEN_Msk);
}

#endif
#if (VCOM_CNT>=4)
void UART3_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART3_RST);

    /* Configure UART3 and set UART3 Baudrate */
    UART_Open(UART3, 115200);
    UART3->FIFO = (UART3->FIFO & ~UART_FIFO_RFITL_Msk) | UART_FIFO_DEPTH;
    UART_SetTimeoutCnt(UART3, TUNE_UART_TIMTOUT);

    /* Enable UART3 RX Time-Out Interrupt and RX Data Available Interrupt */
    UART_EnableInt(UART3, UART_INTEN_RXTOIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RDAIEN_Msk);
}

#endif
#if (VCOM_CNT>=5)
    /* VCOM internal loopback*/
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART_IRQHandler(VCOM_CONTROL_BLOCK_t *tVCOM)
{
    uint32_t u32IntStatus;
    uint8_t bInChar;

    u32IntStatus = tVCOM->psPeripheralInfo->UART_BASE->INTSTS;

    if ((u32IntStatus & UART_INTSTS_RDAIF_Msk) || (u32IntStatus & UART_INTSTS_RXTOIF_Msk))
    {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while ((tVCOM->psPeripheralInfo->UART_BASE->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0)
        {
            /* Get the character from UART Buffer */
            bInChar = tVCOM->psPeripheralInfo->UART_BASE->DAT;

            /* Check if buffer full */
            if (tVCOM->psCtrlVar->u16ComRbytes < RXBUFSIZE)
            {
                /* Enqueue the character */
                tVCOM->psCtrlVar->au8ComRbuf[tVCOM->psCtrlVar->u16ComRtail++] = bInChar;

                if (tVCOM->psCtrlVar->u16ComRtail >= RXBUFSIZE)
                    tVCOM->psCtrlVar->u16ComRtail = 0;

                tVCOM->psCtrlVar->u16ComRbytes++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if ((u32IntStatus & UART_INTSTS_THREIF_Msk))
    {

        if (tVCOM->psCtrlVar->u16ComTbytes
                && (tVCOM->psPeripheralInfo->UART_BASE->INTEN & UART_INTEN_THREIEN_Msk))
            //checking if Tx FIFO empty interrupt enabled
            //to avoid race condition between main program and IRQ handler
        {
            /* Fill the Tx FIFO */
            uint32_t size = tVCOM->psCtrlVar->u16ComTbytes;

            if (size > tVCOM->psPeripheralInfo->u32TX_FIFO_SIZE)
            {
                size = tVCOM->psPeripheralInfo->u32TX_FIFO_SIZE;
            }

            while (size)
            {
                bInChar = tVCOM->psCtrlVar->au8ComTbuf[tVCOM->psCtrlVar->u16ComThead++];
                tVCOM->psPeripheralInfo->UART_BASE->DAT = bInChar;

                if (tVCOM->psCtrlVar->u16ComThead >= TXBUFSIZE)
                    tVCOM->psCtrlVar->u16ComThead = 0;

                tVCOM->psCtrlVar->u16ComTbytes--;
                size--;
            }
        }
        else
        {
            /* No more data, just stop Tx (Stop work) */
            tVCOM->psPeripheralInfo->UART_BASE->INTEN &= (~UART_INTEN_THREIEN_Msk);
        }
    }

}

#if (VCOM_CNT>=1)
void UART0_IRQHandler(void)
{
    UART_IRQHandler(&tVCOM[IDX_VCOM0]);
}

#endif
#if (VCOM_CNT>=2)
void UART1_IRQHandler(void)
{
    UART_IRQHandler(&tVCOM[IDX_VCOM1]);
}

#endif
#if (VCOM_CNT>=3)
void UART2_IRQHandler(void)
{
    UART_IRQHandler(&tVCOM[IDX_VCOM2]);
}

#endif
#if (VCOM_CNT>=4)
void UART3_IRQHandler(void)
{
    UART_IRQHandler(&tVCOM[IDX_VCOM3]);
}

#endif
#if (VCOM_CNT>=5)
    /* VCOM4 internal loopback*/

#endif

void VCOM_TransferData(void)
{
    uint32_t i, u32Len;

#if VCOM_CNT == 1
    const uint32_t u32chidx = 0;
#else
    uint32_t u32chidx;

    for (u32chidx = 0; u32chidx < (VCOM_CNT - 1); u32chidx++)
#endif
    {
        /* Check whether USB is ready for next packet or not */
        if (tVCOM[u32chidx].psCtrlVar->u32TxSize == 0)
        {
            /* Check whether we have new COM Rx data to send to USB or not */
            if (tVCOM[u32chidx].psCtrlVar->u16ComRbytes)
            {
                u32Len = tVCOM[u32chidx].psCtrlVar->u16ComRbytes;

                if (u32Len > DATA_EP_MAX_PKT_SIZE)
                    u32Len = DATA_EP_MAX_PKT_SIZE;

                for (i = 0; i < u32Len; i++)
                {
                    tVCOM[u32chidx].psCtrlVar->au8RxBuf[i] = tVCOM[u32chidx].psCtrlVar->au8ComRbuf[tVCOM[u32chidx].psCtrlVar->u16ComRhead++];

                    if (tVCOM[u32chidx].psCtrlVar->u16ComRhead >= RXBUFSIZE)
                        tVCOM[u32chidx].psCtrlVar->u16ComRhead = 0;
                }

                __set_PRIMASK(1);
                tVCOM[u32chidx].psCtrlVar->u16ComRbytes -= u32Len;
                __set_PRIMASK(0);

                tVCOM[u32chidx].psCtrlVar->u32TxSize = u32Len;
                USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(GET_BULK_IN_EP(u32chidx))), (uint8_t *)tVCOM[u32chidx].psCtrlVar->au8RxBuf, u32Len);
                USBD_SET_PAYLOAD_LEN(GET_BULK_IN_EP(u32chidx), u32Len);
            }
            else
            {
                /* Prepare a zero packet if previous packet size is DATA_EP_MAX_PKT_SIZE and
                   no more data to send at this moment to note Host the transfer has been done */
                u32Len = USBD_GET_PAYLOAD_LEN(GET_BULK_IN_EP(u32chidx));

                if (u32Len == DATA_EP_MAX_PKT_SIZE)
                    USBD_SET_PAYLOAD_LEN(GET_BULK_IN_EP(u32chidx), 0);
            }
        }

        /* Process the Bulk out data when bulk out data is ready. */
        if (tVCOM[u32chidx].psCtrlVar->i8BulkOutReady && (tVCOM[u32chidx].psCtrlVar->u32RxSize <= TXBUFSIZE - tVCOM[u32chidx].psCtrlVar->u16ComTbytes))
        {
            for (i = 0; i < tVCOM[u32chidx].psCtrlVar->u32RxSize; i++)
            {
                tVCOM[u32chidx].psCtrlVar->au8ComTbuf[tVCOM[u32chidx].psCtrlVar->u16ComTtail++] = tVCOM[u32chidx].psCtrlVar->pu8RxBuf[i];

                if (tVCOM[u32chidx].psCtrlVar->u16ComTtail >= TXBUFSIZE)
                    tVCOM[u32chidx].psCtrlVar->u16ComTtail = 0;
            }

            __set_PRIMASK(1);
            tVCOM[u32chidx].psCtrlVar->u16ComTbytes += tVCOM[u32chidx].psCtrlVar->u32RxSize;
            __set_PRIMASK(0);

            tVCOM[u32chidx].psCtrlVar->u32RxSize = 0;
            tVCOM[u32chidx].psCtrlVar->i8BulkOutReady = 0; /* Clear bulk out ready flag */

            /* Ready to get next BULK out */
            USBD_SET_PAYLOAD_LEN(GET_BULK_OUT_EP(u32chidx), DATA_EP_MAX_PKT_SIZE);
        }

        /* Process the software Tx FIFO */
        if (tVCOM[u32chidx].psCtrlVar->u16ComTbytes)
        {
            /* Check if Tx is working */
            if ((tVCOM[u32chidx].psPeripheralInfo->UART_BASE->INTEN & UART_INTEN_THREIEN_Msk) == 0)
            {
                /* Send one bytes out */
                tVCOM[u32chidx].psPeripheralInfo->UART_BASE->DAT = tVCOM[u32chidx].psCtrlVar->au8ComTbuf[tVCOM[u32chidx].psCtrlVar->u16ComThead++];

                if (tVCOM[u32chidx].psCtrlVar->u16ComThead >= TXBUFSIZE)
                {
                    tVCOM[u32chidx].psCtrlVar->u16ComThead = 0;
                }

                tVCOM[u32chidx].psCtrlVar->u16ComTbytes--;

                /* Enable Tx Empty Interrupt. (Trigger first one) */
                tVCOM[u32chidx].psPeripheralInfo->UART_BASE->INTEN |= UART_INTEN_THREIEN_Msk;
            }
        }

    }

#if (VCOM_CNT >= 5)

    /*
        Implement the VCOM internal loopback transfer with peripheral for demo.
        User can replace the following demo code by own code.
    */
    if (tVCOM[IDX_VCOM4].psCtrlVar->u32TxSize == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if (tVCOM[IDX_VCOM4].psCtrlVar->u16ComTbytes)
        {
            u32Len = tVCOM[IDX_VCOM4].psCtrlVar->u16ComTbytes;


            if (u32Len > DATA_EP_MAX_PKT_SIZE)
                u32Len = DATA_EP_MAX_PKT_SIZE;

            for (i = 0; i < u32Len; i++)
            {
                tVCOM[IDX_VCOM4].psCtrlVar->au8RxBuf[i] = tVCOM[IDX_VCOM4].psCtrlVar->au8ComTbuf[tVCOM[IDX_VCOM4].psCtrlVar->u16ComThead++];

                if (tVCOM[IDX_VCOM4].psCtrlVar->u16ComThead >= TXBUFSIZE)
                    tVCOM[IDX_VCOM4].psCtrlVar->u16ComThead = 0;
            }

            __set_PRIMASK(1);
            tVCOM[IDX_VCOM4].psCtrlVar->u16ComTbytes -= u32Len;
            __set_PRIMASK(0);

            tVCOM[IDX_VCOM4].psCtrlVar->u32TxSize = u32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(GET_BULK_IN_EP(IDX_VCOM4))), (uint8_t *)tVCOM[IDX_VCOM4].psCtrlVar->au8RxBuf, u32Len);
            USBD_SET_PAYLOAD_LEN(GET_BULK_IN_EP(IDX_VCOM4), u32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is DATA_EP_MAX_PKT_SIZE
            and no more data to send at this moment to note Host the transfer has been done */
            u32Len = USBD_GET_PAYLOAD_LEN(GET_BULK_IN_EP(IDX_VCOM4));

            if (u32Len == DATA_EP_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(GET_BULK_IN_EP(IDX_VCOM4), 0);
        }
    }

    if (tVCOM[IDX_VCOM4].psCtrlVar->i8BulkOutReady && (tVCOM[IDX_VCOM4].psCtrlVar->u32RxSize <= TXBUFSIZE - tVCOM[IDX_VCOM4].psCtrlVar->u16ComTbytes))
    {
        for (i = 0; i < tVCOM[IDX_VCOM4].psCtrlVar->u32RxSize; i++)
        {
            tVCOM[IDX_VCOM4].psCtrlVar->au8ComTbuf[tVCOM[IDX_VCOM4].psCtrlVar->u16ComTtail++] = tVCOM[IDX_VCOM4].psCtrlVar->pu8RxBuf[i];

            if (tVCOM[IDX_VCOM4].psCtrlVar->u16ComTtail >= TXBUFSIZE)
                tVCOM[IDX_VCOM4].psCtrlVar->u16ComTtail = 0;
        }

        __set_PRIMASK(1);
        tVCOM[IDX_VCOM4].psCtrlVar->u16ComTbytes += tVCOM[IDX_VCOM4].psCtrlVar->u32RxSize;
        __set_PRIMASK(0);

        tVCOM[IDX_VCOM4].psCtrlVar->u32RxSize = 0;
        tVCOM[IDX_VCOM4].psCtrlVar->i8BulkOutReady = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(GET_BULK_OUT_EP(IDX_VCOM4), DATA_EP_MAX_PKT_SIZE);
    }

#endif

}

void PowerDown()
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    // printf("Enter power down ...\n");

#if (VCOM_CNT>=1)

    while (!UART_IS_TX_EMPTY(tVCOM[IDX_VCOM0].psPeripheralInfo->UART_BASE));

#endif
#if (VCOM_CNT>=2)

    while (!UART_IS_TX_EMPTY(tVCOM[IDX_VCOM1].psPeripheralInfo->UART_BASE));

#endif
#if (VCOM_CNT>=3)

    while (!UART_IS_TX_EMPTY(tVCOM[IDX_VCOM2].psPeripheralInfo->UART_BASE));

#endif
#if (VCOM_CNT>=4)

    while (!UART_IS_TX_EMPTY(tVCOM[IDX_VCOM3].psPeripheralInfo->UART_BASE));

#endif
#if (VCOM_CNT>=5)
    //CMD loopback
#endif

    /* Wakeup Enable */
    USBD_ENABLE_INT(USBD_INTEN_WKEN_Msk);

    CLK_PowerDown();

    /* Clear PWR_DOWN_EN if it is not clear by itself */
    if (CLK->PWRCTL & CLK_PWRCTL_PDEN_Msk)
        CLK->PWRCTL ^= CLK_PWRCTL_PDEN_Msk;

    // printf("device wakeup!\n");

    /* Lock protected registers */
    SYS_LockReg();
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

#if (VCOM_CNT>=1)
    UART0_Init();
    NVIC_EnableIRQ(UART0_IRQn);
#endif
#if (VCOM_CNT>=2)
    UART1_Init();
    NVIC_EnableIRQ(UART1_IRQn);
#endif
#if (VCOM_CNT>=3)
    UART2_Init();
    NVIC_EnableIRQ(UART2_IRQn);
#endif
#if (VCOM_CNT>=4)
    UART3_Init();
    NVIC_EnableIRQ(UART3_IRQn);
#endif
#if (VCOM_CNT>=5)
    /* VCOM internally loopback */
#endif

    UartDebugInit();

    printf("\n\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|       NuMicro USB Virtual COM Multi Port Sample Code        |\n");
    printf("+-------------------------------------------------------------+\n");

    USBD_Open(&gsInfo, VCOM_ClassRequest, NULL);

    /* Endpoint configuration */
    VCOM_Init();
    USBD_Start();

    NVIC_EnableIRQ(USBD_IRQn);

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
