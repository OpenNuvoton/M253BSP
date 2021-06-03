/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Transmit and receive data from PC terminal through RS232 interface.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"

#define RXBUFSIZE   1024

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_au8RecData[RXBUFSIZE]  = {0};
volatile uint32_t g_u32ComRbytes = 0;
volatile uint32_t g_u32ComRhead  = 0;
volatile uint32_t g_u32ComRtail  = 0;
volatile int32_t g_i32Wait         = TRUE;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void SYS_Init(void);
void UART4_IRQHandler(void);
void UART_TEST_HANDLE(void);
void UART_FunctionTest(void);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Debug UART clock setting*/
    UartDebugCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for Debug UART RXD and TXD */
    UartDebugMFP();

    /* Lock protected registers */
    SYS_LockReg();

}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Test Sample                                                                                        */
/* Test Item                                                                                               */
/* It sends the received data to HyperTerminal.                                                            */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Init Debug UART for printf */
    UartDebugInit();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    printf("\nUART Sample Program\n");

    /* UART sample function */
    UART_FunctionTest();

    while (1);

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 4 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART4_IRQHandler(void)
{
    UART_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART_TEST_HANDLE(void)
{
    uint32_t u32IntSts = UART4->INTSTS;

    if ((u32IntSts & UART_INTSTS_RDAINT_Msk) || (u32IntSts & UART_INTSTS_RXTOINT_Msk))
    {
        printf("\nInput:");

        /* Get all the input characters */
        while (UART_GET_RX_EMPTY(UART4) == 0)
        {
            /* Get the character from UART Buffer */
            uint8_t u8InChar = UART_READ(UART4);

            printf("%c\n", u8InChar);

            if (u8InChar == '0')
            {
                g_i32Wait = FALSE;
            }

            /* Check if buffer full */
            if (g_u32ComRbytes < RXBUFSIZE)
            {
                /* Enqueue the character */
                g_au8RecData[g_u32ComRtail] = u8InChar;
                g_u32ComRtail = (g_u32ComRtail == (RXBUFSIZE - 1)) ? 0 : (g_u32ComRtail + 1);
                g_u32ComRbytes++;
            }
        }

        printf("Transmission Test:");
        /*Forces a write of all user-space buffered data for the given output*/
        fflush(stdout);

    }

    if (u32IntSts & UART_INTSTS_THREINT_Msk)
    {
        uint16_t u16Temp;
        u16Temp = g_u32ComRtail;

        if (g_u32ComRhead != u16Temp)
        {
            uint8_t u8InChar = g_au8RecData[g_u32ComRhead];

            while (UART_IS_TX_FULL(UART4)); /* Wait Tx is not full to transmit data */

            UART_WRITE(UART4, u8InChar);
            g_u32ComRhead = (g_u32ComRhead == (RXBUFSIZE - 1)) ? 0 : (g_u32ComRhead + 1);
            g_u32ComRbytes--;
        }
    }

    if (UART4->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART4->FIFOSTS = (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk);
    }

}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void UART_FunctionTest(void)
{
    printf("+-----------------------------------------------------------+\n");
    printf("|  UART Function Test                                       |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will print input char on terminal      |\n");
    printf("|    Please enter any to start     (Press '0' to exit)      |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
        Using a RS232 cable to connect UART4 and PC.
        UART4 is set to debug port. UART4 is enable RDA and RLS interrupt.
        When inputting char to terminal screen, RDA interrupt will happen and
        UART4 will print the received char on screen.
    */

    UART_SetTimeoutCnt(UART4, 0x10); // Set Rx Time-out counter

    // Set RX FIFO Interrupt Trigger Level
    UART4->FIFO &= ~ UART_FIFO_RFITL_Msk;
    UART4->FIFO |= UART_FIFO_RFITL_1BYTE;

    /* Enable UART RDA/THRE/Time-out interrupt */
    NVIC_EnableIRQ(UART4_IRQn);
    UART_EnableInt(UART4, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));

    while (g_i32Wait);

    /* Disable UART RDA/THRE/Time-out interrupt */
    UART_DisableInt(UART4, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
    g_i32Wait = TRUE;
    printf("\nUART Sample Demo End.\n");

}




