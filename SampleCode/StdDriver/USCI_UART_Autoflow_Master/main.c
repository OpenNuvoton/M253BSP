/****************************************************************************
 * @file     main.c
 * @version  V0.10
 * @brief
 *           Transmit and receive data with auto flow control.
 *           This sample code needs to work with USCI_UART_Autoflow_Slave.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif


/*User can use GPIO instead of nCTS pin, you can use this definition to open alternatives*/
//#define NOT_SUPPORT_CTS_PIN

/*Use GPIO PA2 instead of USCI_CTL0(CTS) pin*/
#if defined (NOT_SUPPORT_CTS_PIN)
    #define  UUART_CTS   PC2
#endif

#define RXBUFSIZE  1024

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_AutoFlow_FunctionTxTest(void);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 48MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable peripheral clock */
    CLK_EnableModuleClock(USCI0_MODULE);

#if (defined(NOT_SUPPORT_CTS_PIN))
    /* Enable GPC peripheral clock */
    CLK_EnableModuleClock(GPC_MODULE);
#endif
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Debug UART clock setting*/
    UartDebugCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for Debug UART RXD and TXD */
    UartDebugMFP();

    /* Set UUART0 multi-function pins */
    SYS->GPA_MFPH = SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA10MFP_Msk | SYS_GPA_MFPH_PA9MFP_Msk);
    SYS->GPA_MFPH = SYS->GPA_MFPH | (SYS_GPA_MFPH_PA10MFP_USCI0_DAT0 | SYS_GPA_MFPH_PA9MFP_USCI0_DAT1);
    SYS->GPB_MFPH = SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB15MFP_Msk);
    SYS->GPB_MFPH = SYS->GPB_MFPH | (SYS_GPB_MFPH_PB15MFP_USCI0_CTL1);
#if  !(defined(NOT_SUPPORT_CTS_PIN))
    SYS->GPC_MFPH = SYS->GPC_MFPH & ~SYS_GPC_MFPH_PC14MFP_Msk;
    SYS->GPC_MFPH = SYS->GPC_MFPH | SYS_GPC_MFPH_PC14MFP_USCI0_CTL0;
#else
    /*User can use GPIO instead of nCTS pin*/
    /*Replace the USCI_CTL0(CTS) pin with GPIO PC2 */
    SYS->GPC_MFPL = SYS->GPC_MFPL & ~SYS_GPC_MFPL_PC2MFP_Msk;

#endif

}

void USCI0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure USCI0 as UART mode */
    UUART_Open(UUART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif


    /* Lock protected registers */
    SYS_LockReg();

    /* Init Debug UART for printf */
    UartDebugInit();
    /* Init USCI0 for test */
    USCI0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    printf("\nUSCI UART Sample Program\n");

    /* USCI UART auto flow sample master function */
    USCI_AutoFlow_FunctionTxTest();

    while (1);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  AutoFlow Function Tx Test                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_AutoFlow_FunctionTxTest()
{
    uint32_t u32Idx;

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Pin Configure                                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  ______                                            _____  |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |Master|                                          |Slave| |\n");
    printf("| |    TX|--USCI0_DAT1   <======> USCI0_DAT0      --|RX   | |\n");
#if  !(defined(NOT_SUPPORT_CTS_PIN))
    printf("| |  nCTS|--USCI0_CTL0   <======> USCI0_CTL1      --|nRTS | |\n");
#else
    printf("| |  nCTS|--GPIO         <======> USCI0_CTL1      --|nRTS | |\n");
#endif
    printf("| |______|                                          |_____| |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n");

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|       AutoFlow Function Test (Master)                     |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code needs two boards. One is Master and    |\n");
    printf("|    the other is slave. Master will send 1k bytes data     |\n");
    printf("|    to slave. Slave will check if received data is correct |\n");
    printf("|    after getting 1k bytes data.                           |\n");
    printf("|    Press any key to start...                              |\n");
    printf("+-----------------------------------------------------------+\n");
    getchar();

    /* Enable CTS and RTS auto flow control */
#if !(defined(NOT_SUPPORT_CTS_PIN))
    UUART_EnableFlowCtrl(UUART0);
#else
    /*Use GPIO PC2 instead of USCI_CTL0(CTS) pin*/
    GPIO_SetMode(PC, BIT2, GPIO_MODE_INPUT);
#endif

    /* Send 1k bytes data */
    for (u32Idx = 0; u32Idx < RXBUFSIZE; u32Idx++)
    {
#if defined(NOT_SUPPORT_CTS_PIN)

        /*Use GPIO PA2 instead of USCI_CTL0(CTS) pin*/
        /*USCI_CTL0(CTS) is Low active*/
        while (UUART_CTS != 0) {};

#endif
        /* Send 1 byte data */
        UUART_WRITE(UUART0, (u32Idx & 0xFF));

        /* Wait if Tx FIFO is full */
        while (UUART_GET_TX_FULL(UUART0));
    }

    printf("\n Transmit Done\n");

}
