/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Two Single-Wire Loopback data test.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "string.h"
#include "NuMicro.h"

#define BUFSIZE   1024

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_au8TxData [BUFSIZE] = {0};
volatile uint8_t g_au8RecData[BUFSIZE] = {0};
volatile uint32_t g_u32RecLen  =  0;
volatile int32_t  g_i32RecOK  = FALSE;


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void SYS_Init(void);
void UART1_Init(void);
void UART2_Init(void);
void UART1_IRQHandler(void);
void UART1_TEST_HANDLE(void);
void UART2_IRQHandler(void);
void UART2_TEST_HANDLE(void);
void UART_FunctionTest(void);
uint8_t CheckPattern(uint32_t u32Addr0, uint32_t u32Addr1, uint32_t u32Length);
void BuildSrcPattern(uint32_t u32Addr, uint8_t u8Type, uint32_t u32Length);

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

    /* Select IP clock source */
    /* Select UART1 clock source is HIRC */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));
    /* Select UART2 clock source is HIRC */
    CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HIRC, CLK_CLKDIV4_UART2(1));

    /* Enable UART1 peripheral clock */
    CLK_EnableModuleClock(UART1_MODULE);
    /* Enable UART2 peripheral clock */
    CLK_EnableModuleClock(UART2_MODULE);

    /* Enable GPB peripheral clock */
    CLK_EnableModuleClock(GPB_MODULE);

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

    /* Set PB multi-function pins for UART1 RXD */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB2MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB2MFP_UART1_RXD);

    /* Set PB multi-function pins for UART2 RXD */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB0MFP_UART2_RXD);

    /*Set PB2 IO status is Pull-up*/
    GPIO_SetPullCtl(PB, BIT2, GPIO_PUSEL_PULL_UP);

    /*Set PB0 IO status is Pull-up*/
    GPIO_SetPullCtl(PB, BIT0, GPIO_PUSEL_PULL_UP);

    /* Lock protected registers */
    SYS_LockReg();
}



/*---------------------------------------------------------------------------------------------------------*/
/*                               Init Single Wire(UART1)                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_Init(void)
{
    /* Configure Single Wire(UART1) and set Single Wire(UART1) baud rate */
    UART_Open(UART1, 115200);
    UART_SelectSingleWireMode(UART1);
}

/*---------------------------------------------------------------------------------------------------------*/
/*                               Init Single Wire(UART2)                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void UART2_Init(void)
{
    /* Configure Single Wire(UART2) and set Single Wire(UART2) baud rate */
    UART_Open(UART2, 115200);
    UART_SelectSingleWireMode(UART2);
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Test Sample                                                                                        */
/* Test Item                                                                                               */
/* Debug port control the Single wire 1(UART1) send data to Single wire 2(UART2)  or Single wire 2(UART2)  */
/* send data to Single wire 1(UART1)                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/*                                         Main Function                                                   */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
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

    /* Init UART1 and UART2 for Single Wire Test*/
    UART1_Init();
    UART2_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /*                                           SAMPLE CODE                                                   */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("\nUART Sample Program\n");

    /* UART sample function */
    UART_FunctionTest();

    while (1);

}

/*---------------------------------------------------------------------------------------------------------*/
/*                       ISR to handle UART Channel 1 interrupt event                                      */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    UART1_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                  UART1 Callback function                                                */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_TEST_HANDLE(void)
{

    uint32_t u32IntSts = UART1->INTSTS;

    if (u32IntSts & UART_INTSTS_RDAINT_Msk)
    {
        /* Get all the input characters */
        while (UART_IS_RX_READY(UART1))
        {
            /* Get the character from UART Buffer */
            g_au8RecData[g_u32RecLen] = UART_READ(UART1);

            if (g_u32RecLen == BUFSIZE - 1)
            {
                g_i32RecOK = TRUE;
                g_u32RecLen = 0;
            }
            else
            {
                g_u32RecLen++;
            }
        }
    }

    if (u32IntSts & UART_INTSTS_SWBEINT_Msk)
    {
        printf("Single-wire Bit Error Detection \n");
        UART_ClearIntFlag(UART1, UART_INTSTS_SWBEINT_Msk);
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 2 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART2_IRQHandler(void)
{
    UART2_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART2 Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART2_TEST_HANDLE(void)
{

    uint32_t u32IntSts = UART2->INTSTS;

    if (u32IntSts & UART_INTSTS_RDAINT_Msk)
    {
        /* Get all the input characters */
        while (UART_IS_RX_READY(UART2))
        {
            /* Get the character from UART Buffer */
            g_au8RecData[g_u32RecLen] = UART_READ(UART2);

            if (g_u32RecLen == BUFSIZE - 1)
            {
                g_i32RecOK = TRUE;
                g_u32RecLen = 0;
            }
            else
            {
                g_u32RecLen++;
            }
        }
    }

    if (u32IntSts & UART_INTSTS_SWBEINT_Msk)
    {
        printf("Single-wire Bit Error Detection \n");
        UART_ClearIntFlag(UART2, UART_INTSTS_SWBEINT_Msk);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*                              Build Source Pattern function                                              */
/*---------------------------------------------------------------------------------------------------------*/
void BuildSrcPattern(uint32_t u32Addr, uint8_t u8Type, uint32_t u32Length)
{
    uint32_t u32Index = 0, u32Pattern = 0;
    uint8_t *pu8Addr;
    pu8Addr = (uint8_t *)u32Addr;

    if (u8Type == 0)      u32Pattern = 0x1f;
    else if (u8Type == 1) u32Pattern = 0x3f;
    else if (u8Type == 2) u32Pattern = 0x7f;
    else if (u8Type == 3) u32Pattern = 0xff;
    else  u32Pattern = 0xff;

    for (u32Index = 0; u32Index < u32Length ; u32Index++)
        pu8Addr[u32Index] = (u32Index & u32Pattern);

}

/*---------------------------------------------------------------------------------------------------------*/
/*                    Verify that the received data is correct                                             */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t CheckPattern(uint32_t u32Addr0, uint32_t u32Addr1, uint32_t u32Length)
{
    uint32_t u32Index = 0;
    uint8_t u8Result = 1;
    uint8_t *pu8Addr0;
    uint8_t *pu8Addr1;
    pu8Addr0 = (uint8_t *)u32Addr0;
    pu8Addr1 = (uint8_t *)u32Addr1;

    for (u32Index = 0; u32Index < u32Length ; u32Index++)
    {
        if (pu8Addr0[u32Index] != pu8Addr1[u32Index])
        {
            printf("Data Error Idex=%u,tx =%u,rx=%u\n", u32Index, pu8Addr0[u32Index], pu8Addr1[u32Index]) ;
            u8Result = 0;
        }
    }

    return u8Result;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void UART_FunctionTest(void)
{
    char chCmd ;

    printf("+-----------------------------------------------------------+\n");
    printf("|            UART Single Wire Function Test                 |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will print input char on terminal      |\n");
    printf("|    The user must connect the UART1 RX pin(PB2) to         |\n");
    printf("|    UART2_Rx Pin(PB0).                                     |\n");
    printf("|    Single Wire 1(PB2)send data to Single Wire 2(PB0).     |\n");
    printf("|    Single Wire 2(PB0)send data to Single Wire 1(PB2).     |\n");
    printf("|    Please enter any to start    (Press '0' to exit)       |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
        Using a RS232 cable to connect UART4 and PC.UART4 is set to debug port.
        UART1 and UART2 is enable RDA and RLS interrupt.
        The user can use UART4 to control the transmission or reception of UART1(Single Wire mode)
        When UART1(Single Wire 1)transfers data to UART2(Single Wire 2), if data is valid,
        it will enter the interrupt and receive the data.And then check the received data.
        When UART2(Single Wire 2)transfers data to UART1(Single Wire 1), if data is valid,
        it will enter the interrupt and receive the data.And then check the received data.
    */

    /* Enable UART1 RDA/Time-out / Single-wire Bit Error Detection interrupt */
    NVIC_EnableIRQ(UART1_IRQn);
    UART_EnableInt(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_SWBEIEN_Msk));
    /* Enable UART2 RDA/Time-out / Single-wire Bit Error Detection interrupt */
    NVIC_EnableIRQ(UART2_IRQn);
    UART_EnableInt(UART2, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_SWBEIEN_Msk));


    do
    {
        printf("+------------------------------------------------------------+\n");
        printf("|              UART Single Wire Test Item                    |\n");
        printf("+------------------------------------------------------------+\n");
        printf("|    (1)Single Wire 1(PB2)send data to Single Wire 2(PB0).   |\n");
        printf("|    (2)Single Wire 2(PB0)send data to Single Wire 1(PB2).   |\n");
        printf("|    (E)Exit                                                 |\n");
        printf("+------------------------------------------------------------+\n");

        chCmd = getchar();

        switch (chCmd)
        {
            case '1':
            {
                printf("SW1(UART1) --> SW2(UART2)Test :");
                g_i32RecOK  = FALSE;
                BuildSrcPattern((uint32_t)g_au8TxData, UART_WORD_LEN_8, BUFSIZE);

                /* Check the Rx status is Idle */
                while (!UART_RX_IDLE(UART1)) {};

                UART_Write(UART1, g_au8TxData, BUFSIZE);

                while (g_i32RecOK != TRUE) {}

                CheckPattern((uint32_t)g_au8TxData, (uint32_t)g_au8RecData, BUFSIZE) ? printf(" Pass\n") : printf(" Fail\n");
                /* Clear the Tx and Rx data buffer */
                memset((uint8_t *)g_au8TxData, 0, BUFSIZE);
                memset((uint8_t *)g_au8RecData, 0, BUFSIZE);
            }
            break;

            case '2':
            {
                printf("SW2(UART2) --> SW1(UART1)Test :");
                g_i32RecOK  = FALSE;
                BuildSrcPattern((uint32_t)g_au8TxData, UART_WORD_LEN_8, BUFSIZE);

                /* Check the Rx status is Idle */
                while (!UART_RX_IDLE(UART2)) {};

                UART_Write(UART2, g_au8TxData, BUFSIZE);

                while (g_i32RecOK != TRUE) {};

                CheckPattern((uint32_t)g_au8TxData, (uint32_t)g_au8RecData, BUFSIZE) ? printf(" Pass\n") :   printf(" Fail\n");

                /* Clear the Tx and Rx data buffer */
                memset((uint8_t *)g_au8TxData, 0, BUFSIZE);

                memset((uint8_t *)g_au8RecData, 0, BUFSIZE);
            }
            break;

            default:
                break;
        }

    } while ((chCmd != 'E') && (chCmd != 'e'));

    /* Disable UART1 RDA/Time-out interrupt */
    UART_DisableInt(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_SWBEIEN_Msk));
    /* Disable UART2 RDA/Time-out interrupt */
    UART_DisableInt(UART2, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_SWBEIEN_Msk));
    printf("\nUART Sample Demo End.\n");

}
