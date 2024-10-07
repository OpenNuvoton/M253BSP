/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to minimize power consumption when entering power down mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
*/

/*
// <o0> Power-down Mode
//      <0=> PD
//      <2=> FWPD
*/
#define SET_PDMSEL       0

/*
// <o0> POR
//      <0=> Disable
//      <1=> Enable
*/
#define SET_POR       0

/*
// <o0> LIRC
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LIRC       0

/*
// <o0> LXT
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LXT       0

#define GPIO_P0_TO_P15      0xFFFF

void PowerDownFunction(void);
void GPB_IRQHandler(void);
void PorSetting(void);
int32_t LircSetting(void);
int32_t LxtSetting(void);
void SYS_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    /* Check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TX_EMPTY(UART4)

    if (--u32TimeOutCnt == 0) break;

    /* Select Power-down mode */
    CLK_SetPowerDownMode(SET_PDMSEL << CLK_PMUCTL_PDMSEL_Pos);

    /* Enter to Power-down mode */
    CLK_PowerDown();
}

/**
 * @brief       GPIO PB IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PB default IRQ, declared in startup_M253.s.
 */
void GPB_IRQHandler(void)
{
    volatile uint32_t u32temp;

    /* To check if PB.2 interrupt occurred */
    if (GPIO_GET_INT_FLAG(PB, BIT2))
    {
        GPIO_CLR_INT_FLAG(PB, BIT2);
        printf("PB2 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PB interrupts */
        u32temp = PB->INTSRC;
        PB->INTSRC = u32temp;
        printf("Un-expected interrupts.\n");
    }
}

void PorSetting(void)
{
    if (SET_POR == 0)
    {
        SYS_DISABLE_POR();
    }
    else
    {
        SYS_ENABLE_POR();
    }
}

int32_t LircSetting(void)
{
    uint32_t u32TimeOutCnt;

    if (SET_LIRC == 0)
    {
        CLK_DisableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while (CLK->STATUS & CLK_STATUS_LIRCSTB_Msk)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for LIRC disable time-out!\n");
                return -1;
            }
        }
    }
    else
    {
        CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

        if (CLK_WaitClockReady(CLK_PWRCTL_LIRCEN_Msk) == 0)
        {
            printf("Wait for LIRC enable time-out!\n");
            return -1;
        }
    }

    return 0;
}

int32_t LxtSetting(void)
{
    uint32_t u32TimeOutCnt;

    if (SET_LXT == 0)
    {
        CLK_DisableXtalRC(CLK_PWRCTL_LXTEN_Msk);
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while (CLK->STATUS & CLK_STATUS_LXTSTB_Msk)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for LXT disable time-out!\n");
                return -1;
            }
        }
    }
    else
    {
        CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

        if (CLK_WaitClockReady(CLK_PWRCTL_LXTEN_Msk) == 0)
        {
            printf("Wait for LXT enable time-out!\n");
            return -1;
        }
    }

    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init System                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable HIRC and clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable all GPIO clock */
    CLK->AHBCLK |= CLK_AHBCLK_GPACKEN_Msk | CLK_AHBCLK_GPBCKEN_Msk | CLK_AHBCLK_GPCCKEN_Msk | CLK_AHBCLK_GPFCKEN_Msk;

    /* Debug UART clock setting */
    UartDebugCLK();

    /* Init UART4 multi-function pins, RXD(PA.2) and TXD(PA.3) */
    UartDebugMFP();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init Debug UART */
    UartDebugInit();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------------------------+\n");
    printf("|  SYS_PowerDown_MinCurrent and Wake-up by PB.2 Sample Code         |\n");
    printf("+-------------------------------------------------------------------+\n\n");

    printf("+-------------------------------------------------------------------+\n");
    printf("|  Operating sequence                                               |\n");
    printf("|  1. Remove R16                                                    |\n");
    printf("|  2. Configure all GPIO as Quasi-bidirectional Mode                |\n");
    printf("|  3. Disable analog function, e.g. POR module                      |\n");
    printf("|  4. Disable unused clock, e.g. LIRC                               |\n");
    printf("|  5. Enter to Power-Down                                           |\n");
    printf("|  6. Wait for PB.2 falling-edge interrupt event to wake-up the MCU |\n");
    printf("+-------------------------------------------------------------------+\n\n");

    /* To measure Power-down current on NuMaker-M253LE board, remove R16 and measure the current on Ammeter Connector */

    /* Check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TX_EMPTY(UART4)

    if (--u32TimeOutCnt == 0) break;

    /* Set function pin to GPIO mode except UART pin to print message */
    SYS->GPA_MFPL = (SYS_GPA_MFPL_PA2MFP_UART4_RXD | SYS_GPA_MFPL_PA3MFP_UART4_TXD);
    SYS->GPA_MFPH = 0;
    SYS->GPB_MFPL = 0;
    SYS->GPB_MFPH = 0;
    SYS->GPC_MFPL = 0;
    SYS->GPC_MFPH = 0;
    SYS->GPD_MFPL = 0;
    SYS->GPD_MFPH = 0;
    SYS->GPE_MFPL = 0;
    SYS->GPE_MFPH = 0;
    SYS->GPF_MFPL = 0;
    SYS->GPF_MFPH = 0;

    /* Configure all GPIO as Quasi-bidirectional Mode */
    GPIO_SetMode(PA, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PB, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PC, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PF, GPIO_P0_TO_P15, GPIO_MODE_QUASI);

    /* Unlock protected registers for Power-down and wake-up setting */
    SYS_UnlockReg();

    /* POR setting */
    PorSetting();

    /* LIRC setting */
    if (LircSetting() < 0) goto lexit;

    /* LXT setting */
    if (LxtSetting() < 0) goto lexit;

    /* Wake-up source configuraion */
    if ((SET_PDMSEL == CLK_PMUCTL_PDMSEL_PD) || (SET_PDMSEL == CLK_PMUCTL_PDMSEL_FWPD))
    {
        /* Configure PB.2 as Quasi mode and enable interrupt by falling edge trigger */
        GPIO_SetMode(PB, BIT2, GPIO_MODE_QUASI);
        GPIO_EnableInt(PB, 2, GPIO_INT_FALLING);
        NVIC_EnableIRQ(GPB_IRQn);
    }
    else
    {
        printf("Unknown Power-down mode!\n");
        goto lexit;
    }

    /* Enter to Power-down mode */
    if (SET_PDMSEL == CLK_PMUCTL_PDMSEL_PD)        printf("Enter to PD Power-Down ......\n");
    else if (SET_PDMSEL == CLK_PMUCTL_PDMSEL_FWPD) printf("Enter to FWPD Power-Down ......\n");

    PowerDownFunction();

    /* Waiting for PB.2 falling-edge interrupt event */
    printf("System waken-up done.\n\n");

lexit:

    while (1);

}
