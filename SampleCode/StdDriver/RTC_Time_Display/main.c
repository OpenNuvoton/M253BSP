/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate the RTC function and displays current time to the
 *           UART console
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t  g_u32TICK = 0;

/*---------------------------------------------------------------------------------------------------------*/
/*                            Define functions prototype                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void RTC_TickHandle(void);
void RTC_IRQHandler(void);
void LXT_Enable(void);
void SYS_Init(void);
int32_t main(void);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*---------------------------------------------------------------------------------------------------------*/
/*                                        RTC Tick Handle                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void RTC_TickHandle(void)
{
    S_RTC_TIME_DATA_T sCurTime;

    /* Get the current time */
    RTC_GetDateAndTime(&sCurTime);

    printf(" Current Time:%u/%02u/%02u %02u:%02u:%02u\n", sCurTime.u32Year, sCurTime.u32Month, sCurTime.u32Day, sCurTime.u32Hour, sCurTime.u32Minute, sCurTime.u32Second);

    g_u32TICK++;
}

/**
  * @brief  RTC ISR to handle interrupt event
  * @param  None
  * @retval None
  */
void RTC_IRQHandler(void)
{

    /* tick interrupt occurred */
    if ((RTC->INTEN & RTC_INTEN_TICKIEN_Msk) && (RTC->INTSTS & RTC_INTSTS_TICKIF_Msk))
    {
        RTC->INTSTS = 0x2;

        RTC_TickHandle();
    }

}

/*---------------------------------------------------------------------------------------------------------*/
/*                           Enable the external 32768Hz XTAL Clock                                        */
/*---------------------------------------------------------------------------------------------------------*/
void LXT_Enable(void)
{
    /* Set X32_OUT(PF.4) and X32_IN(PF.5) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE4_Msk | GPIO_MODE_MODE5_Msk);

    /* Enable external 32768Hz XTAL */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Disable digital input path of analog pin X32_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 4));

    /* Disable digital input path of analog pin XT32_IN to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 5));
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                      Init System Clock                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{

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

    /* Enable external 32768Hz XTAL */
    LXT_Enable();

    /* Debug UART clock setting */
    UartDebugCLK();

    /* Enable RTC peripheral clock */
    CLK_EnableModuleClock(RTC_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /*                                          Init I/O Multi-function                                        */
    /*---------------------------------------------------------------------------------------------------------*/
    UartDebugMFP();

    /* Lock protected registers */
    SYS_LockReg();

}

/*---------------------------------------------------------------------------------------------------------*/
/*                                          MAIN function                                                  */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    S_RTC_TIME_DATA_T sInitTime;

    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Init Debug UART */
    UartDebugInit();

    printf("\n\nCPU @ %u Hz\n", SystemCoreClock);
    /* Time Setting */
    sInitTime.u32Year       = 2018;
    sInitTime.u32Month      = 12;
    sInitTime.u32Day        = 11;
    sInitTime.u32Hour       = 13;
    sInitTime.u32Minute     = 30;
    sInitTime.u32Second     = 0;
    sInitTime.u32DayOfWeek  = RTC_TUESDAY;
    sInitTime.u32TimeScale  = RTC_CLOCK_24;

    if (RTC_Open(&sInitTime) != 0)
    {
        printf("\n RTC initial fail!!");
        printf("\n Please check h/w setting!!");

        while (1);
    }

    printf("\n RTC Time Display Test (Exit after 10 seconds)\n\n");

    /* Set Tick Period */
    RTC_SetTickPeriod(RTC_TICK_1_SEC);

    /* Enable RTC Tick Interrupt */
    RTC_EnableInt(RTC_INTEN_TICKIEN_Msk);
    NVIC_EnableIRQ(RTC_IRQn);

    g_u32TICK = 0;

    while (g_u32TICK < 10);

    printf("\n RTC Time Display Test End !!\n");

    /* Disable RTC Tick Interrupt */
    RTC_DisableInt(RTC_INTEN_TICKIEN_Msk);
    NVIC_DisableIRQ(RTC_IRQn);

    while (1);

}
