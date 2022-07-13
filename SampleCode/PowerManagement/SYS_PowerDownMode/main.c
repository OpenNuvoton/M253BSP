/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to enter different Power-down mode and wake-up by RTC.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"


#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif


static volatile uint32_t s_u32RTCTickINT = 0;

void RTC_IRQHandler(void);
void RTC_Init(void);
void CheckPowerSource(void);
void SYS_Init(void);


/*---------------------------------------------------------------------------------------------------------*/
/*  RTC IRQ Handler                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void RTC_IRQHandler(void)
{
    /* To check if RTC tick interrupt occurred */
    if (RTC_GET_TICK_INT_FLAG() == 1)
    {
        /* Clear RTC tick interrupt flag */
        RTC_CLEAR_TICK_INT_FLAG();
    }

    s_u32RTCTickINT++;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for RTC wake-up source setting                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void RTC_Init(void)
{
    S_RTC_TIME_DATA_T sWriteRTC;

    /* Init RTC in the start of sample code */
    if (RTC->INIT != RTC_INIT_ACTIVE_Msk)
    {
        printf("\n\nCPU @ %u Hz\n", SystemCoreClock);
        printf("+------------------------------------------+\n");
        printf("|    Power-down and Wake-up Sample Code    |\n");
        printf("+------------------------------------------+\n\n");

        /* Open RTC */
        sWriteRTC.u32Year       = 2020;
        sWriteRTC.u32Month      = 5;
        sWriteRTC.u32Day        = 15;
        sWriteRTC.u32DayOfWeek  = RTC_FRIDAY;
        sWriteRTC.u32Hour       = 0;
        sWriteRTC.u32Minute     = 0;
        sWriteRTC.u32Second     = 0;
        sWriteRTC.u32TimeScale  = RTC_CLOCK_24;

        if (RTC_Open(&sWriteRTC) != 0)
        {
            printf("\n RTC initial fail!!");
            printf("\n Please check h/w setting!!");

            while (1);
        }

        printf("# Set RTC current date/time: 2020/05/15 00:00:00.\n");
    }

    /* Clear RTC tick interrupt flag */
    RTC_CLEAR_TICK_INT_FLAG();

    /* Enable RTC NVIC */
    NVIC_EnableIRQ(RTC_IRQn);

    /* Enable RTC tick interrupt and wake-up function will be enabled also */
    RTC_EnableInt(RTC_INTEN_TICKIEN_Msk);
    RTC_SetTickPeriod(RTC_TICK_1_SEC);

}

void CheckResetStatus(void)
{
    static uint32_t u32ResetCounter = 1UL;

    const uint32_t u32RestStatus = SYS_GetResetSrc();

    /* Reset status register included PMURF,PINRF and PORF after reset */
    printf("Reset Status 0x%x,included...\n", u32RestStatus);

    if (u32RestStatus & SYS_RSTSTS_CPULKRF_Msk)
        printf("[%d] CPU Lockup Reset Flag\n", u32ResetCounter++);

    if (u32RestStatus & SYS_RSTSTS_CPURF_Msk)
        printf("[%d] CPU Reset Flag\n", u32ResetCounter++);

    if (u32RestStatus & SYS_RSTSTS_PMURF_Msk)
        printf("[%d] PMU Reset Flag\n", u32ResetCounter++);

    if (u32RestStatus & SYS_RSTSTS_SYSRF_Msk)
        printf("[%d] System Reset Flag\n", u32ResetCounter++);

    if (u32RestStatus & SYS_RSTSTS_BODRF_Msk)
        printf("[%d] BOD Reset Flag\n", u32ResetCounter++);

    if (u32RestStatus & SYS_RSTSTS_LVRF_Msk)
        printf("[%d] LVR Reset Flag\n", u32ResetCounter++);

    if (u32RestStatus & SYS_RSTSTS_WDTRF_Msk)
        printf("[%d] WDT Reset Flag\n", u32ResetCounter++);

    if (u32RestStatus & SYS_RSTSTS_PINRF_Msk)
        printf("[%d] nRESET Pin Reset Flag\n", u32ResetCounter++);

    if (u32RestStatus & SYS_RSTSTS_PORF_Msk)
        printf("[%d] POR Reset Flag\n", u32ResetCounter++);

    /* Clear all reset flag */
    SYS->RSTSTS = SYS->RSTSTS;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Wait for LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Debug UART clock setting */
    UartDebugCLK();

    /* Enable peripheral clock */
    CLK_EnableModuleClock(RTC_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    UartDebugMFP();

    /* Set multi-function pins for CLKO(PB.14, PF.15) */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB14MFP_Msk) | SYS_GPB_MFPH_PB14MFP_CLKO;
    SYS->GPF_MFPH = (SYS->GPF_MFPH & ~SYS_GPF_MFPH_PF15MFP_Msk) | SYS_GPF_MFPH_PF15MFP_CLKO;

    /* Disable digital input path of analog pin X32_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, BIT4 | BIT5);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    S_RTC_TIME_DATA_T sReadRTC;

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART */
    UartDebugInit();

    /* Enable Clock Output function, output clock is stopped in Power-down mode */
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HCLK, 3, 0);

    /* Get Reset Status */
    CheckResetStatus();

    /* RTC wake-up source setting */
    RTC_Init();

    /*
     *  This sample code will enter different Power-down mode and wake-up by RTC:
     *  1. Normal Power-down mode (PD).
     *  2. Fast Wake-up Power-down mode (FWPD).
     */
    while (1)
    {
        uint8_t u8Item;

        printf("+------------------------------------------+\n");
        printf("|    Please select Power-down mode         |\n");
        printf("+------------------------------------------+\n");
        printf("|[1] Normal Power-down mode (PD)           |\n");
        printf("|[2] Fast Wake-up Power-down mode (FWPD)   |\n");
        printf("+------------------------------------------+\n");
        u8Item = getchar();

        switch (u8Item)
        {
            case '1':
                printf("\nSystem enters PD power-down mode ...\n");
                CLK_SetPowerDownMode(CLK_PMUCTL_PDMSEL_PD);
                break;

            case '2':
                printf("\nSystem enters FWPD power-down mode ...\n");
                CLK_SetPowerDownMode(CLK_PMUCTL_PDMSEL_FWPD);
                break;

            default:
                break;
        }

        /* To check if all the debug messages are finished */
        while (IsDebugFifoEmpty() == 0);

        /* Clear RTC tick interrupt counter */
        s_u32RTCTickINT = 0;

        /* Wait and synchronize the next RTC tick */
        while (s_u32RTCTickINT == 0);

        s_u32RTCTickINT = 0;

        /* Enter Power-down mode */
        CLK_PowerDown();

        /* RTC tick wake-up */
        while (s_u32RTCTickINT == 0);

        printf("Wake-up!\n");

        /* Read current RTC date/time after wake-up */
        RTC_GetDateAndTime(&sReadRTC);
        printf("# Get RTC current date/time: %u/%02u/%02u %02u:%02u:%02u.\n\n\r",
               sReadRTC.u32Year, sReadRTC.u32Month, sReadRTC.u32Day, sReadRTC.u32Hour, sReadRTC.u32Minute, sReadRTC.u32Second);

    }
}
