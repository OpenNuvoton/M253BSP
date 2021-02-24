/****************************************************************************//**
 * @file     main.c
 * @version  V1.01
 * @brief    Use the timer pin PB.5 to demonstrate inter timer trigger mode
 *           function. Also display the measured input frequency to UART console.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

volatile uint8_t u8Complete = 0;

// Timer 0 is working in event count mode, and Timer 1 in capture mode, so we read the
// capture value from Timer 1, _not_ timer 0.
void TMR1_IRQHandler(void)
{
    // Timer clock is 48 MHz, counter value records the duration for 100 event counts.
    printf("Event frequency is %ld Hz\n", FREQ_48MHZ / TIMER_GetCounter(TIMER1) * 100);
    TIMER_ClearCaptureIntFlag(TIMER1);
    u8Complete = 1;
}

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

    /* Set both PCLK0 and PCLK1 as HCLK */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1;

    /* Enable LIRC clock (Internal RC 38.4KHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
    /* Wait for LIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    /* Enable IP clock */
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_PCLK0, 0);

    /* Debug UART clock setting */
    UartDebugCLK();

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    UartDebugMFP();

    /* Set timer event counting pin (PB.5)*/
    SYS->GPB_MFPL = SYS_GPB_MFPL_PB5MFP_TM0;

    /* Set multi-function pins for CLKO(PB.14) */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB14MFP_Msk) | SYS_GPB_MFPH_PB14MFP_CLKO;
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_LIRC, 0, 1);

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init Debug UART */
    UartDebugInit();

    /* This sample code demonstrate inter timer trigger mode using Timer0 and Timer1
     * In this mode, Timer0 is working as counter, and triggers Timer1. Using Timer1
     * to calculate the amount of time used by Timer0 to count specified amount of events.
     * By dividing the time period recorded in Timer1 by the event counts, we get
     * the event frequency.
     */
    printf("Inter timer trigger mode demo code\n");
    printf("Please connect input source PB.14(CLKO_LIRC 38.4K) with Timer 0 counter pin PB.5, press any key to continue\n");
    getchar();

    // Give a dummy target frequency here. Will over write prescale and compare value with macro
    TIMER_Open(TIMER0, TIMER_ONESHOT_MODE, 100);

    // Update prescale and compare value. Calculate average frequency every 100 events
    TIMER_SET_PRESCALE_VALUE(TIMER0, 0);
    TIMER_SET_CMP_VALUE(TIMER0, 100);

    // Update Timer 1 prescale value. So Timer 0 clock is 48MHz
    TIMER_SET_PRESCALE_VALUE(TIMER1, 0);

    // We need capture interrupt
    NVIC_EnableIRQ(TMR1_IRQn);

    while (1)
    {
        u8Complete = 0;
        // Count event by timer 0, disable drop count (set to 0), disable timeout (set to 0). Enable interrupt after complete
        TIMER_EnableFreqCounter(TIMER0, 0, 0, TRUE);

        while (u8Complete == 0);
    }
}
