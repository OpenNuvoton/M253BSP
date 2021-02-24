/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Use BPWM0 Channel 0(PA.0) to capture the PWM0 Channel 0(PB.5) Waveform.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/**
 * @brief       BPWM0 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle BPWM0 interrupt event
 */
void BPWM0_IRQHandler(void)
{
    if (BPWM_GetCaptureIntFlag(BPWM0, 0) > 1)
    {
        BPWM_ClearCaptureIntFlag(BPWM0, 0, BPWM_CAPTURE_INT_FALLING_LATCH);
    }
}

/*--------------------------------------------------------------------------------------*/
/* Capture function to calculate the input waveform information                         */
/* u16Count[4] : Keep the internal counter value when input signal rising / falling     */
/*               happens                                                                */
/*                                                                                      */
/* time    A    B     C     D                                                           */
/*           ___   ___   ___   ___   ___   ___   ___   ___                              */
/*      ____|   |_|   |_|   |_|   |_|   |_|   |_|   |_|   |_____                        */
/* index              0 1   2 3                                                         */
/*                                                                                      */
/* The capture internal counter down count from 0x10000, and reload to 0x10000 after    */
/* input signal falling happens (Time B/C/D)                                            */
/*--------------------------------------------------------------------------------------*/
void CalPeriodTime(BPWM_T *BPWM, uint32_t u32Ch)
{
    uint16_t u16Count[4];
    uint32_t u32i;
    uint16_t u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod;

    /* Clear Capture Falling Indicator (Time A) */
    BPWM_ClearCaptureIntFlag(BPWM, u32Ch, BPWM_CAPTURE_INT_FALLING_LATCH | BPWM_CAPTURE_INT_RISING_LATCH);

    /* Wait for Capture Falling Indicator  */
    while (BPWM_GetCaptureIntFlag(BPWM, u32Ch) < 2);

    /* Clear Capture Falling Indicator (Time B)*/
    BPWM_ClearCaptureIntFlag(BPWM, u32Ch, BPWM_CAPTURE_INT_FALLING_LATCH);

    u32i = 0;

    while (u32i < 4)
    {
        /* Wait for Capture Falling Indicator */
        while (BPWM_GetCaptureIntFlag(BPWM, u32Ch) < 2);

        /* Clear Capture Falling and Rising Indicator */
        BPWM_ClearCaptureIntFlag(BPWM, u32Ch, BPWM_CAPTURE_INT_FALLING_LATCH | BPWM_CAPTURE_INT_RISING_LATCH);

        /* Get Capture Falling Latch Counter Data */
        u16Count[u32i++] = BPWM_GET_CAPTURE_FALLING_DATA(BPWM, u32Ch);

        /* Wait for Capture Rising Indicator */
        while (BPWM_GetCaptureIntFlag(BPWM, u32Ch) < 1);

        /* Clear Capture Rising Indicator */
        BPWM_ClearCaptureIntFlag(BPWM, u32Ch, BPWM_CAPTURE_INT_RISING_LATCH);

        /* Get Capture Rising Latch Counter Data */
        u16Count[u32i++] = BPWM_GET_CAPTURE_RISING_DATA(BPWM, u32Ch);
    }

    u16RisingTime = u16Count[1];

    u16FallingTime = u16Count[0];

    u16HighPeriod = u16Count[1] - u16Count[2];

    u16LowPeriod = 0x10000 - u16Count[1];

    u16TotalPeriod = 0x10000 - u16Count[2];

    printf("\nPWM generate: \nHigh Period=11999 ~ 12001, Low Period=11999 ~ 12001, Total Period=23999 ~ 24001\n");
    printf("\nCapture Result: Rising Time = %d, Falling Time = %d \nHigh Period = %d, Low Period = %d, Total Period = %d.\n\n",
           u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod);

    if ((u16HighPeriod < 11999) || (u16HighPeriod > 12001) || (u16LowPeriod < 11999) || (u16LowPeriod > 12001) || (u16TotalPeriod < 23999) || (u16TotalPeriod > 24001))
        printf("Capture Test Fail!!\n");
    else
        printf("Capture Test Pass!!\n");
}

void SYS_Init(void)
{
    outpw(0x4000C04C, 0x103);
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Debug UART clock setting */
    UartDebugCLK();

    /* Enable BPWM module clock */
    CLK_EnableModuleClock(BPWM0_MODULE);

    /* Enable TIMER0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select TIMER0 module clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set UART Default MPF */
    UartDebugMFP();

    /* Set PA.0 multi-function pin for BPWM0 channel 0 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA0MFP_Msk) | SYS_GPA_MFPL_PA0MFP_BPWM0_CH0;

    /* Set PB.5 multi-function pin for TIMER0 toggle out */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB5MFP_Msk) | SYS_GPB_MFPL_PB5MFP_TM0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init Debug UART */
    UartDebugInit();

    printf("+------------------------------------------------------------------------+\n");
    printf("|                          BPWM Driver Sample Code                       |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use BPWM0 channel 0 to capture\n  the signal from TIMER0 toggle out pin.\n");
    printf("  I/O configuration:\n");
    printf("    BPWM0_CH0(PA.0 BPWM0 channel 0) <--> TM0(PB.5 TIMER0 toggle out pin)\n\n");
    printf("Use BPWM0 Channel 0(PA.0) to capture the TIMER0 toggle out pin(PB.5) Waveform\n");

    while (1)
    {
        printf("Press any key to start BPWM Capture Test\n");
        getchar();

        /*--------------------------------------------------------------------------------------*/
        /* Set the TM0 as BPWM output function.                                       */
        /*--------------------------------------------------------------------------------------*/

        /* To generate 500HZ toggle output, timer frequency must set to 1000Hz.
           Because toggle output state change on every timer timeout event */
        if (TIMER_Open(TIMER0, TIMER_TOGGLE_MODE, 1000) != 1000)
        {
            printf("Set the frequency different from the user\n");
        }

        /* Start Timer Counting */
        TIMER_Start(TIMER0);

        /*--------------------------------------------------------------------------------------*/
        /* Set the BPWM0 channel 0 for capture function                                          */
        /*--------------------------------------------------------------------------------------*/

        /* If input minimum frequency is 250Hz, user can calculate capture settings by follows.
           Capture clock source frequency = PCLK = 48000000 in the sample code.
           (CNR+1) = Capture clock source frequency/prescaler/minimum input frequency
                   = 48000000/4/250 = 48000
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 0xFFFF
           (Note: In capture mode, user should set CNR to 0xFFFF to increase capture frequency range.)
        */
        /* set BPWM0 channel 0 capture configuration */
        BPWM_ConfigCaptureChannel(BPWM0, 0, 80, 0);

        /* Enable capture falling edge interrupt for BPWM0 channel 0 */
        //BPWM_EnableCaptureInt(BPWM0, 0, BPWM_CAPTURE_INT_FALLING_LATCH);

        /* Enable BPWM0 NVIC interrupt */
        //NVIC_EnableIRQ(BPWM0_IRQn);

        /* Enable Timer for BPWM0 channel 0 */
        BPWM_Start(BPWM0, BPWM_CH_0_MASK);

        /* Enable Capture Function for BPWM0 channel 0 */
        BPWM_EnableCapture(BPWM0, BPWM_CH_0_MASK);

        /* Enable falling capture reload */
        BPWM0->CAPCTL |= BPWM_CAPCTL_FCRLDEN0_Msk;

        /* Wait until BPWM0 channel 0 Timer start to count */
        while ((BPWM0->CNT) == 0);

        /* Capture the Input Waveform Data */
        CalPeriodTime(BPWM0, 0);

        /*---------------------------------------------------------------------------------------------------------*/
        /* Stop BPWM0 channel 0 (Recommended procedure method 1)                                                    */
        /* Disable Timer0 Counting */
        /*---------------------------------------------------------------------------------------------------------*/

        /* Stop Timer Counting */
        TIMER_Stop(TIMER0);

        /*---------------------------------------------------------------------------------------------------------*/
        /* Stop BPWM0 channel 0 (Recommended procedure method 1)                                                    */
        /* Set BPWM Timer loaded value(Period) as 0. When BPWM internal counter(CNT) reaches to 0, disable BPWM Timer */
        /*---------------------------------------------------------------------------------------------------------*/

        /* Disable BPWM0 NVIC */
        //NVIC_DisableIRQ(BPWM0_IRQn);

        /* Set loaded value as 0 for BPWM0 channel 0 */
        BPWM_Stop(BPWM0, BPWM_CH_0_MASK);

        /* Wait until BPWM0 channel 0 current counter reach to 0 */
        while ((BPWM0->CNT & BPWM_CNT_CNT_Msk) != 0);

        /* Disable Timer for BPWM0 channel 0 */
        BPWM_ForceStop(BPWM0, BPWM_CH_0_MASK);

        /* Disable Capture Function and Capture Input path for BPWM0 channel 0 */
        BPWM_DisableCapture(BPWM0, BPWM_CH_0_MASK);

        /* Clear Capture Interrupt flag for BPWM0 channel 0 */
        BPWM_ClearCaptureIntFlag(BPWM0, 0, BPWM_CAPTURE_INT_FALLING_LATCH);
    }
}
