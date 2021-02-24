/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate the System run to power down mode and RTC Alram
 *           function to Wake up the system
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PD_MODE   0  // Power-down mode
#define FWPD_MODE 1  // Fast wake up

/*---------------------------------------------------------------------------------------------------------*/
/*                                       Global variables                                                  */
/*---------------------------------------------------------------------------------------------------------*/
volatile int32_t  g_i32Alarm  = FALSE;
volatile int32_t  g_i32WakeUp = FALSE;

/*---------------------------------------------------------------------------------------------------------*/
/*                            Define functions prototype                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void EnterToPowerDown(uint32_t u32PDMode);
void PWRWU_IRQHandler(void);
void RTC_IRQHandler(void);
void LXT_Enable(void);
void SYS_Init(void);
int32_t main(void);

/**
  * @brief      Enter To Power Down
  * @param[in]   u32PDMode    The specified Power down module.
  *                               - \ref CLK_PMUCTL_PDMSEL_PD      : Power-down mode
  *                               - \ref CLK_PMUCTL_PDMSEL_FWPD    : Fast wake up
  *
  * @return     None
  *
  * @details    This API is used to get the current RTC date and time value.
  */
void EnterToPowerDown(uint32_t u32PDMode)
{
    g_i32WakeUp = FALSE;
    SYS_UnlockReg();
    /* To program PWRCTL register, it needs to disable register protection first. */
    CLK->PMUCTL &= ~(CLK_PMUCTL_PDMSEL_Msk);

    if (u32PDMode == PD_MODE)
        CLK->PMUCTL |= CLK_PMUCTL_PDMSEL_PD;    //Power down
    else if ((u32PDMode == FWPD_MODE))
        CLK->PMUCTL |= CLK_PMUCTL_PDMSEL_FWPD;  //fast up

    CLK->PWRCTL &= ~(CLK_PWRCTL_PDEN_Msk | CLK_PWRCTL_PDWKIEN_Msk);
    CLK->PWRCTL |=  CLK_PWRCTL_PDWKIEN_Msk;
    CLK_PowerDown();
    SYS_LockReg();

    while (g_i32WakeUp == FALSE);

}

/*---------------------------------------------------------------------------------------------------------*/
/*                                         PWRWU  Handle                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void PWRWU_IRQHandler(void)
{
    g_i32WakeUp = TRUE;
    CLK->PWRCTL |= CLK_PWRCTL_PDWKIF_Msk;

}

/**
  * @brief  RTC ISR to handle interrupt event
  * @param  None
  * @retval None
  */
void RTC_IRQHandler(void)
{

    /* alarm interrupt occurred */
    if ((RTC->INTEN & RTC_INTEN_ALMIEN_Msk) && (RTC->INTSTS & RTC_INTSTS_ALMIF_Msk))
    {
        g_i32Alarm = TRUE;
        RTC_CLEAR_ALARM_INT_FLAG();
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
/*                                          Init System Clock                                              */
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

    /*Enable external 32768Hz XTAL */
    LXT_Enable();

    /* Enable RTC peripheral clock */
    CLK_EnableModuleClock(RTC_MODULE);

    /* Debug UART clock setting */
    UartDebugCLK();

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /*                                      Init I/O Multi-function                                            */
    /*---------------------------------------------------------------------------------------------------------*/
    UartDebugMFP();

    /* Lock protected registers */
    SYS_LockReg();

}

/*---------------------------------------------------------------------------------------------------------*/
/*                                     MAIN function                                                       */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    S_RTC_TIME_DATA_T sInitTime;
    S_RTC_TIME_DATA_T sCurTime;

    SYS_Init();

    /* Init Debug UART */
    UartDebugInit();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    /* Time Setting */
    sInitTime.u32Year       = 2018;
    sInitTime.u32Month      = 12;
    sInitTime.u32Day        = 11;
    sInitTime.u32Hour       = 13;
    sInitTime.u32Minute     = 30;
    sInitTime.u32Second     = 0;
    sInitTime.u32DayOfWeek  = RTC_TUESDAY;
    sInitTime.u32TimeScale  = RTC_CLOCK_24;

    RTC_Open(&sInitTime);

    printf("\n[ RTC Alarm Interrupt Wake-up Test ](Alarm after 10 seconds)\n\n");

    g_i32Alarm = FALSE;

    /* Get the current time */
    RTC_GetDateAndTime(&sCurTime);

    printf(" Current Time:%d/%02d/%02d %02d:%02d:%02d\n", sCurTime.u32Year, sCurTime.u32Month,
           sCurTime.u32Day, sCurTime.u32Hour, sCurTime.u32Minute, sCurTime.u32Second);

    /* Wait the UART FIFO buffer is empty */
    while (!IsDebugFifoEmpty()) {}; //Debug uart print port

    /* The alarm time setting */
    sCurTime.u32Second = sCurTime.u32Second + 10;

    /* Set the alarm time */
    RTC_SetAlarmDateAndTime(&sCurTime);

    /* Clear interrupt status */
    RTC->INTSTS = RTC_INTSTS_ALMIF_Msk;

    /* Enable RTC Alarm Interrupt */
    RTC_EnableInt(RTC_INTEN_ALMIEN_Msk);

    NVIC_EnableIRQ(RTC_IRQn);

    NVIC_EnableIRQ(PWRWU_IRQn);

    EnterToPowerDown(PD_MODE);

    printf(" RTC Wake Up success\n");

    while (!g_i32Alarm) {};

    /* Get the current time */
    RTC_GetDateAndTime(&sCurTime);

    printf(" Current Time:%d/%02d/%02d %02d:%02d:%02d\n", sCurTime.u32Year, sCurTime.u32Month,
           sCurTime.u32Day, sCurTime.u32Hour, sCurTime.u32Minute, sCurTime.u32Second);

    printf("\n RTC Alarm Interrupt Wake-up Test End !!\n");

    /* Disable RTC Tick Interrupt */
    RTC_DisableInt(RTC_INTEN_ALMIEN_Msk);

    NVIC_DisableIRQ(RTC_IRQn);

    NVIC_DisableIRQ(PWRWU_IRQn);

    while (1);

}
