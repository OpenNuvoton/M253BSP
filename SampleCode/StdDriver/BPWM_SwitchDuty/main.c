/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Change duty cycle of output waveform by configured period.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/


uint32_t CalNewDutyCMR(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32DutyCycle, uint32_t u32CycleResolution);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable BPWM0 module clock */
    CLK_EnableModuleClock(BPWM0_MODULE);

    /* Debug UART clock setting */
    UartDebugCLK();

    /* Reset BPWM0 module */
    SYS_ResetModule(BPWM0_RST);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set UART Default MPF */
    UartDebugMFP();

    /* Set PA multi-function pins for BPWM0 Channel0 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA0MFP_Msk)) | SYS_GPA_MFPL_PA0MFP_BPWM0_CH0;

}

/**
 * @brief       Calculate the comparator value of new duty by configured period
 *
 * @param       bpwm                  The pointer of the specified BPWM module
 *
 * @param       u32ChannelNum        BPWM channel number. Valid values are between 0~5
 *
 * @param       u32DutyCycle         Target generator duty cycle percentage. Valid range are between 0 ~ u32CycleResolution.
 *                                   If u32CycleResolution is 100, and u32DutyCycle is 10 means 10%, 20 means 20% ...
 *
 * @param       u32CycleResolution   Target generator duty cycle resolution. The value in general is 100.
 *
 * @return      The comparator value by new duty cycle
 */
uint32_t CalNewDutyCMR(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32DutyCycle, uint32_t u32CycleResolution)
{
    if (u32DutyCycle >= u32CycleResolution)
        return BPWM_GET_CNR(bpwm, u32ChannelNum);

    return (u32DutyCycle * (BPWM_GET_CNR(bpwm, u32ChannelNum) + 1) / u32CycleResolution);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint8_t  u8Option;
    uint32_t u32NewDutyCycle = 0, u32NewCMR = 0;

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init Debug UART */
    UartDebugInit();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-----------------------------------------------------------------------------------+\n");
    printf("|                          BPWM Driver Sample Code                                  |\n");
    printf("+-----------------------------------------------------------------------------------+\n");
    printf("  This sample code will use BPWM0 channel 0 to output waveform, and switch duty cycle.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: BPWM0 channel 0(PA.0)\n");
    printf("\nOutput waveform is 1250Hz and it's duty is 50%%.\n");

    /*
      Configure BPWM0 channel 0 init period and duty(up counter type).
      Period is HIRC_Freq / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 48 MHz / (1 * (38399 + 1)) = 1250 Hz
      Duty ratio = (19200) / (38399 + 1) = 50%
    */

    /* BPWM0 channel 0 frequency is 1250Hz, duty 50%, */
    BPWM_ConfigOutputChannel(BPWM0, 0, 1250, 50);

    /* Enable output of BPWM0 channel 0 */
    BPWM_EnableOutput(BPWM0, BPWM_CH_0_MASK);

    /* Start BPWM counter */
    BPWM_Start(BPWM0, BPWM_CH_0_MASK);

    while (1)
    {
        printf("\nSelect new duty: \n");
        printf("[1] 100%% \n");
        printf("[2] 75%% \n");
        printf("[3] 25%% \n");
        printf("[4] 0%% \n");
        printf("[Other] Exit \n");
        u8Option = getchar();
        printf("Select : %d \n", u8Option - 48);

        if (u8Option == '1')
        {
            u32NewDutyCycle = 100;
        }
        else if (u8Option == '2')
        {
            u32NewDutyCycle = 75;
        }
        else if (u8Option == '3')
        {
            u32NewDutyCycle = 25;
        }
        else if (u8Option == '4')
        {
            u32NewDutyCycle = 0;
        }
        else
        {
            printf("Exit\n");
            break;
        }

        /* Get new comparator value by call CalNewDutyCMR() */
        u32NewCMR = CalNewDutyCMR(BPWM0, 0, u32NewDutyCycle, 100);
        /* Set new comparator value to register */
        BPWM_SET_CMR(BPWM0, 0, u32NewCMR);
    }

    /* Stop BPWM counter */
    BPWM_Stop(BPWM0, BPWM_CH_0_MASK);
    /* Disable output of BPWM0 channel 0 */
    BPWM_DisableOutput(BPWM0, BPWM_CH_0_MASK);

    while (1);

}
