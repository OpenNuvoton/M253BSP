/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Demonstrate how to compare average conversion result.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

// DAT gets unexpected data during conversion.
// If you want to avoid this issue, please use PDMA.
#define _USE_PDMA  1

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32IsTestOver = 0;
volatile int16_t  g_ai16ConversionData[1] = {0};

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void EADC_FunctionTest(void);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Debug UART clock setting */
    UartDebugCLK();

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC_MODULE);

    /* EADC clock source is PCLK1, set divider to 8, ADC clock is PCLK1/8 MHz */
    /* Note: The EADC_CLK speed should meet datasheet spec (<16MHz) and rules in following table.   */
    /* +--------------+------------------+                                                          */
    /* | PCLK divider | EADC_CLK divider |                                                          */
    /* +--------------+------------------+                                                          */
    /* | 1            | 1, 2, 3, 4, ...  |                                                          */
    /* +--------------+------------------+                                                          */
    /* | 2, 4, 8, 16  | 2, 4, 6, 8, ...  |                                                          */
    /* +--------------+------------------+                                                          */
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(8));
#if(_USE_PDMA==1)
    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA_MODULE);
#endif
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    UartDebugMFP();

    /* Set PB.0 and PB.1 to input mode */
    PB->MODE &= ~(GPIO_MODE_MODE0_Msk | GPIO_MODE_MODE1_Msk);

    /* Configure the EADC analog input pins.  */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB0MFP_Msk) | SYS_GPB_MFPL_PB0MFP_EADC0_CH0;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB1MFP_Msk) | SYS_GPB_MFPL_PB1MFP_EADC0_CH1;

    /* Disable the digital input path to avoid the leakage current for EADC analog input pins. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT0 | BIT1);  /* Disable PB0 and PB1 */

    /* Set PA multi-function pins for BPWM Channe0 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA0MFP_Msk));
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA0MFP_BPWM0_CH0;
}

#if(_USE_PDMA==1)
void PDMA_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init PDMA                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset PDMA module */
    SYS_ResetModule(PDMA_RST);

    /* Configure PDMA peripheral mode form EADC to memory */
    /* Open Channel 2 */
    PDMA_Open(PDMA, BIT2);

    /* transfer width is half word(16 bit) and transfer count is 1 */
    PDMA_SetTransferCnt(PDMA, 2, PDMA_WIDTH_16, 1);

    /* Set source address as EADC data register(no increment) and destination address as g_ai16ConversionData array(increment) */
    PDMA_SetTransferAddr(PDMA, 2, (uint32_t) & (EADC->CURDAT), PDMA_SAR_FIX, (uint32_t)g_ai16ConversionData, PDMA_DAR_INC);

    /* Select PDMA request source as ADC RX */
    PDMA_SetTransferMode(PDMA, 2, PDMA_EADC_RX, FALSE, 0);

    /* Set PDMA as single request type for EADC */
    PDMA_SetBurstType(PDMA, 2, PDMA_REQ_SINGLE, PDMA_BURST_4);

    PDMA_EnableInt(PDMA, 2, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA_IRQn);
}


void ReloadPDMA(void)
{
    /* transfer width is half word(16 bit) and transfer count is 1 */
    PDMA_SetTransferCnt(PDMA, 2, PDMA_WIDTH_16, 1);

    /* Select PDMA request source as EADC RX */
    PDMA_SetTransferMode(PDMA, 2, PDMA_EADC_RX, FALSE, 0);
}

#endif

/*---------------------------------------------------------------------------------------------------------*/
/* EADC function test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest(void)
{
    int32_t  i32ConversionData, i32Target;
    uint32_t u32IntNum,  u32ModuleNum, u32ChannelNum, u32ModuleMask;

    u32IntNum = 0;
    u32ModuleNum = 1;   /* Use Sample Module 1 */

    u32ModuleMask = (BIT0 << u32ModuleNum);

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|             EADC Average with sofeware compare sample code           |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIn this test, software will get 1 conversion result from the specified channel.\n");

    /* Set the EADC and enable the A/D converter */
    EADC_Open(EADC, 0);

    while (1)
    {
        printf("Select test items:\n");
        printf("  [1] Basic EADC conversion (channel 0 only)\n");
        printf("  [2] Basic EADC conversion (channel 1 only)\n");
        printf("  Other keys: exit EADC test\n");

        uint8_t  u8Option;
        u8Option = getchar();

        if (u8Option == '1')
            u32ChannelNum = 0;
        else if (u8Option == '2')
            u32ChannelNum = 1;
        else
            break;  /* exit while loop */

        /* Configure the sample module for analog input channel and enable BPWM0 trigger source */
        EADC_ConfigSampleModule(EADC, u32ModuleNum, EADC_BPWM0TG_TRIGGER, u32ChannelNum);

        /* Set sample module external sampling time to 10 */
        EADC_SetExtendSampleTime(EADC, u32ModuleNum, 10);

        /* Enable Accumulate feature */
        EADC_ENABLE_ACU(EADC, u32ModuleNum, EADC_MCTL1_ACU_8);

        /* Enable Average feature */
        EADC_ENABLE_AVG(EADC, u32ModuleNum);

#if(_USE_PDMA==1)
        /* reload PDMA configuration for next transmission */
        ReloadPDMA();
        EADC_ENABLE_PDMA(EADC, u32ModuleNum);
#else
        uint32_t u32IntMask;

        u32IntMask = (BIT0 << u32IntNum);

        /* Clear the A/D ADINT0 interrupt flag for safe */
        EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);
        /* Enable the sample module interrupt.  */
        EADC_ENABLE_INT(EADC, u32IntMask);
        EADC_ENABLE_SAMPLE_MODULE_INT(EADC, u32IntNum, u32ModuleMask);
        NVIC_EnableIRQ(EADC_INT0_IRQn);
#endif
        /* Reset the interrupt indicator and trigger sample module to start A/D conversion */
        g_u32IsTestOver = 0;
        EADC_START_CONV(EADC, u32ModuleMask);

        /* Wait interrupt (g_u32IsTestOver will be set at IRQ_Handler function) */
        while (g_u32IsTestOver == 0);

        /* Disable Average feature */
        EADC_DISABLE_ACU(EADC, u32ModuleNum);

        /* Disable Accumulate feature */
        EADC_DISABLE_AVG(EADC, u32ModuleNum);
#if(_USE_PDMA==1)
        EADC_DISABLE_PDMA(EADC, u32IntNum);
        i32ConversionData = g_ai16ConversionData[0];
#else
        /* Disable the ADINTx interrupt */
        EADC_DISABLE_INT(EADC, u32IntMask);
        EADC_DISABLE_SAMPLE_MODULE_INT(EADC, u32IntNum, u32ModuleMask);
        NVIC_DisableIRQ(EADC_INT0_IRQn);
        /* Get the conversion result of the sample module */
        i32ConversionData = EADC_GET_CONV_DATA(EADC, u32ModuleNum);
#endif
        /* EADC oversampling mode or averaging mode can not be monitored by compare and window compare functions */
        /* Implement software compare to replace hardware compare */
        printf("Conversion result of channel %u: 0x%X (%d)\n", u32ChannelNum, i32ConversionData, i32ConversionData);
        i32Target = 0x600;

        if (i32ConversionData >= i32Target)
        {
            printf("The conversion result of channel %u is >= 0x%03X\n", u32ChannelNum, i32Target);
        }
        else
        {
            printf("The conversion result of channel %u is < 0x%03X\n", u32ChannelNum, i32Target);
        }

    }   /* End of while(1) */

    /* Disable the A/D converter */
    EADC_Close(EADC);
}

#if(_USE_PDMA==1)
/*---------------------------------------------------------------------------------------------------------*/
/* PDMA interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);

    if (status & 0x1)   /* abort */
    {
        if (PDMA_GET_ABORT_STS(PDMA) & BIT2)
            g_u32IsTestOver = 2;

        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_ABTSTS_ABTIF2_Msk);
    }
    else if (status & 0x2)     /* done */
    {
        if (PDMA_GET_TD_STS(PDMA) & BIT2)
            g_u32IsTestOver = 1;

        PDMA_CLR_TD_FLAG(PDMA, PDMA_ABTSTS_ABTIF2_Msk);
    }
    else
        printf("PDMA_IRQHandler(): unknown interrupt !!\n");
}

#else
/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_INT0_IRQHandler(void)
{
    g_u32IsTestOver = 1;
    /* Clear the A/D ADINT0 interrupt flag */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);
}

#endif

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
#if(_USE_PDMA==1)
    /* Init PDMA for EADC */
    PDMA_Init();
#endif
    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\nSystem clock rate: %u Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

    /* Reset EADC module */
    SYS_ResetModule(EADC_RST);
#if(_USE_PDMA==1)
    /* Disable PDMA clock source */
    CLK_DisableModuleClock(PDMA_MODULE);
#endif
    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC_MODULE);
#if(_USE_PDMA==1)
    /* Disable PDMA Interrupt */
    NVIC_DisableIRQ(PDMA_IRQn);
#endif
    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC_INT0_IRQn);

    printf("Exit EADC sample code\n");

    while (1);
}
