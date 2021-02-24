/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to set different core voltage level.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"


#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif


static volatile uint8_t s_u8IsINTEvent = 0;

void WDT_IRQHandler(void);
int32_t pi(void);
void CheckSystemWork(void);
void SYS_Init(void);
void UART4_Init(void);


/*---------------------------------------------------------------------------------------------------------*/
/*  WDT IRQ Handler                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void WDT_IRQHandler(void)
{

    if (WDT_GET_TIMEOUT_INT_FLAG())
    {
        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG();
    }

    s_u8IsINTEvent = 1;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Simple calculation test function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define PI_NUM  256
static int32_t s_ai32f[PI_NUM + 1];
static uint32_t s_au32piTbl[19] =
{
    3141,
    5926,
    5358,
    9793,
    2384,
    6264,
    3383,
    2795,
    288,
    4197,
    1693,
    9937,
    5105,
    8209,
    7494,
    4592,
    3078,
    1640,
    6284
};

static int32_t s_ai32piResult[19];

int32_t pi(void)
{
    int32_t i, i32Err;
    int32_t a = 10000, b = 0, c = PI_NUM, d = 0, e = 0, g = 0;

    for (; b - c;)
        s_ai32f[b++] = a / 5;

    i = 0;

    for (; (void)(d = 0), g = c * 2; c -= 14, s_ai32piResult[i++] = e + d / a, e = d % a)
    {
        if (i == 19)
            break;

        for (b = c; (void)(d += s_ai32f[b] * a), (void)(s_ai32f[b] = d % --g), (void)(d /= g--), --b; d *= b);
    }

    i32Err = 0;

    for (i = 0; i < 19; i++)
    {
        if (s_au32piTbl[i] != (uint32_t)s_ai32piResult[i])
            i32Err = -1;
    }

    return i32Err;
}

void CheckSystemWork(void)
{
    if (pi())
    {
        printf("Check system work [FAIL]\n\n");
    }
    else
    {
        printf("Check system work [OK]\n\n");
    }
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Wait for LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Enable LIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

    /* Wait for LIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable IP module clock */
    CLK_EnableModuleClock(UART4_MODULE);
    CLK_EnableModuleClock(WDT_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(UART4_MODULE, CLK_CLKSEL3_UART4SEL_LXT, CLK_CLKDIV4_UART4(1));
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDTSEL_LIRC, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    UartDebugMFP();

    /* Disable digital input path of analog pin X32_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, BIT4 | BIT5);
}

void UART4_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART4 */
    SYS_ResetModule(UART4_RST);

    /* Configure UART4 and set UART4 baud rate */
    UART_Open(UART4, 4800);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART4 for printf */
    UART4_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+---------------------------------------+\n");
    printf("|         Power Mode Sample Code        |\n");
    printf("+---------------------------------------+\n");

    /* Unlock protected registers before setting power level */
    SYS_UnlockReg();

    /* Set core clock as 32.7KHz from LXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_LXT, CLK_CLKDIV0_HCLK(1));

    /* Set power level to 3 */
    printf("Set power level to 3...\n");
    SYS_SetPowerLevel(SYS_PLCTL_PLSEL_PL3);

    /* Check system work */
    CheckSystemWork();

    /* Set power level to 0 */
    SYS_SetPowerLevel(SYS_PLCTL_PLSEL_PL0);
    printf("Set power level to 0...\n");

    /* Set core clock as 48MHz from HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Check system work */
    CheckSystemWork();

    /* Enter Idle Mode and wake-up by WDT */
    printf("Enter Idle mode and wake-up...\n");

    /* Enable WDT NVIC */
    NVIC_EnableIRQ(WDT_IRQn);

    /* Configure WDT settings and start WDT counting */
    WDT_Open(WDT_TIMEOUT_2POW14, WDT_RESET_DELAY_1026CLK, FALSE, TRUE);

    /* Enable WDT interrupt function */
    WDT_EnableInt();

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* Enter Idle mode */
    CLK_Idle();

    /* Check if WDT time-out interrupt and wake-up occurred or not */
    while (s_u8IsINTEvent == 0);

    /* Check system work */
    CheckSystemWork();

    printf("Sample code end.\n");

    while (1);

}
