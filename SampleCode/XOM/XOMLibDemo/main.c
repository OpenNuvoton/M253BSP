/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demo how to use XOM library
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
#include "xomlib.h"


/* XOM limitation : After return from XOM region function, need to delay one cycle */
/* The marco XOM_CALL is using for avoid XOM limitation. */
#define XOM_CALL(pfunc, ret, ...)      {ret = pfunc(__VA_ARGS__) ; __NOP();}


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

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    UartDebugMFP();
}


int32_t main(void)
{
    uint32_t u32Data;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init Debug UART */
    UartDebugInit();
    /*
        This sample code is used to show how to call XOM libary.

        The XOM libary is build by XOMLib project.
        User need to add include path of xomlib.h and add object file xomlib.o to using XOM library built by XOMLib project.
    */

    printf("\n\n");
    printf("+-------------------------------------------+\n");
    printf("|  Demo how to use XOM library Sample Code  |\n");
    printf("+-------------------------------------------+\n");


    XOM_CALL(XOM_Add, u32Data, 100, 200);
    printf(" 100 + 200 = %u\n", u32Data);

    XOM_CALL(XOM_Sub, u32Data, 500, 100);
    printf(" 500 - 100 = %u\n", u32Data);

    XOM_CALL(XOM_Mul, u32Data, 200, 100);
    printf(" 200 * 100 = %u\n", u32Data);

    XOM_CALL(XOM_Div, u32Data, 1000, 250);
    printf("1000 / 250 = %u\n", u32Data);

    while (1);
}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
