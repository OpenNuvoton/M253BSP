/**************************************************************************//**
 * @file     main.c
 * @version  V0.11
 * @brief    Implement CRC in CRC-CCITT mode and get the CRC checksum result.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable peripheral clock*/
    CLK_EnableModuleClock(CRC_MODULE);

    /* Debug UART clock setting */
    UartDebugCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    UartDebugMFP();
    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    const uint8_t acCRCSrcPattern[] = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38};
    uint32_t i, u32TargetChecksum = 0xA12B, u32CalChecksum = 0;
    uint16_t *p16SrcAddr;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART */
    UartDebugInit();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+---------------------------------------------+\n");
    printf("|    CRC-CCITT Polynomial Mode Sample Code    |\n");
    printf("+---------------------------------------------+\n\n");

    printf("# Calculate [0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38] CRC-CCITT checksum value.\n");
    printf("    - Seed value is 0xFFFF             \n");
    printf("    - CPU write data length is 16-bit \n");
    printf("    - Checksum complement disable    \n");
    printf("    - Checksum reverse disable       \n");
    printf("    - Write data complement disable  \n");
    printf("    - Write data reverse disable     \n");
    printf("    - Checksum should be 0x%X        \n\n", u32TargetChecksum);

    /* Configure CRC controller for CRC-CCITT CPU mode */
    CRC_Open(CRC_CCITT, 0, 0xFFFF, CRC_CPU_WDATA_16);

    /* Convert 16-bit source data */
    p16SrcAddr = (uint16_t *)acCRCSrcPattern;

    /* Start to execute CRC-CCITT operation */
    for (i = 0; i < sizeof(acCRCSrcPattern) / 2; i++)
    {
        CRC_WRITE_DATA((p16SrcAddr[i] & 0xFFFF));
    }

    /* Get CRC-CCITT checksum value */
    u32CalChecksum = CRC_GetChecksum();
    printf("CRC checksum is 0x%X ... %s.\n", u32CalChecksum, (u32CalChecksum == u32TargetChecksum) ? "PASS" : "FAIL");

    /* Disable CRC function */
    CLK_DisableModuleClock(CRC_MODULE);

    while (1);
}
