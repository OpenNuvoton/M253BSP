/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Use PDMA to implement Ping-Pong buffer
 *           by scatter-gather mode(memory to memory).
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PDMA_TEST_COUNT  50

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t g_au32SrcArray0[1] = {0x55555555};
uint32_t g_au32SrcArray1[1] = {0xAAAAAAAA};
uint32_t g_au32DestArray[1];
uint32_t volatile g_u32IsTestOver = 0;
uint32_t volatile g_u32TransferredCount = 0;
uint32_t g_u32DMAConfig = 0;

typedef struct dma_desc_t
{
    uint32_t u32Ctl;
    uint32_t u32Src;
    uint32_t u32Dest;
    uint32_t u32Offset;
} DMA_DESC_T;

DMA_DESC_T DMA_DESC[2];

/**
 * @brief       DMA IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The DMA default IRQ, declared in startup_M253.s.
 */
void PDMA_IRQHandler(void)
{
    /* Check channel transfer done status */
    if (PDMA_GET_TD_STS(PDMA) == PDMA_TDSTS_TDIF4_Msk)
    {
        /* When finished a descriptor table then g_u32TransferredCount increases 1 */
        g_u32TransferredCount++;

        /* Check if PDMA has finished PDMA_TEST_COUNT tasks */
        if (g_u32TransferredCount >= PDMA_TEST_COUNT)
        {
            /* Set PDMA into idle state by Descriptor table */
            DMA_DESC[0].u32Ctl &= ~PDMA_DSCT_CTL_OPMODE_Msk;
            DMA_DESC[1].u32Ctl &= ~PDMA_DSCT_CTL_OPMODE_Msk;
            g_u32IsTestOver = 1;
        }

        /* Clear transfer done flag of channel 4 */
        PDMA_CLR_TD_FLAG(PDMA, PDMA_TDSTS_TDIF4_Msk);
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 48 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable PDMA module clock */
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Debug UART clock setting */
    UartDebugCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    UartDebugMFP();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    /* If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register. */
    SYS_LockReg();

    /* Init UART for printf */
    UartDebugInit();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-----------------------------------------------------------------------+ \n");
    printf("|    M253 PDMA Driver Ping-Pong Buffer Sample Code (Scatter-gather)     | \n");
    printf("+-----------------------------------------------------------------------+ \n");

    /* This sample will transfer data by looped around two descriptor tables from two different source to the same destination buffer in sequence.
       And operation sequence will be table 1 -> table 2-> table 1 -> table 2 -> table 1 -> ... -> until PDMA configuration doesn't be reloaded. */

    /*--------------------------------------------------------------------------------------------------
      PDMA transfer configuration:

        Channel = 4
        Operation mode = scatter-gather mode
        First scatter-gather descriptor table = DMA_DESC[0]
        Request source = PDMA_MEM(memory to memory)

        Transmission flow:

                                            loop around
                                      PDMA_TEST_COUNT/2 times
           ------------------------                             -----------------------
          |                        | ------------------------> |                       |
          |  DMA_DESC[0]           |                           |  DMA_DESC[1]          |
          |  (Descriptor table 1)  |                           |  (Descriptor table 2) |
          |                        | <-----------------------  |                       |
           ------------------------                             -----------------------

        Note: The configuration of each table in SRAM need to be reloaded after transmission finished.
    --------------------------------------------------------------------------------------------------*/

    /* Open Channel 4 */
    PDMA_Open(PDMA, 1 << 4);

    /* Enable Scatter Gather mode, assign the first scatter-gather descriptor table is table 1,
       and set transfer mode as memory to memory */
    PDMA_SetTransferMode(PDMA, 4, PDMA_MEM, TRUE, (uint32_t)&DMA_DESC[0]);


    /* Scatter-Gather descriptor table configuration in SRAM */
    g_u32DMAConfig = \
                     (0 << PDMA_DSCT_CTL_TXCNT_Pos) | /* Transfer count is 1 */ \
                     PDMA_WIDTH_32 |  /* Transfer width is 32 bits(one word) */ \
                     PDMA_SAR_FIX |   /* Source increment size is fixed(no increment) */ \
                     PDMA_DAR_FIX |   /* Destination increment size is fixed(no increment) */ \
                     PDMA_REQ_BURST | /* Transfer type is burst transfer type */ \
                     PDMA_BURST_1 |   /* Burst size is 128. No effect in single transfer type */ \
                     PDMA_OP_SCATTER; /* Operation mode is scatter-gather mode */
    /*-----------------------------------------------------------------------------------------------------------------------------------------------------------
       Note: PDMA_REQ_BURST is only supported in memory-to-memory transfer mode.
             PDMA transfer type should be set as PDMA_REQ_SINGLE in memory-to-peripheral and peripheral-to-memory transfer mode,
             then above code will be modified as follows:
             g_u32DMAConfig = (0 << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_FIX | PDMA_BURST_1 | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    -----------------------------------------------------------------------------------------------------------------------------------------------------------*/

    /*------------------------------------------------------------------------------------------------------
      Descriptor table 1 configuration:

             g_au32SrcArray0               transfer 1 times    g_au32DestArray
             ---------------------------   ----------------->  ---------------------------
            |            [0]            |                     |            [0]            |
             ---------------------------                       ---------------------------
             \                         /                       \                         /
                   32bits(one word)                                  32bits(one word)

        Operation mode = scatter-gather mode
        Next descriptor table = DMA_DESC[1](Descriptor table 2)
        transfer done and table empty interrupt = enable

        Transfer count = 1
        Transfer width = 32 bits(one word)
        Source address = g_au32SrcArray0
        Source address increment size = fixed address(no increment)
        Destination address = au8DestArray0
        Destination address increment size = fixed address(no increment)
        Transfer type = burst transfer

        Total transfer length = 1 * 32 bits
    ------------------------------------------------------------------------------------------------------*/
    DMA_DESC[0].u32Ctl = g_u32DMAConfig;
    /* Configure source address */
    DMA_DESC[0].u32Src = (uint32_t)g_au32SrcArray0; /* Ping-Pong buffer 1 */
    /* Configure destination address */
    DMA_DESC[0].u32Dest = (uint32_t)&g_au32DestArray[0];
    /* Configure next descriptor table address */
    DMA_DESC[0].u32Offset = (uint32_t)&DMA_DESC[1] - (PDMA->SCATBA); /* next operation table is table 2 */

    /*------------------------------------------------------------------------------------------------------
      Descriptor table 2 configuration:

             g_au32SrcArray1               transfer 1 times    g_au32DestArray
             ---------------------------   ----------------->  ---------------------------
            |            [0]            |                     |            [0]            |
             ---------------------------                       ---------------------------
             \                         /                       \                         /
                   32bits(one word)                                  32bits(one word)

        Operation mode = scatter-gather mode
        Next descriptor table = DMA_DESC[0](Descriptor table 1)
        transfer done and table empty interrupt = enable

        Transfer count = 1
        Transfer width = 32 bits(one word)
        Source address = g_au32SrcArray1
        Source address increment size = fixed address(no increment)
        Destination address = au8DestArray0
        Destination address increment size = fixed address(no increment)
        Transfer type = burst transfer

        Total transfer length = 1 * 32 bits
    ------------------------------------------------------------------------------------------------------*/
    DMA_DESC[1].u32Ctl = g_u32DMAConfig;
    /* Configure source address */
    DMA_DESC[1].u32Src = (uint32_t)g_au32SrcArray1; /* Ping-Pong buffer 2 */
    /* Configure destination address */
    DMA_DESC[1].u32Dest = (uint32_t)&g_au32DestArray[0];
    /* Configure next descriptor table address */
    DMA_DESC[1].u32Offset = (uint32_t)&DMA_DESC[0] - (PDMA->SCATBA); /* next operation table is table 1 */


    /* Enable transfer done interrupt */
    PDMA_EnableInt(PDMA, 4, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA_IRQn);
    g_u32IsTestOver = 0;

    /* Start PDMA operation */
    PDMA_Trigger(PDMA, 4);

    while (1)
    {
        if (g_u32IsTestOver == 1)
        {
            g_u32IsTestOver = 0;
            printf("test done...\n");

            /* Close PDMA channel */
            PDMA_Close(PDMA);
        }
    }
}
