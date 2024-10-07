/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use CANFD Monitor mode to listen to CAN bus communication test.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "stdio.h"
#include "string.h"
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/


CANFD_FD_MSG_T   g_sRxFifo0MsgFrame[3];
CANFD_FD_MSG_T   g_sRxFifo1MsgFrame[3];
uint8_t g_u8RxFifo0RcvOk = 0;
uint8_t g_u8RxFifo1RcvOk = 0;
uint8_t g_u8RxFifo0MsgIndex = 0;
uint8_t g_u8RxFifo1MsgIndex = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void SYS_Init(void);
void CANFD_MonitorMode_Init(uint32_t u32NormBitRate, uint32_t u32DataBitRate);
uint32_t Get_CANFD_NominalBitRate(CANFD_T *psCanfd);
uint32_t Get_CANFD_DataBitRate(CANFD_T *psCanfd);
void CANFD_ShowMsg(CANFD_FD_MSG_T *sRxMsg);
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*---------------------------------------------------------------------------------------------------------*/
/*  ISR to handle CAN FD Line 0 interrupt event                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD0_IRQ0_IRQHandler(void)
{
    /*Rx FIFO 0 New Message Interrupt */
    if (CANFD0->IR & CANFD_IR_RF0N_Msk)
    {
        g_u8RxFifo0RcvOk = 1;
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_RF0N_Msk);
    }

    /*Rx FIFO 1 New Message Interrupt */
    if (CANFD0->IR & CANFD_IR_RF1N_Msk)
    {
        g_u8RxFifo1RcvOk = 1;
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_RF1N_Msk);
    }

    /*Rx FIFO 0 Message Lost Interrup*/
    if (CANFD0->IR & CANFD_IR_RF0L_Msk)
    {
        printf("Rx FIFO 0 Message Lost(Standard ID)\n");
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_RF0L_Msk);
    }

    /*Rx FIFO 1 Message Lost Interrup*/
    if (CANFD0->IR & CANFD_IR_RF1L_Msk)
    {
        printf("Rx FIFO 1 Message Lost(Extended ID)\n");
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_RF1L_Msk);
    }

}

/*---------------------------------------------------------------------------*/
/*  Show Message Function                                                    */
/*---------------------------------------------------------------------------*/
void CANFD_ShowMsg(CANFD_FD_MSG_T *sRxMsg)
{
    uint8_t u8Cnt;

    /* Show the message information */
    if (sRxMsg->eIdType == eCANFD_SID)
        printf("Rx buf 0: ID = 0x%08X(11-bit),DLC = %d\n", sRxMsg->u32Id, sRxMsg->u32DLC);
    else
        printf("Rx buf 1: ID = 0x%08X(29-bit),DLC = %d\n", sRxMsg->u32Id, sRxMsg->u32DLC);

    printf("Message Data : ");

    for (u8Cnt = 0; u8Cnt < sRxMsg->u32DLC; u8Cnt++)
    {
        printf("%02u ,", sRxMsg->au8Data[u8Cnt]);
    }

    printf("\n\n");
}

/*---------------------------------------------------------------------------*/
/*  Get the CANFD interface Nominal bit rate Function                        */
/*---------------------------------------------------------------------------*/
uint32_t Get_CANFD_NominalBitRate(CANFD_T *psCanfd)
{
    uint32_t u32BitRate = 0;
    uint32_t u32CanClk  = 0;
    uint32_t u32CanDiv  = 0;
    uint8_t  u8Tq = 0;
    uint8_t  u8NtSeg1 = 0;
    uint8_t  u8NtSeg2 = 0;

    if (CLK_GetModuleClockSource(CANFD0_MODULE) == (CLK_CLKSEL0_CANFD0SEL_HCLK >> CLK_CLKSEL0_CANFD0SEL_Pos))
        u32CanClk = CLK_GetHCLKFreq();
    else
        u32CanClk = CLK_GetHXTFreq();

    u32CanDiv = ((CLK->CLKDIV4 & CLK_CLKDIV4_CANFD0DIV_Msk) >> CLK_CLKDIV4_CANFD0DIV_Pos) + 1;
    u32CanClk = u32CanClk / u32CanDiv;
    u8Tq = ((psCanfd->NBTP & CANFD_NBTP_NBRP_Msk) >> CANFD_NBTP_NBRP_Pos) + 1 ;
    u8NtSeg1 = ((psCanfd->NBTP & CANFD_NBTP_NTSEG1_Msk) >> CANFD_NBTP_NTSEG1_Pos);
    u8NtSeg2 = ((psCanfd->NBTP & CANFD_NBTP_NTSEG2_Msk) >> CANFD_NBTP_NTSEG2_Pos);
    u32BitRate = u32CanClk / u8Tq / (u8NtSeg1 + u8NtSeg2 + 3);

    return u32BitRate;
}

/*---------------------------------------------------------------------------*/
/*  Get the CANFD interface Data bit rate Function                           */
/*---------------------------------------------------------------------------*/
uint32_t Get_CANFD_DataBitRate(CANFD_T *psCanfd)
{
    uint32_t u32BitRate = 0;
    uint32_t u32CanClk  = 0;
    uint32_t u32CanDiv  = 0;
    uint8_t  u8Tq = 0;
    uint8_t  u8NtSeg1 = 0;
    uint8_t  u8NtSeg2 = 0;

    if (CLK_GetModuleClockSource(CANFD0_MODULE) == (CLK_CLKSEL0_CANFD0SEL_HCLK >> CLK_CLKSEL0_CANFD0SEL_Pos))
        u32CanClk = CLK_GetHCLKFreq();
    else
        u32CanClk = CLK_GetHXTFreq();

    u32CanDiv = ((CLK->CLKDIV4 & CLK_CLKDIV4_CANFD0DIV_Msk) >> CLK_CLKDIV4_CANFD0DIV_Pos) + 1;
    u32CanClk = u32CanClk / u32CanDiv;
    u8Tq = ((psCanfd->DBTP & CANFD_DBTP_DBRP_Msk) >> CANFD_DBTP_DBRP_Pos) + 1 ;
    u8NtSeg1 = ((psCanfd->DBTP & CANFD_DBTP_DTSEG1_Msk) >> CANFD_DBTP_DTSEG1_Pos);
    u8NtSeg2 = ((psCanfd->DBTP & CANFD_DBTP_DTSEG2_Msk) >> CANFD_DBTP_DTSEG2_Pos);
    u32BitRate = u32CanClk / u8Tq / (u8NtSeg1 + u8NtSeg2 + 3);

    return u32BitRate;
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Select CAN FD0 clock source is HCLK */
    CLK_SetModuleClock(CANFD0_MODULE, CLK_CLKSEL0_CANFD0SEL_HCLK, CLK_CLKDIV4_CANFD0(1));
    /* Enable CAN FD0 peripheral clock */
    CLK_EnableModuleClock(CANFD0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Debug UART clock setting*/
    UartDebugCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    UartDebugMFP();

    /* Set PC multi-function pins for CAN RXD and TXD */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC4MFP_Msk)) | SYS_GPC_MFPL_PC4MFP_CAN0_RXD;
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC5MFP_Msk)) | SYS_GPC_MFPL_PC5MFP_CAN0_TXD;

    /* Lock protected registers */
    SYS_LockReg();
}


/*---------------------------------------------------------------------------------------------------------*/
/*                                    Init CAN                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_MonitorMode_Init(uint32_t u32NormBitRate, uint32_t u32DataBitRate)
{
    CANFD_FD_T sCANFD_Config;

    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_FD_MODE);
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = u32NormBitRate;
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = u32DataBitRate;
    CANFD_Open(CANFD0, &sCANFD_Config);
    printf("CANFD monitoring Nominal baud rate(bps): %d\n", Get_CANFD_NominalBitRate(CANFD0));
    printf("CANFD monitoring Data baud rate(bps): %d\n", Get_CANFD_DataBitRate(CANFD0));
    /*Enable the Bus Monitoring Mode */
    CANFD0->CCCR |= CANFD_CCCR_MON_Msk;

    /*Non-matching Frames with Extended ID and Standard ID are stored in Rx FIFO0 or Rx FIFO1,Reject all remote frames with 11-bit standard IDs and 29-bit extended IDs */
    CANFD_SetGFC(CANFD0, eCANFD_ACC_NON_MATCH_FRM_RX_FIFO0, eCANFD_ACC_NON_MATCH_FRM_RX_FIFO1, 1, 1);
    /* Enable RX FIFO New message, Message lost interrupt using interrupt line 0. */
    CANFD_EnableInt(CANFD0, (CANFD_IE_RF0NE_Msk | CANFD_IE_RF0LE_Msk | CANFD_IE_RF1NE_Msk | CANFD_IE_RF1LE_Msk), 0, 0, 0);
    /* CAN FD0 Run to Normal mode  */
    CANFD_RunToNormal(CANFD0, TRUE);
}




/*---------------------------------------------------------------------------------------------------------*/
/*                                         Main Function                                                   */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Lock protected registers */
    SYS_LockReg();

    /* Init Debug UART for printf */
    UartDebugInit();

    /*---------------------------------------------------------------------------------------------------------*/
    /*                                                 SAMPLE CODE                                             */
    /*---------------------------------------------------------------------------------------------------------*/
    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-----------------------------------------------------------------------------+\n");
    printf("|                      CANFD Monitor Mode sample code                         |\n");
    printf("+-----------------------------------------------------------------------------+\n");
    printf("|  Description :                                                              |\n");
    printf("|    The sample code needs three boards. Use CANFD Monitor mode to listen to  |\n");
    printf("|    CAN Bus communication test.The sample code must be set up on the node of |\n");
    printf("|    CAN communication transmission. Users can use one of the sample codes    |\n");
    printf("|    ' CANFD_CANFD_TxRx ' or ' CANFD_CANFD_TxRxINT ' as the CAN communication |\n");
    printf("|    transmission network.                                                    |\n");
    printf("|    Note: Users need to confirm whether the transmission rate matches.       |\n");
    printf("+-----------------------------------------------------------------------------+\n\n");

    printf("+-----------------------------------------------------------------------------+\n");
    printf("|                              Pin Configure                                  |\n");
    printf("+-----------------------------------------------------------------------------+\n");
    printf("|                                                         CAN_TXD(Any board)  |\n");
    printf("|                                 CAN BUS                 CAN_RXD(Any board)  |\n");
    printf("|                                   ||    CAN_H   |-----------|               |\n");
    printf("|                                   || <--------->|           |<------        |\n");
    printf("|                                   ||            |    CAN    |CAN_TX         |\n");
    printf("|  CAN0_TXD(PC5)                    ||    CAN_L   |Transceiver|               |\n");
    printf("|  CAN0_RXD(PC4)                    || <--------->|           |------>        |\n");
    printf("|          |-----------|   CAN_H    ||            |           |CAN_RX         |\n");
    printf("|  ------> |           |<---------> ||            |-----------|               |\n");
    printf("|  CAN0_TX |   CAN     |            ||                                        |\n");
    printf("|          |Transceiver|            ||                    CAN_TXD(Any board)  |\n");
    printf("|  <------ |           |   CAN_L    ||                    CAN_RXD(Any board)  |\n");
    printf("|  CAN0_RX |           |<---------> ||    CAN_H   |-----------|               |\n");
    printf("|          |-----------|            || <--------->|           |<------        |\n");
    printf("|                                   ||            |    CAN    |CAN_TX         |\n");
    printf("|                                   ||    CAN_L   |Transceiver|               |\n");
    printf("|                                   || <--------->|           |------>        |\n");
    printf("|                                   ||            |           |CAN_RX         |\n");
    printf("|                                   ||            |-----------|               |\n");
    printf("|                                                                             |\n");
    printf("+-----------------------------------------------------------------------------+\n\n");

    /* CANFD interface initialization*/
    CANFD_MonitorMode_Init(1000000, 4000000);

    while (1)
    {
        if (g_u8RxFifo0RcvOk == 1)
        {
            if (g_u8RxFifo0MsgIndex > 2)
                g_u8RxFifo0MsgIndex = 0;

            /*Receive the Rx Fifo0 message(Standard ID) */
            CANFD_ReadRxFifoMsg(CANFD0, 0, &g_sRxFifo0MsgFrame[g_u8RxFifo0MsgIndex]);
            g_u8RxFifo0MsgIndex++;
            g_u8RxFifo0RcvOk = 0;
        }

        if (g_u8RxFifo0RcvOk == 0 && g_u8RxFifo0MsgIndex != 0)
        {
            CANFD_ShowMsg(&g_sRxFifo0MsgFrame[g_u8RxFifo0MsgIndex - 1]);
            g_u8RxFifo0MsgIndex--;
        }

        if (g_u8RxFifo1RcvOk == 1)
        {
            if (g_u8RxFifo1MsgIndex > 2)
                g_u8RxFifo1MsgIndex = 0;

            /*Receive the Rx Fifo0 message(Extended ID) */
            CANFD_ReadRxFifoMsg(CANFD0, 1, &g_sRxFifo1MsgFrame[g_u8RxFifo1MsgIndex]);
            g_u8RxFifo1MsgIndex++;
            g_u8RxFifo1RcvOk = 0;
        }

        if (g_u8RxFifo1RcvOk == 0 && g_u8RxFifo1MsgIndex != 0)
        {
            CANFD_ShowMsg(&g_sRxFifo1MsgFrame[g_u8RxFifo1MsgIndex - 1]);
            g_u8RxFifo1MsgIndex--;
        }
    }
}
