/***************************************************************************//**
 * @file     main.c
 * @brief    ISP tool main function
 * @version  0x32
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define GPIO_SETMODE(port, pin, u32Mode) ((port)->MODE = ((port)->MODE & ~(0x3ul << ((pin) << 1))) | ((u32Mode) << ((pin) << 1)))

#define CAN_BAUD_RATE                     500000
#define Master_ISP_ID                     0x487
#define Device0_ISP_ID                    0x784
#define CAN_ISP_DtatLength                0x08
#define CAN_RETRY_COUNTS                  0x1fffffff

#define CMD_READ_CONFIG                   0xA2000000
#define CMD_RUN_APROM                     0xAB000000
#define CMD_GET_DEVICEID                  0xB1000000

/*---------------------------------------------------------------------------*/
/*  Function Declare                                                         */
/*---------------------------------------------------------------------------*/
/* Declare a CAN message structure */
typedef struct
{
    uint32_t  Address;
    uint32_t  Data;
} STR_CANMSG_ISP_T;

static CANFD_FD_MSG_T g_sRxMsgFrame;
static volatile uint8_t s_u8CANPackageFlag = 0, s_u8CANAckFlag = 0;

void CANFD0_IRQ0_IRQHandler(void);
void SYS_Init(void);
void CAN_Package_ACK(CANFD_T *psCanfd);
void CAN_Init(void);
void ProcessHardFault(void);
void SH_Return(void);
void SendChar_ToUART(int ch);
uint32_t CAN_Parsing_MSG(uint8_t *u8pMsg);
/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle CAN FD0 Line0 interrupt event                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD0_IRQ0_IRQHandler(void)
{
    uint32_t u32IIDRstatus;
    u32IIDRstatus = CANFD0->IR;

    /**************************/
    /* Status Change interrupt*/
    /**************************/
    if (u32IIDRstatus & CANFD_IR_RF0N_Msk)
    {
        /*Clear the Interrupt flag */
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_TOO_Msk | CANFD_IR_RF0N_Msk);
        CANFD_ReadRxFifoMsg(CANFD0, 0, &g_sRxMsgFrame);
        s_u8CANPackageFlag = 1;
    }
}

void SYS_Init(void)
{
    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Select CAN FD0 clock source is HCLK */
    CLK_SetModuleClock(CANFD0_MODULE, CLK_CLKSEL0_CANFD0SEL_HCLK, CLK_CLKDIV4_CANFD0(1));

    /* Update System Core Clock */
    SystemCoreClockUpdate();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Tx Msg by Normal Mode Function (With Message RAM)                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_Package_ACK(CANFD_T *psCanfd)
{
    CANFD_FD_MSG_T  sTxMsgFrame;
    s_u8CANAckFlag = 1;
    /* Send a 11-bit Standard Identifier message */
    sTxMsgFrame.eFrmType = eCANFD_DATA_FRM;
    sTxMsgFrame.eIdType  = eCANFD_SID;
    sTxMsgFrame.u32Id    = Device0_ISP_ID;
    sTxMsgFrame.u32DLC   = CAN_ISP_DtatLength;

    sTxMsgFrame.au32Data[0] = g_sRxMsgFrame.au32Data[0];
    sTxMsgFrame.au32Data[1] = g_sRxMsgFrame.au32Data[1];

    if (CANFD_TransmitTxMsg(psCanfd, 0, &sTxMsgFrame) == FALSE)    // Configure Msg RAM and send the Msg in the RAM
    {
        return;
    }

}

void CAN_Init(void)
{
    CANFD_FD_T sCANFD_Config;
    /* Enable CAN FD0 peripheral clock */
    CLK_EnableModuleClock(CANFD0_MODULE);

    /* Set PC multi-function pins for CAN RXD and TXD */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC4MFP_Msk)) | SYS_GPC_MFPL_PC4MFP_CAN0_RXD;
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC5MFP_Msk)) | SYS_GPC_MFPL_PC5MFP_CAN0_TXD;

    /* Set CAN transceiver to high speed mode */
    GPIO_SETMODE(PC, 11, GPIO_MODE_OUTPUT);
    PC11 = 0;

    /*Get the CAN configuration value*/
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_MODE);
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = CAN_BAUD_RATE;
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 0;

    /*Open the CAN feature*/
    CANFD_Open(CANFD0, &sCANFD_Config);

    /* Set CAN reveive message */
    CANFD_SetSIDFltr(CANFD0, 0, CANFD_RX_FIFO0_STD_MASK(Master_ISP_ID, 0x7FF));

    /* Enable Standard ID and  Extended ID Filter as RX FOFI0*/
    CANFD_SetGFC(CANFD0, eCANFD_ACC_NON_MATCH_FRM_RX_FIFO0, eCANFD_ACC_NON_MATCH_FRM_RX_FIFO0, 1, 1);
    /* Enable RX fifo0 new message interrupt using interrupt line 0. */
    CANFD_EnableInt(CANFD0, (CANFD_IE_TOOE_Msk | CANFD_IE_RF0NE_Msk | CANFD_IE_TCE_Msk), 0, 0, 0);

    /* CAN FD0 Run to Normal mode  */
    CANFD_RunToNormal(CANFD0, TRUE);

}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Enable FMC ISP AP CFG function & clear ISPFF */
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk | FMC_ISPCTL_CFGUEN_Msk | FMC_ISPCTL_ISPFF_Msk;

    /* Init CAN port */
    CAN_Init();

    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL   = (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    while (1)
    {
        if (s_u8CANPackageFlag == 1)
        {
            break;
        }

        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
        {
            goto lexit;
        }
    }

    /* state of update program */
    while (1)
    {
        if (s_u8CANPackageFlag)
        {
            STR_CANMSG_ISP_T *psISPCanMsg = (STR_CANMSG_ISP_T *)&g_sRxMsgFrame.au32Data[0];
            s_u8CANPackageFlag = 0;

            if (psISPCanMsg->Address == CMD_GET_DEVICEID)
            {
                psISPCanMsg->Data = SYS->PDID;
            }
            else if (psISPCanMsg->Address == CMD_READ_CONFIG)
            {
                psISPCanMsg->Data = FMC_Read(psISPCanMsg->Data);
            }
            else if (psISPCanMsg->Address == CMD_RUN_APROM)
            {
                break;
            }
            else
            {
                if ((psISPCanMsg->Address % FMC_FLASH_PAGE_SIZE) == 0)
                {
                    FMC_Erase(psISPCanMsg->Address);
                }

                FMC_Write(psISPCanMsg->Address, psISPCanMsg->Data);
                psISPCanMsg->Data = FMC_Read(psISPCanMsg->Address);
            }

            /* send CAN FD ISP Package (ACK) */
            CAN_Package_ACK(CANFD0);
        }
    }

lexit:

    /* Trap the CPU */
    for (;;)
    {
        __WFI();
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Empty functions for reduce code size to fit  into LDROM & solve the functions are not be defined.      */
/*---------------------------------------------------------------------------------------------------------*/
void ProcessHardFault()
{}

void SH_Return()
{}

void SendChar_ToUART(int ch)
{
    (void)ch;
}
