/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Show how an I2C 7-bit address slave receives data from master.
 *           This sample code needs to work with USCI_I2C_Master.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define PLL_CLOCK   FREQ_48MHZ

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_au8SlvData[256];
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_au8SlvRxData[4];
volatile uint16_t g_u16RecvAddr;

volatile uint32_t g_u32SlaveBuffAddr;
volatile uint16_t g_u16SlvRcvAddr;
volatile uint8_t g_u8SlvDataLen;

volatile enum UI2C_SLAVE_EVENT g_eSEvent;

typedef void (*UI2C_FUNC)(uint32_t u32Status);

volatile static UI2C_FUNC s_UI2C0HandlerFn = NULL;
/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C IRQ Handler                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void USCI0_IRQHandler(void)
{
    uint32_t u32Status;

    //UI2C0 Interrupt
    u32Status = UI2C_GET_PROT_STATUS(UI2C0);

    if (s_UI2C0HandlerFn != NULL)
    {
        s_UI2C0HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C0 TRx Callback Function                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void UI2C_SlaveTRx(uint32_t u32Status)
{
    uint8_t u8Data;
    uint32_t u32TempAddr;

    if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)                   /* Re-Start been received */
    {
        /* Clear START INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);

        /* Event process */
        g_eSEvent = SLAVE_ADDRESS_ACK;

        /* Trigger USCI I2C */
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)                /* USCI I2C Bus have been received ACK */
    {
        /* Clear ACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);

        /* Event process */
        if (g_eSEvent == SLAVE_ADDRESS_ACK)                                                   /* Address Data has been received */
        {
            /* Check address if match address 0 or address 1*/
            if ((UI2C0->ADMAT & UI2C_ADMAT_ADMAT0_Msk) == UI2C_ADMAT_ADMAT0_Msk)
            {
                /* Address 0 match */
                UI2C0->ADMAT = UI2C_ADMAT_ADMAT0_Msk;
            }
            else if ((UI2C0->ADMAT & UI2C_ADMAT_ADMAT1_Msk) == UI2C_ADMAT_ADMAT1_Msk)
            {
                /* Address 1 match */
                UI2C0->ADMAT = UI2C_ADMAT_ADMAT1_Msk;
            }
            else
            {
                printf("No Address Match!!!\n");

                while (1);
            }

            /* USCI I2C receives Slave command type */
            if ((UI2C0->PROTSTS & UI2C_PROTSTS_SLAREAD_Msk) == UI2C_PROTSTS_SLAREAD_Msk)
            {
                g_eSEvent = SLAVE_SEND_DATA;                                                  /* Slave address read has been received */
                UI2C_SET_DATA(UI2C0, g_au8SlvData[g_u32SlaveBuffAddr]);
                g_u32SlaveBuffAddr++;
            }
            else
            {
                g_u8SlvDataLen = 0;                                                         /* Slave address write has been received */
                g_eSEvent = SLAVE_GET_DATA;
            }

            /* Read address from USCI I2C RXDAT*/
            g_u16RecvAddr = (uint8_t)UI2C_GET_DATA(UI2C0);
        }
        else if (g_eSEvent == SLAVE_GET_DATA)
        {
            /* Read data from USCI I2C RXDAT*/
            u8Data = (uint8_t)UI2C_GET_DATA(UI2C0);

            if (g_u8SlvDataLen < 2)
            {
                g_au8SlvRxData[g_u8SlvDataLen++] = u8Data;
                u32TempAddr = g_au8SlvRxData[1];
                u32TempAddr = (g_au8SlvRxData[0] << 8) + u32TempAddr;
                g_u32SlaveBuffAddr = u32TempAddr;
            }
            else
            {
                g_au8SlvData[g_u32SlaveBuffAddr++] = u8Data;

                if (g_u32SlaveBuffAddr == 256)
                {
                    g_u32SlaveBuffAddr = 0;
                }
            }
        }
        else if (g_eSEvent == SLAVE_SEND_DATA)
        {
            /* Write transmit data to USCI I2C TXDAT*/
            UI2C0->TXDAT = g_au8SlvData[g_u32SlaveBuffAddr];
            g_u32SlaveBuffAddr++;

            if (g_u32SlaveBuffAddr > 256)
            {
                g_u32SlaveBuffAddr = 0;
            }
        }

        /* Trigger USCI I2C */
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        /* Clear NACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);

        /* Event process */
        g_u8SlvDataLen = 0;
        g_eSEvent = SLAVE_ADDRESS_ACK;

        /* Trigger USCI I2C */
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        /* Clear STOP INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STORIF_Msk);

        g_u8SlvDataLen = 0;
        g_eSEvent = SLAVE_ADDRESS_ACK;

        /* Trigger USCI I2C */
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 48MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable peripheral clock */
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPB_MODULE);

    /* Debug UART clock setting */
    UartDebugCLK();

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* using semihost to show message so we don't set multi-function pins for uart*/
    UartDebugMFP();

    /* Set UI2C0 multi-function pins */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_USCI0_CLK | SYS_GPB_MFPH_PB13MFP_USCI0_DAT0);

    /* I2C pins enable schmitt trigger */
    PB->SMTEN |= GPIO_SMTEN_SMTEN12_Msk | GPIO_SMTEN_SMTEN13_Msk;
}

void UI2C0_Init(uint32_t u32ClkSpeed)
{
    /* Open USCI_I2C0 and set clock to 100k */
    UI2C_Open(UI2C0, u32ClkSpeed);

    /* Get USCI_I2C0 Bus Clock */
    printf("UI2C0 clock %d Hz\n", UI2C_GetBusClockFreq(UI2C0));

    /* Set USCI_I2C0 Slave Addresses */
    UI2C_SetSlaveAddr(UI2C0, 0, 0x15, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x15 */
    UI2C_SetSlaveAddr(UI2C0, 1, 0x35, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x35 */

    /* Set USCI_I2C0 Slave Addresses Mask */
    UI2C_SetSlaveAddrMask(UI2C0, 0, 0x01);                    /* Slave Address : 0x1 */
    UI2C_SetSlaveAddrMask(UI2C0, 1, 0x04);                    /* Slave Address : 0x4 */

    /* Enable UI2C0 protocol interrupt */
    UI2C_ENABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk));
    NVIC_EnableIRQ(USCI0_IRQn);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32Idx;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART for message printing */
    UartDebugInit();

    /*
        This sample code sets USCI_I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("+--------------------------------------------------------+\n");
    printf("|  USCI_I2C Driver Sample Code for Master access 7-bit   |\n");
    printf("|  address Slave. This sample code needs work with       |\n");
    printf("|  USCI_I2C_Master sample code                           |\n");
    printf("|         UI2C0(Master)  <----> UI2C0(Slave)             |\n");
    printf("|  This sample code requires two boards for testing      |\n");
    printf("+--------------------------------------------------------+\n");

    printf("\n");
    printf("Configure UI2C0 as Slave.\n");
    printf("The I/O connection for UI2C0:\n");
    printf("UI2C0_SDA(PB13), UI2C0_SCL(PB12)\n");

    /* Init USCI_I2C0 */
    UI2C0_Init(100000);

    g_eSEvent = SLAVE_ADDRESS_ACK;

    UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));

    for (u32Idx = 0; u32Idx < 0x100; u32Idx++)
    {
        g_au8SlvData[u32Idx] = 0;
    }

    /* I2C function to Slave receive/transmit data */
    s_UI2C0HandlerFn = UI2C_SlaveTRx;

    while (1);
}
