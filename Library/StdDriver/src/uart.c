/****************************************************************************
 * @file     uart.c
 * @version  V1.00
 * @brief    M253 series UART driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup UART_Driver UART Driver
  @{
*/

/** @addtogroup UART_EXPORTED_FUNCTIONS UART Exported Functions
  @{
*/

/**
 *    @brief        Clear UART specified interrupt flag
 *
 *    @param[in]    uart                The pointer of the specified UART module.
 *    @param[in]    u32InterruptFlag    The specified interrupt of UART module.
 *                                      - \ref UART_INTSTS_SWBEINT_Msk   : Single-wire Bit Error Detect Interrupt
 *                                      - \ref UART_INTSTS_LININT_Msk    : LIN bus interrupt
 *                                      - \ref UART_INTEN_WKIEN_Msk      : Wake-up interrupt
 *                                      - \ref UART_INTSTS_BUFERRINT_Msk : Buffer Error interrupt
 *                                      - \ref UART_INTSTS_MODEMINT_Msk  : Modem Status interrupt
 *                                      - \ref UART_INTSTS_RLSINT_Msk    : Receive Line Status interrupt
 *
 *
 *    @details      The function is used to clear UART specified interrupt flag.
 */

void UART_ClearIntFlag(UART_T *uart, uint32_t u32InterruptFlag)
{


    if (u32InterruptFlag & UART_INTSTS_SWBEINT_Msk)   /* Clear Bit Error Detection Interrupt */
    {
        uart->FIFOSTS = UART_INTSTS_SWBEIF_Msk;
    }

    if (u32InterruptFlag & UART_INTSTS_RLSINT_Msk)   /* Clear Receive Line Status Interrupt */
    {
        uart->FIFOSTS = UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_PEF_Msk |
                        UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_ADDRDETF_Msk;
    }

    if (u32InterruptFlag & UART_INTSTS_MODEMINT_Msk)   /* Clear Modem Status Interrupt */
    {
        uart->MODEMSTS |= UART_MODEMSTS_CTSDETF_Msk;
    }

    if (u32InterruptFlag & UART_INTSTS_BUFERRINT_Msk)   /* Clear Buffer Error Interrupt */
    {
        uart->FIFOSTS = UART_FIFOSTS_RXOVIF_Msk | UART_FIFOSTS_TXOVIF_Msk;
    }

    if (u32InterruptFlag & UART_INTSTS_WKINT_Msk)   /* Clear Wake-up Interrupt */
    {
        uart->WKSTS = UART_WKSTS_CTSWKF_Msk  | UART_WKSTS_DATWKF_Msk  |
                      UART_WKSTS_RFRTWKF_Msk | UART_WKSTS_RS485WKF_Msk |
                      UART_WKSTS_TOUTWKF_Msk;
    }

    if (u32InterruptFlag & UART_INTSTS_LININT_Msk)   /* Clear LIN Bus Interrupt */
    {
        uart->INTSTS = UART_INTSTS_LINIF_Msk;
        uart->LINSTS = UART_LINSTS_BITEF_Msk    | UART_LINSTS_BRKDETF_Msk  |
                       UART_LINSTS_SLVSYNCF_Msk | UART_LINSTS_SLVIDPEF_Msk |
                       UART_LINSTS_SLVHEF_Msk   | UART_LINSTS_SLVHDETF_Msk ;
    }
}


/**
 *  @brief      Disable UART interrupt
 *
 *  @param[in]  uart The pointer of the specified UART module.
 *
 *
 *  @details    The function is used to disable UART interrupt.
 */
void UART_Close(UART_T *uart)
{
    uart->INTEN = 0ul;
}


/**
 *  @brief      Disable UART auto flow control function
 *
 *  @param[in]  uart The pointer of the specified UART module.
 *
 *
 *  @details    The function is used to disable UART auto flow control.
 */
void UART_DisableFlowCtrl(UART_T *uart)
{
    uart->INTEN &= ~(UART_INTEN_ATORTSEN_Msk | UART_INTEN_ATOCTSEN_Msk);
}


/**
 *    @brief        Disable UART specified interrupt
 *
 *    @param[in]    uart                The pointer of the specified UART module.
 *    @param[in]    u32InterruptFlag    The specified interrupt of UART module.
 *                                      - \ref UART_INTSTS_SWBEINT_Msk   : Single-wire Bit Error Detect Interrupt
 *                                      - \ref UART_INTEN_WKIEN_Msk      : Wake-up interrupt
 *                                      - \ref UART_INTEN_LINIEN_Msk     : Lin bus interrupt
 *                                      - \ref UART_INTEN_BUFERRIEN_Msk  : Buffer Error interrupt
 *                                      - \ref UART_INTEN_RXTOIEN_Msk    : Rx time-out interrupt
 *                                      - \ref UART_INTEN_MODEMIEN_Msk   : Modem status interrupt
 *                                      - \ref UART_INTEN_RLSIEN_Msk     : Receive Line status interrupt
 *                                      - \ref UART_INTEN_THREIEN_Msk    : Tx empty interrupt
 *                                      - \ref UART_INTEN_RDAIEN_Msk     : Rx ready interrupt
 *
 *
 *    @details      The function is used to disable UART specified interrupt.
 */
void UART_DisableInt(UART_T  *uart, uint32_t u32InterruptFlag)
{
    /* Disable UART specified interrupt */
    UART_DISABLE_INT(uart, u32InterruptFlag);
}


/**
 *    @brief        Enable UART auto flow control function
 *
 *    @param[in]    uart    The pointer of the specified UART module.
 *
 *
 *    @details      The function is used to Enable UART auto flow control.
 */
void UART_EnableFlowCtrl(UART_T *uart)
{
    /* Set RTS pin output is low level active */
    uart->MODEM |= UART_MODEM_RTSACTLV_Msk;

    /* Set CTS pin input is low level active */
    uart->MODEMSTS |= UART_MODEMSTS_CTSACTLV_Msk;

    /* Set RTS and CTS auto flow control enable */
    uart->INTEN |= UART_INTEN_ATORTSEN_Msk | UART_INTEN_ATOCTSEN_Msk;
}


/**
 *    @brief        Enable UART specified interrupt.
 *
 *    @param[in]    uart                The pointer of the specified UART module.
 *    @param[in]    u32InterruptFlag    The specified interrupt of UART module:
 *                                      - \ref UART_INTSTS_SWBEINT_Msk   : Single-wire Bit Error Detect Interrupt
 *                                      - \ref UART_INTEN_WKIEN_Msk      : Wake-up interrupt
 *                                      - \ref UART_INTEN_LINIEN_Msk     : Lin bus interrupt
 *                                      - \ref UART_INTEN_BUFERRIEN_Msk  : Buffer Error interrupt
 *                                      - \ref UART_INTEN_RXTOIEN_Msk    : Rx time-out interrupt
 *                                      - \ref UART_INTEN_MODEMIEN_Msk   : Modem status interrupt
 *                                      - \ref UART_INTEN_RLSIEN_Msk     : Receive Line status interrupt
 *                                      - \ref UART_INTEN_THREIEN_Msk    : Tx empty interrupt
 *                                      - \ref UART_INTEN_RDAIEN_Msk     : Rx ready interrupt
 *
 *
 *    @details      The function is used to enable UART specified interrupt.
 */
void UART_EnableInt(UART_T  *uart, uint32_t u32InterruptFlag)
{
    /* Enable UART specified interrupt */
    UART_ENABLE_INT(uart, u32InterruptFlag);
}


/**
 *    @brief        Open and set UART function
 *
 *    @param[in]    uart            The pointer of the specified UART module.
 *    @param[in]    u32BaudRate     The baud rate of UART module.
 *
 *
 *    @details      This function use to enable UART function and set baud-rate.
 */
void UART_Open(UART_T *uart, uint32_t u32BaudRate)
{
    uint32_t u32UartClkSrcSel = 0ul, u32UartClkDivNum = 0ul;
    uint32_t au32ClkTbl[6ul] = {__HXT, 0ul, __LXT, __HIRC, 0ul, __LIRC};



    if (uart == (UART_T *)UART0)
    {
        /* Get UART clock source selection */
        u32UartClkSrcSel = ((uint32_t)(CLK->CLKSEL1 & CLK_CLKSEL1_UART0SEL_Msk)) >> CLK_CLKSEL1_UART0SEL_Pos;
        /* Get UART clock divider number */
        u32UartClkDivNum = (CLK->CLKDIV0 & CLK_CLKDIV0_UART0DIV_Msk) >> CLK_CLKDIV0_UART0DIV_Pos;
    }
    else if (uart == (UART_T *)UART1)
    {
        /* Get UART clock source selection */
        u32UartClkSrcSel = (CLK->CLKSEL1 & CLK_CLKSEL1_UART1SEL_Msk) >> CLK_CLKSEL1_UART1SEL_Pos;
        /* Get UART clock divider number */
        u32UartClkDivNum = (CLK->CLKDIV0 & CLK_CLKDIV0_UART1DIV_Msk) >> CLK_CLKDIV0_UART1DIV_Pos;
    }
    else if (uart == (UART_T *)UART2)
    {
        /* Get UART clock source selection */
        u32UartClkSrcSel = (CLK->CLKSEL3 & CLK_CLKSEL3_UART2SEL_Msk) >> CLK_CLKSEL3_UART2SEL_Pos;
        /* Get UART clock divider number */
        u32UartClkDivNum = (CLK->CLKDIV4 & CLK_CLKDIV4_UART2DIV_Msk) >> CLK_CLKDIV4_UART2DIV_Pos;
    }
    else if (uart == (UART_T *)UART3)
    {
        /* Get UART clock source selection */
        u32UartClkSrcSel = (CLK->CLKSEL3 & CLK_CLKSEL3_UART3SEL_Msk) >> CLK_CLKSEL3_UART3SEL_Pos;
        /* Get UART clock divider number */
        u32UartClkDivNum = (CLK->CLKDIV4 & CLK_CLKDIV4_UART3DIV_Msk) >> CLK_CLKDIV4_UART3DIV_Pos;
    }
    else if (uart == (UART_T *)UART4)
    {
        /* Get UART clock source selection */
        u32UartClkSrcSel = (CLK->CLKSEL3 & CLK_CLKSEL3_UART4SEL_Msk) >> CLK_CLKSEL3_UART4SEL_Pos;
        /* Get UART clock divider number */
        u32UartClkDivNum = (CLK->CLKDIV4 & CLK_CLKDIV4_UART4DIV_Msk) >> CLK_CLKDIV4_UART4DIV_Pos;
    }
    else {}

    /* Select UART function */
    uart->FUNCSEL = UART_FUNCSEL_UART;

    /* Set UART line configuration */
    uart->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

    /* Set UART Rx and RTS trigger level */
    uart->FIFO &= ~(UART_FIFO_RFITL_Msk | UART_FIFO_RTSTRGLV_Msk);

    /* Get PLL clock frequency if UART clock source selection is PLL */
    if (u32UartClkSrcSel == 1ul)
    {
        /*Not Support PLL Clock*/
    }

    /* Get PCLK clock frequency if UART clock source selection is PCLK */
    if (u32UartClkSrcSel == 4ul)
    {
        /* UART Port as UART0 or UART2 or UART4 */
        if ((uart == (UART_T *)UART0) || (uart == (UART_T *)UART2) || (uart == (UART_T *)UART4))
        {
            au32ClkTbl[u32UartClkSrcSel] =  CLK_GetPCLK0Freq();
        }
        else     /* UART Port as UART1 or UART3*/
        {
            au32ClkTbl[u32UartClkSrcSel] =  CLK_GetPCLK1Freq();
        }

    }

    /* Set UART baud rate */
    if (u32BaudRate != 0ul)
    {
        uint32_t u32Baud_Div = 0ul;

        u32Baud_Div = UART_BAUD_MODE2_DIVIDER((au32ClkTbl[u32UartClkSrcSel]) / (u32UartClkDivNum + 1ul), u32BaudRate);

        if (u32Baud_Div > 0xFFFFul)
        {
            uart->BAUD = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER((au32ClkTbl[u32UartClkSrcSel]) / (u32UartClkDivNum + 1ul), u32BaudRate));
        }
        else
        {
            uart->BAUD = (UART_BAUD_MODE2 | u32Baud_Div);
        }
    }
}


/**
 *    @brief        Read UART data
 *
 *    @param[in]    uart            The pointer of the specified UART module.
 *    @param[in]    pu8RxBuf        The buffer to receive the data of receive FIFO.
 *    @param[in]    u32ReadBytes    The the read bytes number of data.
 *
 *    @return       u32Count Receive byte count
 *
 *    @details      The function is used to read Rx data from RX FIFO and the data will be stored in pu8RxBuf.
 */
uint32_t UART_Read(UART_T *uart, uint8_t pu8RxBuf[], uint32_t u32ReadBytes)
{
    uint32_t  u32Count;
    uint32_t  u32Exit = 0ul;

    for (u32Count = 0ul; u32Count < u32ReadBytes; u32Count++)
    {
        uint32_t u32DelayNum = 0ul;

        while (uart->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk)   /* Check RX empty => failed */
        {
            u32DelayNum++;

            if (u32DelayNum >= 0x40000000ul)
            {
                u32Exit = 1ul;
                break;
            }
            else
            {
            }
        }

        if (u32Exit == 1ul)
        {
            break;
        }
        else
        {
            pu8RxBuf[u32Count] = (uint8_t)uart->DAT; /* Get Data from UART RX  */
        }
    }

    return u32Count;

}


/**
 *    @brief        Set UART line configuration
 *
 *    @param[in]    uart            The pointer of the specified UART module.
 *    @param[in]    u32BaudRate     The register value of baudrate of UART module.
 *                                  If u32BaudRate = 0, UART baudrate will not change.
 *    @param[in]    u32DataWidth    The data length of UART module.
 *                                  - \ref UART_WORD_LEN_5
 *                                  - \ref UART_WORD_LEN_6
 *                                  - \ref UART_WORD_LEN_7
 *                                  - \ref UART_WORD_LEN_8
 *    @param[in]    u32Parity       The parity setting (none/odd/even/mark/space) of UART module.
 *                                  - \ref UART_PARITY_NONE
 *                                  - \ref UART_PARITY_ODD
 *                                  - \ref UART_PARITY_EVEN
 *                                  - \ref UART_PARITY_MARK
 *                                  - \ref UART_PARITY_SPACE
 *    @param[in]    u32StopBits     The stop bit length (1/1.5/2 bit) of UART module.
 *                                  - \ref UART_STOP_BIT_1
 *                                  - \ref UART_STOP_BIT_1_5
 *                                  - \ref UART_STOP_BIT_2
 *
 *
 *    @details      This function use to config UART line setting.
 */
void UART_SetLine_Config(UART_T *uart, uint32_t u32BaudRate, uint32_t u32DataWidth, uint32_t u32Parity, uint32_t  u32StopBits)
{
    uint32_t u32UartClkSrcSel = 0ul, u32UartClkDivNum = 0ul;
    uint32_t au32ClkTbl[6ul] = {__HXT, 0ul, __LXT, __HIRC, 0, __LIRC};



    if (uart == (UART_T *)UART0)
    {
        /* Get UART clock source selection */
        u32UartClkSrcSel = (CLK->CLKSEL1 & CLK_CLKSEL1_UART0SEL_Msk) >> CLK_CLKSEL1_UART0SEL_Pos;
        /* Get UART clock divider number */
        u32UartClkDivNum = (CLK->CLKDIV0 & CLK_CLKDIV0_UART0DIV_Msk) >> CLK_CLKDIV0_UART0DIV_Pos;
    }
    else if (uart == (UART_T *)UART1)
    {
        /* Get UART clock source selection */
        u32UartClkSrcSel = (CLK->CLKSEL1 & CLK_CLKSEL1_UART1SEL_Msk) >> CLK_CLKSEL1_UART1SEL_Pos;
        /* Get UART clock divider number */
        u32UartClkDivNum = (CLK->CLKDIV0 & CLK_CLKDIV0_UART1DIV_Msk) >> CLK_CLKDIV0_UART1DIV_Pos;
    }
    else if (uart == (UART_T *)UART2)
    {
        /* Get UART clock source selection */
        u32UartClkSrcSel = (CLK->CLKSEL3 & CLK_CLKSEL3_UART2SEL_Msk) >> CLK_CLKSEL3_UART2SEL_Pos;
        /* Get UART clock divider number */
        u32UartClkDivNum = (CLK->CLKDIV4 & CLK_CLKDIV4_UART2DIV_Msk) >> CLK_CLKDIV4_UART2DIV_Pos;
    }
    else if (uart == (UART_T *)UART3)
    {
        /* Get UART clock source selection */
        u32UartClkSrcSel = (CLK->CLKSEL3 & CLK_CLKSEL3_UART3SEL_Msk) >> CLK_CLKSEL3_UART3SEL_Pos;
        /* Get UART clock divider number */
        u32UartClkDivNum = (CLK->CLKDIV4 & CLK_CLKDIV4_UART3DIV_Msk) >> CLK_CLKDIV4_UART3DIV_Pos;
    }
    else if (uart == (UART_T *)UART4)
    {
        /* Get UART clock source selection */
        u32UartClkSrcSel = (CLK->CLKSEL3 & CLK_CLKSEL3_UART4SEL_Msk) >> CLK_CLKSEL3_UART4SEL_Pos;
        /* Get UART clock divider number */
        u32UartClkDivNum = (CLK->CLKDIV4 & CLK_CLKDIV4_UART4DIV_Msk) >> CLK_CLKDIV4_UART4DIV_Pos;
    }
    else {}

    /* Get PLL clock frequency if UART clock source selection is PLL */
    if (u32UartClkSrcSel == 1ul)
    {
        /*Not Support PLL Clock*/
    }

    /* Get PCLK clock frequency if UART clock source selection is PCLK */
    if (u32UartClkSrcSel == 4ul)
    {
        /* UART Port as UART0 or UART2 or UART4 */
        if ((uart == (UART_T *)UART0) || (uart == (UART_T *)UART2) || (uart == (UART_T *)UART4))
        {
            au32ClkTbl[u32UartClkSrcSel] =  CLK_GetPCLK0Freq();
        }
        else     /* UART Port as UART1 or UART3*/
        {
            au32ClkTbl[u32UartClkSrcSel] =  CLK_GetPCLK1Freq();
        }
    }


    /* Set UART baud rate */
    if (u32BaudRate != 0ul)
    {
        uint32_t u32Baud_Div = 0ul;

        u32Baud_Div = UART_BAUD_MODE2_DIVIDER((au32ClkTbl[u32UartClkSrcSel]) / (u32UartClkDivNum + 1ul), u32BaudRate);

        if (u32Baud_Div > 0xFFFFul)
        {
            uart->BAUD = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER((au32ClkTbl[u32UartClkSrcSel]) / (u32UartClkDivNum + 1ul), u32BaudRate));
        }
        else
        {
            uart->BAUD = (UART_BAUD_MODE2 | u32Baud_Div);
        }
    }

    /* Set UART line configuration */
    uart->LINE = u32DataWidth | u32Parity | u32StopBits;
}


/**
 *    @brief        Set Rx timeout count
 *
 *    @param[in]    uart    The pointer of the specified UART module.
 *    @param[in]    u32TOC  Rx timeout counter.
 *
 *
 *    @details      This function use to set Rx timeout count.
 */
void UART_SetTimeoutCnt(UART_T *uart, uint32_t u32TOC)
{
    /* Set time-out interrupt comparator */
    uart->TOUT = (uart->TOUT & ~UART_TOUT_TOIC_Msk) | (u32TOC);

    /* Set time-out counter enable */
    uart->INTEN |= UART_INTEN_TOCNTEN_Msk;
}


/**
 *    @brief        Select and configure IrDA function
 *
 *    @param[in]    uart            The pointer of the specified UART module.
 *    @param[in]    u32BuadRate     The baud rate of UART module.
 *    @param[in]    u32Direction    The direction of UART module in IrDA mode:
 *                                  - \ref UART_IRDA_TXEN
 *                                  - \ref UART_IRDA_RXEN
 *
  *
 *    @details      The function is used to configure IrDA relative settings. It consists of TX or RX mode and baudrate.
 */
void UART_SelectIrDAMode(UART_T *uart, uint32_t u32BuadRate, uint32_t u32Direction)
{
    uint32_t u32UartClkSrcSel = 0ul, u32UartClkDivNum = 0ul;
    uint32_t au32ClkTbl[6ul] = {__HXT, 0ul, __LXT, __HIRC, 0ul, __LIRC};

    /* Select IrDA function mode */
    uart->FUNCSEL = UART_FUNCSEL_IrDA;


    if (uart == UART0)
    {
        /* Get UART clock source selection */
        u32UartClkSrcSel = (CLK->CLKSEL1 & CLK_CLKSEL1_UART0SEL_Msk) >> CLK_CLKSEL1_UART0SEL_Pos;
        /* Get UART clock divider number */
        u32UartClkDivNum = (CLK->CLKDIV0 & CLK_CLKDIV0_UART0DIV_Msk) >> CLK_CLKDIV0_UART0DIV_Pos;
    }
    else if (uart == UART1)
    {
        /* Get UART clock source selection */
        u32UartClkSrcSel = (CLK->CLKSEL1 & CLK_CLKSEL1_UART1SEL_Msk) >> CLK_CLKSEL1_UART1SEL_Pos;
        /* Get UART clock divider number */
        u32UartClkDivNum = (CLK->CLKDIV0 & CLK_CLKDIV0_UART1DIV_Msk) >> CLK_CLKDIV0_UART1DIV_Pos;
    }
    else if (uart == UART2)
    {
        /* Get UART clock source selection */
        u32UartClkSrcSel = (CLK->CLKSEL3 & CLK_CLKSEL3_UART2SEL_Msk) >> CLK_CLKSEL3_UART2SEL_Pos;
        /* Get UART clock divider number */
        u32UartClkDivNum = (CLK->CLKDIV4 & CLK_CLKDIV4_UART2DIV_Msk) >> CLK_CLKDIV4_UART2DIV_Pos;
    }
    else if (uart == (UART_T *)UART3)
    {
        /* Get UART clock source selection */
        u32UartClkSrcSel = (CLK->CLKSEL3 & CLK_CLKSEL3_UART3SEL_Msk) >> CLK_CLKSEL3_UART3SEL_Pos;
        /* Get UART clock divider number */
        u32UartClkDivNum = (CLK->CLKDIV4 & CLK_CLKDIV4_UART3DIV_Msk) >> CLK_CLKDIV4_UART3DIV_Pos;
    }
    else if (uart == (UART_T *)UART4)
    {
        /* Get UART clock source selection */
        u32UartClkSrcSel = (CLK->CLKSEL3 & CLK_CLKSEL3_UART4SEL_Msk) >> CLK_CLKSEL3_UART4SEL_Pos;
        /* Get UART clock divider number */
        u32UartClkDivNum = (CLK->CLKDIV4 & CLK_CLKDIV4_UART4DIV_Msk) >> CLK_CLKDIV4_UART4DIV_Pos;
    }
    else {}


    /* Get PLL clock frequency if UART clock source selection is PLL */
    if (u32UartClkSrcSel == 1ul)
    {
        /*Not Support PLL Clock*/
    }

    /* Get PCLK clock frequency if UART clock source selection is PCLK */
    if (u32UartClkSrcSel == 4ul)
    {
        /* UART Port as UART0 or UART2 or UART4 */
        if ((uart == (UART_T *)UART0) || (uart == (UART_T *)UART2) || (uart == (UART_T *)UART4))
        {
            au32ClkTbl[u32UartClkSrcSel] =  CLK_GetPCLK0Freq();
        }
        else     /* UART Port as UART1 or UART3*/
        {
            au32ClkTbl[u32UartClkSrcSel] =  CLK_GetPCLK1Freq();
        }
    }


    /* Set UART IrDA baud rate in mode 0 */
    if (u32BuadRate != 0ul)
    {
        uint32_t u32Baud_Div = 0ul;

        u32Baud_Div = UART_BAUD_MODE0_DIVIDER((au32ClkTbl[u32UartClkSrcSel]) / (u32UartClkDivNum + 1ul), u32BuadRate);

        if (u32Baud_Div < 0xFFFFul)
        {
            uart->BAUD = (UART_BAUD_MODE0 | u32Baud_Div);
        }
        else
        {
        }
    }

    /* Configure IrDA relative settings */
    if (u32Direction == UART_IRDA_RXEN)
    {
        uart->IRDA |= UART_IRDA_RXINV_Msk;     /*Rx signal is inverse*/
        uart->IRDA &= ~UART_IRDA_TXEN_Msk;
    }
    else
    {
        uart->IRDA &= ~UART_IRDA_TXINV_Msk;    /*Tx signal is not inverse*/
        uart->IRDA |= UART_IRDA_TXEN_Msk;
    }

}


/**
 *    @brief        Select and configure RS485 function
 *
 *    @param[in]    uart        The pointer of the specified UART module.
 *    @param[in]    u32Mode     The operation mode(NMM/AUD/AAD).
 *                              - \ref UART_ALTCTL_RS485NMM_Msk
 *                              - \ref UART_ALTCTL_RS485AUD_Msk
 *                              - \ref UART_ALTCTL_RS485AAD_Msk
 *    @param[in]    u32Addr     The RS485 address.
 *
 *
 *    @details      The function is used to set RS485 relative setting.
 */
void UART_SelectRS485Mode(UART_T *uart, uint32_t u32Mode, uint32_t u32Addr)
{
    /* Select UART RS485 function mode */
    uart->FUNCSEL = UART_FUNCSEL_RS485;

    /* Set RS485 configuration */
    uart->ALTCTL &= ~(UART_ALTCTL_RS485NMM_Msk | UART_ALTCTL_RS485AUD_Msk | UART_ALTCTL_RS485AAD_Msk | UART_ALTCTL_ADDRMV_Msk);
    uart->ALTCTL |= (u32Mode | (u32Addr << UART_ALTCTL_ADDRMV_Pos));
}


/**
 *    @brief        Write UART data
 *
 *    @param[in]    uart            The pointer of the specified UART module.
 *    @param[in]    pu8TxBuf        The buffer to send the data to UART transmission FIFO.
 *    @param[out]   u32WriteBytes   The byte number of data.
 *
 *    @return       u32Count transfer byte count
 *
 *    @details      The function is to write data into TX buffer to transmit data by UART.
 */
uint32_t UART_Write(UART_T *uart, uint8_t pu8TxBuf[], uint32_t u32WriteBytes)
{
    uint32_t  u32Count;
    uint32_t  u32Exit = 0ul;

    for (u32Count = 0ul; u32Count != u32WriteBytes; u32Count++)
    {
        uint32_t u32DelayNum = 0ul;

        while (uart->FIFOSTS & UART_FIFOSTS_TXFULL_Msk)   /* Check Tx Full */
        {
            u32DelayNum++;

            if (u32DelayNum >= 0x40000000ul)
            {
                u32Exit = 1ul;
                break;
            }
            else
            {
            }
        }

        if (u32Exit == 1ul)
        {
            break;
        }
        else
        {
            uart->DAT = pu8TxBuf[u32Count];    /* Send UART Data from buffer */
        }
    }

    return u32Count;

}

/**
 *    @brief        Select Single Wire mode function
 *
 *    @param[in]    uart        The pointer of the specified UART module.
 *
 *
 *    @details      The function is used to select Single Wire mode.
 */
void UART_SelectSingleWireMode(UART_T *uart)
{

    /* Select UART SingleWire function mode */
    uart->FUNCSEL = ((uart->FUNCSEL & (~UART_FUNCSEL_FUNCSEL_Msk)) | UART_FUNCSEL_SINGLE_WIRE);

}


/** @} end of group UART_EXPORTED_FUNCTIONS */

/** @} end of group UART_Driver */

/** @} end of group Standard_Driver */
