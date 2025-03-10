/**************************************************************************//**
 * @file     startup_M253.c
 * @version  V1.00
 * @brief    CMSIS Device Startup File for NuMicro M253
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <inttypes.h>
#include <stdio.h>
#include "NuMicro.h"

/*----------------------------------------------------------------------------
  External References
 *----------------------------------------------------------------------------*/
extern uint32_t __INITIAL_SP;
extern uint32_t __STACK_LIMIT;

extern __NO_RETURN void __PROGRAM_START(void);

/*----------------------------------------------------------------------------
  Internal References
 *----------------------------------------------------------------------------*/
__NO_RETURN void Reset_Handler(void);
__NO_RETURN void Default_Handler(void);

/*----------------------------------------------------------------------------
  Exception / Interrupt Handler
 *----------------------------------------------------------------------------*/
/* Exceptions */
void NMI_Handler(void)              __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)        __attribute__((weak));
void SVC_Handler(void)              __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)           __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void)          __attribute__((weak, alias("Default_Handler")));

/* External Interrupts */
void BOD_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 0: Brown Out detection
void IRCTRIM_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));    // 1: Internal RC
void PWRWU_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 2: Power down wake up
void CLKFAIL_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));    // 4: Clock detection fail
void RTC_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 6: Real Time Clock
void WDT_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 8: Watchdog timer
void WWDT_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 9: Window watchdog timer
void EINT0_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 10: External Input 0
void EINT1_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 11: External Input 1
void EINT2_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 12: External Input 2
void EINT3_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 13: External Input 3
void EINT4_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 14: External Input 4
void EINT5_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 15: External Input 5
void GPA_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 16: GPIO Port A
void GPB_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 17: GPIO Port B
void GPC_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 18: GPIO Port C
void GPD_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 19: GPIO Port D
void GPE_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 20: GPIO Port E
void GPF_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 21: GPIO Port F
void SPI0_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 23: SPI0
void CANFD0_IRQ0_IRQHandler(void)   __attribute__((weak, alias("Default_Handler")));    // 24: CANFD0 interrupt 0
void CANFD0_IRQ1_IRQHandler(void)   __attribute__((weak, alias("Default_Handler")));    // 25: CANFD0 interrupt 1
void TMR0_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 32: Timer 0
void TMR1_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 33: Timer 1
void TMR2_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 34: Timer 2
void TMR3_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 35: Timer 3
void UART0_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 36: UART0
void UART1_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 37: UART1
void I2C0_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 38: I2C0
void I2C1_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 39: I2C1
void PDMA_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 40: Peripheral DMA
void EADC_INT0_IRQHandler(void)     __attribute__((weak, alias("Default_Handler")));    // 42: EADC interrupt source 0
void EADC_INT1_IRQHandler(void)     __attribute__((weak, alias("Default_Handler")));    // 43: EADC interrupt source 1
void BPWM0_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 45: BPWM0
void EADC_INT2_IRQHandler(void)     __attribute__((weak, alias("Default_Handler")));    // 46: EADC interrupt source 2
void EADC_INT3_IRQHandler(void)     __attribute__((weak, alias("Default_Handler")));    // 47: EADC interrupt source 3
void UART2_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 48: UART2
void UART3_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 49: UART3
void USCI0_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 50: USCI0
void UART4_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 51: SPI1
void USBD_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 53: USB device

/*----------------------------------------------------------------------------
  Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/
#if defined ( __GNUC__ )
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wpedantic"
#endif

extern const VECTOR_TABLE_Type __VECTOR_TABLE[];
const VECTOR_TABLE_Type __VECTOR_TABLE[] __VECTOR_TABLE_ATTRIBUTE =
{
    (VECTOR_TABLE_Type)(&__INITIAL_SP),       /*       Initial Stack Pointer                            */
    Reset_Handler,                            /*       Reset Handler                                    */
    NMI_Handler,                              /*   -14 NMI Handler                                      */
    HardFault_Handler,                        /*   -13 Hard Fault Handler                               */
    0,                                        /*   -12 Reserved                                         */
    0,                                        /*   -11 Reserved                                         */
    0,                                        /*   -10 Reserved                                         */
    0,                                        /*    -9 Reserved                                         */
    0,                                        /*    -8 Reserved                                         */
    0,                                        /*    -7 Reserved                                         */
    0,                                        /*    -6 Reserved                                         */
    SVC_Handler,                              /*    -5 SVC Handler                                      */
    0,                                        /*    -4 Reserved                                         */
    0,                                        /*    -3 Reserved                                         */
    PendSV_Handler,                           /*    -2 PendSV Handler Handler                           */
    SysTick_Handler,                          /*    -1 SysTick Handler                                  */

    /* Interrupts */
    BOD_IRQHandler,                           /*    0: Brown Out detection                               */
    IRCTRIM_IRQHandler,                       /*    1: Internal RC                                       */
    PWRWU_IRQHandler,                         /*    2: Power down wake up                                */
    Default_Handler,                          /*    3: Reserved                                          */
    CLKFAIL_IRQHandler,                       /*    4: Clock detection fail                              */
    Default_Handler,                          /*    5: Reserved                                          */
    RTC_IRQHandler,                           /*    6: Real Time Clock                                   */
    Default_Handler,                          /*    7: Reserved                                          */
    WDT_IRQHandler,                           /*    8: Watchdog timer                                    */
    WWDT_IRQHandler,                          /*    9: Window watchdog timer                             */
    EINT0_IRQHandler,                         /*    10: External Input 0                                 */
    EINT1_IRQHandler,                         /*    11: External Input 1                                 */
    EINT2_IRQHandler,                         /*    12: External Input 2                                 */
    EINT3_IRQHandler,                         /*    13: External Input 3                                 */
    EINT4_IRQHandler,                         /*    14: External Input 4                                 */
    EINT5_IRQHandler,                         /*    15: External Input 5                                 */
    GPA_IRQHandler,                           /*    16: GPIO Port A                                      */
    GPB_IRQHandler,                           /*    17: GPIO Port B                                      */
    GPC_IRQHandler,                           /*    18: GPIO Port C                                      */
    GPD_IRQHandler,                           /*    19: GPIO Port D                                      */
    GPE_IRQHandler,                           /*    20: GPIO Port E                                      */
    GPF_IRQHandler,                           /*    21: GPIO Port F                                      */
    Default_Handler,                          /*    22: Reserved                                         */
    SPI0_IRQHandler,                          /*    23: SPI0                                             */
    CANFD0_IRQ0_IRQHandler,                   /*    24: CANFD0 interrupt 0                               */
    CANFD0_IRQ1_IRQHandler,                   /*    25: CANFD0 interrupt 1                               */
    Default_Handler,                          /*    26: Reserved                                         */
    Default_Handler,                          /*    27: Reserved                                         */
    Default_Handler,                          /*    28: Reserved                                         */
    Default_Handler,                          /*    29: Reserved                                         */
    Default_Handler,                          /*    30: Reserved                                         */
    Default_Handler,                          /*    31: Reserved                                         */
    TMR0_IRQHandler,                          /*    32: Timer 0                                          */
    TMR1_IRQHandler,                          /*    33: Timer 1                                          */
    TMR2_IRQHandler,                          /*    34: Timer 2                                          */
    TMR3_IRQHandler,                          /*    35: Timer 3                                          */
    UART0_IRQHandler,                         /*    36: UART0                                            */
    UART1_IRQHandler,                         /*    37: UART1                                            */
    I2C0_IRQHandler,                          /*    38: I2C0                                             */
    I2C1_IRQHandler,                          /*    39: I2C1                                             */
    PDMA_IRQHandler,                          /*    40: Peripheral DMA                                   */
    Default_Handler,                          /*    41: Reserved                                         */
    EADC_INT0_IRQHandler,                     /*    42: EADC interrupt source 0                          */
    EADC_INT1_IRQHandler,                     /*    43: EADC interrupt source 1                          */
    Default_Handler,                          /*    44: Reserved                                         */
    BPWM0_IRQHandler,                         /*    45: BPWM0                                            */
    EADC_INT2_IRQHandler,                     /*    46: EADC interrupt source 2                          */
    EADC_INT3_IRQHandler,                     /*    47: EADC interrupt source 3                          */
    UART2_IRQHandler,                         /*    48: UART2                                            */
    UART3_IRQHandler,                         /*    49: UART3                                            */
    USCI0_IRQHandler,                         /*    50: USCI0                                            */
    UART4_IRQHandler,                         /*    51: UART4                                            */
    Default_Handler,                          /*    52: Reserved                                         */
    USBD_IRQHandler                           /*    53: USB device                                       */
};

#if defined ( __GNUC__ )
    #pragma GCC diagnostic pop
#endif

__WEAK void Reset_Handler_PreInit(void)
{
    // Empty function
}

/*----------------------------------------------------------------------------
  Reset Handler called on controller reset
 *----------------------------------------------------------------------------*/
__NO_RETURN void Reset_Handler(void)
{
    __set_PSP((uint32_t)(&__INITIAL_SP));
    __set_MSPLIM((uint32_t)(&__STACK_LIMIT));
    __set_PSPLIM((uint32_t)(&__STACK_LIMIT));

    Reset_Handler_PreInit();
    /* Unlock protected registers */
    SYS_UnlockReg();

    SystemInit();               /* CMSIS System Initialization */

    /* Init POR */
    SYS->PORCTL0 = 0x5AA5;
    SYS->PORCTL1 = 0x5AA5;
    /* Lock protected registers */
    SYS_LockReg();

    __PROGRAM_START();          /* Enter PreMain (C library entry point) */
}

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wmissing-noreturn"
#endif

/*---------------------------------------------------------------------------
  Hard Fault Handler
 *---------------------------------------------------------------------------*/
__WEAK void HardFault_Handler(void)
{
    while (1);
}

/*----------------------------------------------------------------------------
  Default Handler for Exceptions / Interrupts
 *----------------------------------------------------------------------------*/
void Default_Handler(void)
{
    while (1);
}

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic pop
#endif
