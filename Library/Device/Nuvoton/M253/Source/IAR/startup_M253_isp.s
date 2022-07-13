;/******************************************************************************
; * @file     startup_M253.s
; * @version  V0.10
; * @brief    CMSIS Cortex-M23 Core Device Startup File for M253
; *
; * SPDX-License-Identifier: Apache-2.0
; * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
        ;EXTERN  HardFault_Handler
        EXTERN  ProcessHardFault
        EXTERN  SystemInit
        PUBLIC  __vector_table
       ; PUBLIC  __vector_table_0x1c
        PUBLIC  __Vectors
        PUBLIC  __Vectors_End
        PUBLIC  __Vectors_Size

        DATA

__vector_table
        DCD     sfe(CSTACK)
        DCD     Reset_Handler

        DCD     NMI_Handler
        DCD     HardFault_Handler
        DCD     MemManage_Handler
        DCD     BusFault_Handler
        DCD     UsageFault_Handler
;__vector_table_0x1c
        DCD     0
        DCD     0
        DCD     0
        DCD     0
        DCD     SVC_Handler
        DCD     0
        DCD     0
        DCD     PendSV_Handler
        DCD     SysTick_Handler

        ; External Interrupts
        DCD     BOD_IRQHandler            ; 0: Brown Out detection
        DCD     IRCTRIM_IRQHandler        ; 1: Internal RC
        DCD     PWRWU_IRQHandler          ; 2: Power down wake up
        DCD     DEFAULT_IRQHandler        ; 3: Reserved
        DCD     CLKFAIL_IRQHandler        ; 4: Clock detection fail
        DCD     DEFAULT_IRQHandler        ; 5: Reserved
        DCD     RTC_IRQHandler            ; 6: Real Time Clock
        DCD     DEFAULT_IRQHandler        ; 7: Reserved
        DCD     WDT_IRQHandler            ; 8: Watchdog timer
        DCD     WWDT_IRQHandler           ; 9: Window watchdog timer
        DCD     EINT0_IRQHandler          ; 10: External Input 0
        DCD     EINT1_IRQHandler          ; 11: External Input 1
        DCD     EINT2_IRQHandler          ; 12: External Input 2
        DCD     EINT3_IRQHandler          ; 13: External Input 3
        DCD     EINT4_IRQHandler          ; 14: External Input 4
        DCD     EINT5_IRQHandler          ; 15: External Input 5
        DCD     DEFAULT_IRQHandler        ; 16: GPIO Port A
        DCD     DEFAULT_IRQHandler        ; 17: GPIO Port B
        DCD     DEFAULT_IRQHandler        ; 18: GPIO Port C
        DCD     DEFAULT_IRQHandler        ; 19: GPIO Port D
        DCD     DEFAULT_IRQHandler        ; 20: GPIO Port E
        DCD     DEFAULT_IRQHandler        ; 21: GPIO Port F
        DCD     DEFAULT_IRQHandler        ; 22: Reserved
        DCD     SPI0_IRQHandler           ; 23: SPI0
        DCD     CANFD0_IRQ0_IRQHandler    ; 24: CAN FD0 interrupt 0
        DCD     CANFD0_IRQ1_IRQHandler    ; 25: CAN FD0 interrupt 1
        DCD     DEFAULT_IRQHandler        ; 26: Reserved
        DCD     DEFAULT_IRQHandler        ; 27: Reserved
        DCD     DEFAULT_IRQHandler        ; 28: Reserved
        DCD     DEFAULT_IRQHandler        ; 29: Reserved
        DCD     DEFAULT_IRQHandler        ; 30: Reserved
        DCD     DEFAULT_IRQHandler        ; 31: Reserved
        DCD     TMR0_IRQHandler           ; 32: Timer 0
        DCD     TMR1_IRQHandler           ; 33: Timer 1
        DCD     TMR2_IRQHandler           ; 34: Timer 2
        DCD     TMR3_IRQHandler           ; 35: Timer 3
        DCD     UART0_IRQHandler          ; 36: UART0
        DCD     UART1_IRQHandler          ; 37: UART1
        DCD     I2C0_IRQHandler           ; 38: I2C0
        DCD     I2C1_IRQHandler           ; 39: I2C1
        DCD     PDMA_IRQHandler           ; 40: Peripheral DMA
        DCD     DEFAULT_IRQHandler        ; 41: Reserved
        DCD     DEFAULT_IRQHandler        ; 42: EADC interrupt source 0
        DCD     DEFAULT_IRQHandler        ; 43: EADC interrupt source 1
        DCD     DEFAULT_IRQHandler        ; 44: Reserved
        DCD     DEFAULT_IRQHandler        ; 45: BPWM0
        DCD     DEFAULT_IRQHandler        ; 46: EADC interrupt source 2
        DCD     DEFAULT_IRQHandler        ; 47: EADC interrupt source 3
        DCD     DEFAULT_IRQHandler        ; 48: UART2
        DCD     DEFAULT_IRQHandler        ; 49: UART3
        DCD     DEFAULT_IRQHandler        ; 50: USCI0
        DCD     DEFAULT_IRQHandler        ; 51: UART4
        DCD     DEFAULT_IRQHandler        ; 52: Reserved
        DCD     USBD_IRQHandler           ; 53: USB device
;        DCD     DEFAULT_IRQHandler        ; 54: Reserved
;        DCD     DEFAULT_IRQHandler        ; 55: Reserved
;        DCD     DEFAULT_IRQHandler        ; 56: Reserved
;        DCD     DEFAULT_IRQHandler        ; 57: Reserved
;        DCD     DEFAULT_IRQHandler        ; 58: Reserved
;        DCD     DEFAULT_IRQHandler        ; 59: Reserved
;        DCD     DEFAULT_IRQHandler        ; 60: Reserved
;        DCD     DEFAULT_IRQHandler        ; 61: Reserved
;        DCD     DEFAULT_IRQHandler        ; 62: Reserved
;        DCD     DEFAULT_IRQHandler        ; 63: Reserved
__Vectors_End

__Vectors       EQU   __vector_table
__Vectors_Size  EQU   __Vectors_End - __Vectors


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
        THUMB

        PUBWEAK Reset_Handler
        SECTION .text:CODE:REORDER:NOROOT(2)
Reset_Handler
        ; Unlock Register
        LDR     R0, =0x40000100
        LDR     R1, =0x59
        STR     R1, [R0]
        LDR     R1, =0x16
        STR     R1, [R0]
        LDR     R1, =0x88
        STR     R1, [R0]

        LDR     R0, =SystemInit
        BLX     R0

        ; Init POR
        LDR     R2, =0x40000024
        LDR     R1, =0x00005AA5
        STR     R1, [R2]

	    LDR     R2, =0x400001EC
        STR     R1, [R2]

        ; Lock register
        LDR     R0, =0x40000100
        MOVS    R1, #0
        STR     R1, [R0]

        LDR     R0, =__iar_program_start
        BX      R0

        PUBWEAK NMI_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
NMI_Handler
        B NMI_Handler

        PUBWEAK HardFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
HardFault_Handler
        B HardFault_Handler

        PUBWEAK MemManage_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
MemManage_Handler
        B MemManage_Handler

        PUBWEAK BusFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
BusFault_Handler
        B BusFault_Handler

        PUBWEAK UsageFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UsageFault_Handler
        B UsageFault_Handler

        PUBWEAK SVC_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SVC_Handler
        B SVC_Handler

        ;PUBWEAK DebugMon_Handler
        ;SECTION .text:CODE:REORDER:NOROOT(1)
;DebugMon_Handler
        ;B DebugMon_Handler

        PUBWEAK PendSV_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PendSV_Handler
        B PendSV_Handler

        PUBWEAK SysTick_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SysTick_Handler
        B SysTick_Handler

        PUBWEAK    BOD_IRQHandler
        PUBWEAK    IRCTRIM_IRQHandler
        PUBWEAK    PWRWU_IRQHandler
       ;PUBWEAK    DEFAULT_IRQHandler
        PUBWEAK    CLKFAIL_IRQHandler
       ;PUBWEAK    DEFAULT_IRQHandler
        PUBWEAK    RTC_IRQHandler
       ;PUBWEAK    DEFAULT_IRQHandler
        PUBWEAK    WDT_IRQHandler
        PUBWEAK    WWDT_IRQHandler
        PUBWEAK    EINT0_IRQHandler
        PUBWEAK    EINT1_IRQHandler
        PUBWEAK    EINT2_IRQHandler
        PUBWEAK    EINT3_IRQHandler
        PUBWEAK    EINT4_IRQHandler
        PUBWEAK    EINT5_IRQHandler
        ;PUBWEAK    GPA_IRQHandler
        ;PUBWEAK    GPB_IRQHandler
        ;PUBWEAK    GPC_IRQHandler
        ;PUBWEAK    GPD_IRQHandler
        ;PUBWEAK    GPE_IRQHandler
        ;PUBWEAK    GPF_IRQHandler
       ;PUBWEAK    DEFAULT_IRQHandler
        PUBWEAK    SPI0_IRQHandler
        PUBWEAK    CANFD0_IRQ0_IRQHandler
        PUBWEAK    CANFD0_IRQ1_IRQHandler
       ;PUBWEAK    DEFAULT_IRQHandler
       ;PUBWEAK    DEFAULT_IRQHandler
       ;PUBWEAK    DEFAULT_IRQHandler
       ;PUBWEAK    DEFAULT_IRQHandler
       ;PUBWEAK    DEFAULT_IRQHandler
       ;PUBWEAK    DEFAULT_IRQHandler
        PUBWEAK    TMR0_IRQHandler
        PUBWEAK    TMR1_IRQHandler
        PUBWEAK    TMR2_IRQHandler
        PUBWEAK    TMR3_IRQHandler
        PUBWEAK    UART0_IRQHandler
        PUBWEAK    UART1_IRQHandler
        PUBWEAK    I2C0_IRQHandler
        PUBWEAK    I2C1_IRQHandler
        PUBWEAK    PDMA_IRQHandler
       ;PUBWEAK    DEFAULT_IRQHandler
       ;PUBWEAK    EADC_INT0_IRQHandler
       ;PUBWEAK    EADC_INT1_IRQHandler
       ;PUBWEAK    DEFAULT_IRQHandler
       ;PUBWEAK    BPWM0_IRQHandler
       ;PUBWEAK    EADC_INT2_IRQHandler
       ;PUBWEAK    EADC_INT3_IRQHandler
       ;PUBWEAK    UART2_IRQHandler
       ;PUBWEAK    UART3_IRQHandler
       ;PUBWEAK    USCI0_IRQHandler
       ;PUBWEAK    UART4_IRQHandler
       ;PUBWEAK    DEFAULT_IRQHandler
       PUBWEAK    USBD_IRQHandler
       ;PUBWEAK    DEFAULT_IRQHandler
       ;PUBWEAK    DEFAULT_IRQHandler
       ;PUBWEAK    DEFAULT_IRQHandler
       ;PUBWEAK    DEFAULT_IRQHandler
       ;PUBWEAK    DEFAULT_IRQHandler
       ;PUBWEAK    DEFAULT_IRQHandler
       ;PUBWEAK    DEFAULT_IRQHandler
       ;PUBWEAK    DEFAULT_IRQHandler
       ;PUBWEAK    DEFAULT_IRQHandler
       ;PUBWEAK    DEFAULT_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)

BOD_IRQHandler
IRCTRIM_IRQHandler
PWRWU_IRQHandler
;DEFAULT_IRQHandler
CLKFAIL_IRQHandler
;DEFAULT_IRQHandler
RTC_IRQHandler
;DEFAULT_IRQHandler
WDT_IRQHandler
WWDT_IRQHandler
EINT0_IRQHandler
EINT1_IRQHandler
EINT2_IRQHandler
EINT3_IRQHandler
EINT4_IRQHandler
EINT5_IRQHandler
;GPA_IRQHandler
;GPB_IRQHandler
;GPC_IRQHandler
;GPD_IRQHandler
;GPE_IRQHandler
;GPF_IRQHandler
;DEFAULT_IRQHandler
SPI0_IRQHandler
CANFD0_IRQ0_IRQHandler
CANFD0_IRQ1_IRQHandler
;DEFAULT_IRQHandler
;DEFAULT_IRQHandler
;DEFAULT_IRQHandler
;DEFAULT_IRQHandler
;DEFAULT_IRQHandler
;DEFAULT_IRQHandler
TMR0_IRQHandler
TMR1_IRQHandler
TMR2_IRQHandler
TMR3_IRQHandler
UART0_IRQHandler
UART1_IRQHandler
I2C0_IRQHandler
I2C1_IRQHandler
PDMA_IRQHandler
;DEFAULT_IRQHandler
;EADC_INT0_IRQHandler
;EADC_INT1_IRQHandler
;DEFAULT_IRQHandler
;BPWM0_IRQHandler
;EADC_INT2_IRQHandler
;EADC_INT3_IRQHandler
;UART2_IRQHandler
;UART3_IRQHandler
;USCI0_IRQHandler
;UART4_IRQHandler
;DEFAULT_IRQHandler
USBD_IRQHandler
;DEFAULT_IRQHandler
;DEFAULT_IRQHandler
;DEFAULT_IRQHandler
;DEFAULT_IRQHandler
;DEFAULT_IRQHandler
;DEFAULT_IRQHandler
;DEFAULT_IRQHandler
;DEFAULT_IRQHandler
;DEFAULT_IRQHandler
DEFAULT_IRQHandler
    B DEFAULT_IRQHandler

    END
