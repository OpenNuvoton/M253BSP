/**************************************************************************//**
 * @file     M253.h
 * @version  V1.0
 * @brief    Peripheral Access Layer Header File
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/**
  \mainpage NuMicro M253 Series CMSIS BSP Driver Reference
  *
  * <b>Introduction</b>
  *
  * This user manual describes the usage of M253 Series MCU device driver
  *
  * <b>Disclaimer</b>
  *
  * The Software is furnished "AS IS", without warranty as to performance or results, and
  * the entire risk as to performance or results is assumed by YOU. Nuvoton disclaims all
  * warranties, express, implied or otherwise, with regard to the Software, its use, or
  * operation, including without limitation any and all warranties of merchantability, fitness
  * for a particular purpose, and non-infringement of intellectual property rights.
  *
  * <b>Important Notice</b>
  *
  * Nuvoton Products are neither intended nor warranted for usage in systems or equipment,
  * any malfunction or failure of which may cause loss of human life, bodily injury or severe
  * property damage. Such applications are deemed, "Insecure Usage".
  *
  * Insecure usage includes, but is not limited to: equipment for surgical implementation,
  * atomic energy control instruments, airplane or spaceship instruments, the control or
  * operation of dynamic, brake or safety systems designed for vehicular use, traffic signal
  * instruments, all types of safety devices, and other applications intended to support or
  * sustain life.
  *
  * All Insecure Usage shall be made at customer's risk, and in the event that third parties
  * lay claims to Nuvoton as a result of customer's Insecure Usage, customer shall indemnify
  * the damages and liabilities thus incurred by Nuvoton.
  *
  * Please note that all data and specifications are subject to change without notice. All the
  * trademarks of products and companies mentioned in this datasheet belong to their respective
  * owners.
  *
  * <b>Copyright Notice</b>
  *
  * SPDX-License-Identifier: Apache-2.0<br/>
  * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
  */

#ifndef __M253_H__
#define __M253_H__

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************/
/*                Processor and Core Peripherals                              */
/******************************************************************************/
/** @addtogroup CMSIS_Device CMSIS Definitions
  Configuration of the Cortex-M23 Processor and Core Peripherals
  @{
*/

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */

/**
 * @details  Interrupt Number Definition.
 */
typedef enum IRQn
{
    /******  Cortex-M0 Processor Exceptions Numbers ***********************************************/
    NonMaskableInt_IRQn       = -14,    /*!< 2 Non Maskable Interrupt                             */
    HardFault_IRQn            = -13,    /*!< 3 Cortex-M0 Hard Fault Interrupt                     */
    SVCall_IRQn               = -5,     /*!< 11 Cortex-M0 SV Call Interrupt                       */
    PendSV_IRQn               = -2,     /*!< 14 Cortex-M0 Pend SV Interrupt                       */
    SysTick_IRQn              = -1,     /*!< 15 Cortex-M0 System Tick Interrupt                   */

    /******  ARMIKMCU Swift specific Interrupt Numbers ********************************************/
    BOD_IRQn                  = 0,      /*!< Brown-Out Low Voltage Detected Interrupt             */
    IRCTRIM_IRQn              = 1,      /*!< Watch Dog Timer Interrupt                            */
    PWRWU_IRQn                = 2,      /*!< EINT0, EINT2 and EINT4 Interrupt                     */
    RESERVE0                  = 3,      /*!< Reserve 0                                            */
    CLKFAIL_IRQn              = 4,      /*!< Clock fail detected Interrupt                        */
    RESERVE1                  = 5,      /*!< Reserve 1                                            */
    RTC_IRQn                  = 6,      /*!< Real Time Clock Interrupt                            */
    RESERVE2                  = 7,      /*!< Reserve 2                                            */
    WDT_IRQn                  = 8,      /*!< Watch Dog Timer Interrupt                            */
    WWDT_IRQn                 = 9,      /*!< Window Watch Dog Timer Interrupt                     */
    EINT0_IRQn                = 10,     /*!< External Input 0 Interrupt                           */
    EINT1_IRQn                = 11,     /*!< External Input 1 Interrupt                           */
    EINT2_IRQn                = 12,     /*!< External Input 2 Interrupt                           */
    EINT3_IRQn                = 13,     /*!< External Input 3 Interrupt                           */
    EINT4_IRQn                = 14,     /*!< External Input 4 Interrupt                           */
    EINT5_IRQn                = 15,     /*!< External Input 5 Interrupt                           */
    GPA_IRQn                  = 16,     /*!< GPIO PORT A Interrupt                                */
    GPB_IRQn                  = 17,     /*!< GPIO PORT B Interrupt                                */
    GPC_IRQn                  = 18,     /*!< GPIO PORT C Interrupt                                */
    GPD_IRQn                  = 19,     /*!< GPIO PORT D Interrupt                                */
    GPE_IRQn                  = 20,     /*!< GPIO PORT E Interrupt                                */
    GPF_IRQn                  = 21,     /*!< GPIO PORT F Interrupt                                */
    RESERVE3                  = 22,     /*!< Reserve 3                                            */
    SPI0_IRQn                 = 23,     /*!< SPI0 Interrupt                                       */
    CANFD0_IRQ0_IRQn          = 24,     /*!< CANFD0 Interrupt0                                    */
    CANFD0_IRQ1_IRQn          = 25,     /*!< CANFD0 Interrupt1                                    */
    RESERVE5                  = 26,     /*!< Reserve 5                                            */
    RESERVE6                  = 27,     /*!< Reserve 6                                            */
    RESERVE7                  = 28,     /*!< Reserve 7                                            */
    RESERVE8                  = 29,     /*!< Reserve 8                                            */
    RESERVE9                  = 30,     /*!< Reserve 9                                            */
    RESERVE10                 = 31,     /*!< Reserve 10                                           */
    TMR0_IRQn                 = 32,     /*!< TIMER0  Interrupt                                    */
    TMR1_IRQn                 = 33,     /*!< TIMER1  Interrupt                                    */
    TMR2_IRQn                 = 34,     /*!< TIMER2  Interrupt                                    */
    TMR3_IRQn                 = 35,     /*!< TIMER3  Interrupt                                    */
    UART0_IRQn                = 36,     /*!< UART0  Interrupt                                     */
    UART1_IRQn                = 37,     /*!< UART1  Interrupt                                     */
    I2C0_IRQn                 = 38,     /*!< I2C0  Interrupt                                      */
    I2C1_IRQn                 = 39,     /*!< I2C1  Interrupt                                      */
    PDMA_IRQn                 = 40,     /*!< Peripheral DMA Interrupt                             */
    RESERVE11                 = 41,     /*!< Reserve 11                                           */
    EADC_INT0_IRQn            = 42,     /*!< Enhance ADC Interrupt 0                              */
    EADC_INT1_IRQn            = 43,     /*!< Enhance ADC Interrupt 1                              */
    RESERVE12                 = 44,     /*!< Reserve 12                                           */
    BPWM0_IRQn                = 45,     /*!< BPWM0 Interrupt                                      */
    EADC_INT2_IRQn            = 46,     /*!< Enhance EADC Interrupt 2                             */
    EADC_INT3_IRQn            = 47,     /*!< Enhance EADC Interrupt 3                             */
    UART2_IRQn                = 48,     /*!< UART2 Interrupt                                      */
    UART3_IRQn                = 49,     /*!< UART3 Interrupt                                      */
    USCI0_IRQn                = 50,     /*!< USCI0 Interrupt                                      */
    UART4_IRQn                = 51,     /*!< UART4 Interrupt                                      */
    RESERVE14                 = 52,     /*!< Reserve 13                                           */
    USBD_IRQn                 = 53,     /*!< USB Device Interrupt                                 */
    RESERVE15                 = 54,     /*!< Reserve 14                                           */
    RESERVE16                 = 55,     /*!< Reserve 15                                           */
    RESERVE17                 = 56,     /*!< Reserve 16                                           */
    RESERVE18                 = 57,     /*!< Reserve 17                                           */
    RESERVE19                 = 58,     /*!< Reserve 18                                           */
    RESERVE20                 = 59,     /*!< Reserve 19                                           */
    RESERVE21                 = 60,     /*!< Reserve 20                                           */
    RESERVE22                 = 61,     /*!< Reserve 21                                           */
    RESERVE23                 = 62,     /*!< Reserve 22                                           */
    RESERVE24                 = 63,     /*!< Reserve 23                                           */
} IRQn_Type;

/* ================================================================================ */
/* ================      Processor and Core Peripheral Section     ================ */
/* ================================================================================ */

/* -------  Start of section using anonymous unions and disabling warnings  ------- */
#if   defined (__CC_ARM)
#pragma push
#pragma anon_unions
#elif defined (__ICCARM__)
#pragma language=extended
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wc11-extensions"
#pragma clang diagnostic ignored "-Wreserved-id-macro"
#elif defined (__GNUC__)
/* anonymous unions are enabled by default */
#elif defined (__TMS470__)
/* anonymous unions are enabled by default */
#elif defined (__TASKING__)
#pragma warning 586
#elif defined (__CSMC__)
/* anonymous unions are enabled by default */
#else
#warning Not supported compiler type
#endif

/* --------  Configuration of the Cortex-ARMv8MBL Processor and Core Peripherals  ------- */
#define __ARMv8MBL_REV            0x0000U   /* Core revision r0p0                         */
#define __SAU_PRESENT             0U        /* SAU present                                */
#define __MPU_PRESENT             1U        /* MPU present                                */
#define __VTOR_PRESENT            1U        /* VTOR present                               */
#define __NVIC_PRIO_BITS          2U        /* Number of Bits used for Priority Levels    */
#define __Vendor_SysTickConfig    0U        /* Set to 1 if different SysTick Config is used */
#define USE_ASSERT                0U        /* Define to use Assert function or not       */

/** @} end of group CMSIS_Device */


#include <stdint.h>
#include "core_cm23.h"                      /* Processor and core peripherals             */
#include "system_M253.h"                    /* System Header                              */


/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/

/** @addtogroup REGISTER Control Register

  @{

*/

#include "bpwm_reg.h"
#include "canfd_reg.h"
#include "clk_reg.h"
#include "crc_reg.h"
#include "eadc_reg.h"
#include "fmc_reg.h"
#include "gpio_reg.h"
#include "i2c_reg.h"
#include "pdma_reg.h"
#include "rtc_reg.h"
#include "spi_reg.h"
#include "sys_reg.h"
#include "timer_reg.h"
#include "uart_reg.h"
#include "uuart_reg.h"
#include "ui2c_reg.h"
#include "uspi_reg.h"
#include "usbd_reg.h"
#include "wdt_reg.h"
#include "wwdt_reg.h"

/** @} end of REGISTER group */


/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/

/** @addtogroup PERIPHERAL_BASE Peripheral Memory Base
  Memory Mapped Structure for Series Peripheral
  @{
 */

/* Peripheral and SRAM base address */
#define FLASH_BASE            ((uint32_t)0x00000000UL)      /*!< Flash Base Address      */
#define SRAM_BASE             ((uint32_t)0x20000000UL)      /*!< SRAM Base Address       */
#define PERIPH_BASE           ((uint32_t)0x40000000UL)      /*!< Peripheral Base Address */

/* Peripheral memory map */
#define AHBPERIPH_BASE         PERIPH_BASE                  /*!< AHB Base Address        */
#define APBPERIPH_BASE        (PERIPH_BASE + 0x00040000UL)  /*!< APB Base Address        */

/*!< AHB peripherals */
#define SYS_BASE              (AHBPERIPH_BASE + 0x00000UL)
#define CLK_BASE              (AHBPERIPH_BASE + 0x00200UL)
#define NMI_BASE              (AHBPERIPH_BASE + 0x00300UL)
#define GPIO_BASE             (AHBPERIPH_BASE + 0x04000UL)
#define GPIOA_BASE            (AHBPERIPH_BASE + 0x04000UL)
#define GPIOB_BASE            (AHBPERIPH_BASE + 0x04040UL)
#define GPIOC_BASE            (AHBPERIPH_BASE + 0x04080UL)
#define GPIOD_BASE            (AHBPERIPH_BASE + 0x040C0UL)
#define GPIOE_BASE            (AHBPERIPH_BASE + 0x04100UL)
#define GPIOF_BASE            (AHBPERIPH_BASE + 0x04140UL)
#define GPIO_DBCTL_BASE       (AHBPERIPH_BASE + 0x04440UL)
#define GPIO_PIN_DATA_BASE    (AHBPERIPH_BASE + 0x04800UL)
#define PDMA_BASE             (AHBPERIPH_BASE + 0x08000UL)
#define FMC_BASE              (AHBPERIPH_BASE + 0x0C000UL)
#define CANFD_BASE            (AHBPERIPH_BASE + 0x20000UL)
#define CRC_BASE              (AHBPERIPH_BASE + 0x31000UL)


/*!< APB0 peripherals */
#define WDT_BASE              (APBPERIPH_BASE + 0x00000UL)
#define WWDT_BASE             (APBPERIPH_BASE + 0x00100UL)
#define TIMER01_BASE          (APBPERIPH_BASE + 0x10000UL)
#define BPWM0_BASE            (APBPERIPH_BASE + 0x1A000UL)
#define UART0_BASE            (APBPERIPH_BASE + 0x30000UL)
#define UART2_BASE            (APBPERIPH_BASE + 0x32000UL)
#define I2C0_BASE             (APBPERIPH_BASE + 0x40000UL)
#define USBD_BASE             (APBPERIPH_BASE + 0x80000UL)
#define USCI0_BASE            (APBPERIPH_BASE + 0x90000UL)
#define UART4_BASE            (APBPERIPH_BASE + 0x34000UL)


/*!< APB1 peripherals */
#define RTC_BASE              (APBPERIPH_BASE + 0x01000UL)
#define EADC_BASE             (APBPERIPH_BASE + 0x03000UL)
#define TIMER23_BASE          (APBPERIPH_BASE + 0x11000UL)
#define BPWM1_BASE            (APBPERIPH_BASE + 0x1B000UL)
#define SPI0_BASE             (APBPERIPH_BASE + 0x21000UL)
#define UART1_BASE            (APBPERIPH_BASE + 0x31000UL)
#define I2C1_BASE             (APBPERIPH_BASE + 0x41000UL)
#define UART3_BASE            (APBPERIPH_BASE + 0x33000UL)

/** @} end of group PERIPHERAL_BASE */


/******************************************************************************/
/*                         Peripheral declaration                             */
/******************************************************************************/

/** @addtogroup PERIPHERAL_DECLARATION Peripheral Pointer
  The Declaration of Peripheral Pointer
  @{
 */

/*!< AHB peripherals */
#define SYS                  ((SYS_T *)             SYS_BASE)
#define CLK                  ((CLK_T *)             CLK_BASE)
#define PA                   ((GPIO_T *)            GPIOA_BASE)
#define PB                   ((GPIO_T *)            GPIOB_BASE)
#define PC                   ((GPIO_T *)            GPIOC_BASE)
#define PD                   ((GPIO_T *)            GPIOD_BASE)
#define PE                   ((GPIO_T *)            GPIOE_BASE)
#define PF                   ((GPIO_T *)            GPIOF_BASE)
#define GPIO                 ((GPIO_DBCTL_T *)      GPIO_DBCTL_BASE)
#define PDMA                 ((PDMA_T *)            PDMA_BASE)
#define FMC                  ((FMC_T *)             FMC_BASE)
#define CANFD0               ((CANFD_T*)            CANFD_BASE)
#define CRC                  ((CRC_T *)             CRC_BASE)

/*!< APB0 peripherals */
#define WDT                  ((WDT_T *)             WDT_BASE)
#define WWDT                 ((WWDT_T *)            WWDT_BASE)
#define TIMER0               ((TIMER_T *)           TIMER01_BASE)
#define TIMER1               ((TIMER_T *)           (TIMER01_BASE + 0x100UL))
#define BPWM0                ((BPWM_T *)            BPWM0_BASE)
#define UART0                ((UART_T *)            UART0_BASE)
#define UART2                ((UART_T *)            UART2_BASE)
#define I2C0                 ((I2C_T *)             I2C0_BASE)
#define USBD                 ((USBD_T *)            USBD_BASE)
#define UI2C0                ((UI2C_T *)            USCI0_BASE)
#define USPI0                ((USPI_T *)            USCI0_BASE)
#define UUART0               ((UUART_T *)           USCI0_BASE)
#define UART4                ((UART_T *)            UART4_BASE)

/*!< APB1 peripherals */
#define RTC                  ((RTC_T *)             RTC_BASE)
#define EADC                 ((EADC_T *)            EADC_BASE)
#define TIMER2               ((TIMER_T *)           TIMER23_BASE)
#define TIMER3               ((TIMER_T *)           (TIMER23_BASE+ 0x100UL))
#define BPWM1                ((BPWM_T *)            BPWM1_BASE)
#define SPI0                 ((SPI_T *)             SPI0_BASE)
#define UART1                ((UART_T *)            UART1_BASE)
#define I2C1                 ((I2C_T *)             I2C1_BASE)
#define UART3                ((UART_T *)            UART3_BASE)

/** @} end of group PERIPHERAL_DECLARATION */


/* --------------------  End of section using anonymous unions  ------------------- */
#if   defined (__CC_ARM)
#pragma pop
#elif defined (__ICCARM__)
/* leave anonymous unions enabled */
#elif (__ARMCC_VERSION >= 6010050)
#pragma clang diagnostic pop
#elif defined (__GNUC__)
/* anonymous unions are enabled by default */
#elif defined (__TMS470__)
/* anonymous unions are enabled by default */
#elif defined (__TASKING__)
#pragma warning restore
#elif defined (__CSMC__)
/* anonymous unions are enabled by default */
#else
#warning Not supported compiler type
#endif

#ifdef __cplusplus
}

#endif


/*=============================================================================*/

/** @addtogroup IO_ROUTINE I/O Routines
  The Declaration of I/O Routines
  @{
 */

typedef volatile uint8_t  vu8;        ///< Define 8-bit unsigned volatile data type
typedef volatile uint16_t vu16;       ///< Define 16-bit unsigned volatile data type
typedef volatile uint32_t vu32;       ///< Define 32-bit unsigned volatile data type
typedef volatile uint64_t vu64;       ///< Define 64-bit unsigned volatile data type

/**
  * @brief Get a 8-bit unsigned value from specified address
  * @param[in] addr Address to get 8-bit data from
  * @return  8-bit unsigned value stored in specified address
  */
#define M8(addr)  (*((vu8  *) (addr)))

/**
  * @brief Get a 16-bit unsigned value from specified address
  * @param[in] addr Address to get 16-bit data from
  * @return  16-bit unsigned value stored in specified address
  * @note The input address must be 16-bit aligned
  */
#define M16(addr) (*((vu16 *) (addr)))

/**
  * @brief Get a 32-bit unsigned value from specified address
  * @param[in] addr Address to get 32-bit data from
  * @return  32-bit unsigned value stored in specified address
  * @note The input address must be 32-bit aligned
  */
#define M32(addr) (*((vu32 *) (addr)))

/**
  * @brief Set a 32-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 32-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 32-bit aligned
  */
#define outpw(port,value)   (*((volatile unsigned int *)(port))=(value))

/**
  * @brief Get a 32-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 32-bit data from
  * @return  32-bit unsigned value stored in specified I/O port
  * @note The input port must be 32-bit aligned
  */
#define inpw(port)          ((*((volatile unsigned int *)(port))))

/**
  * @brief Set a 16-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 16-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 16-bit aligned
  */
#define outps(port,value)   (*((volatile unsigned short *)(port))=(value))

/**
  * @brief Get a 16-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 16-bit data from
  * @return  16-bit unsigned value stored in specified I/O port
  * @note The input port must be 16-bit aligned
  */
#define inps(port)          ((*((volatile unsigned short *)(port))))

/**
  * @brief Set a 8-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 8-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  */
#define outpb(port,value)   (*((volatile unsigned char *)(port))=(value))

/**
  * @brief Get a 8-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 8-bit data from
  * @return  8-bit unsigned value stored in specified I/O port
  */
#define inpb(port)          ((*((volatile unsigned char *)(port))))

/**
  * @brief Set a 32-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 32-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 32-bit aligned
  */
#define outp32(port,value)  (*((volatile unsigned int *)(port))=(value))

/**
  * @brief Get a 32-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 32-bit data from
  * @return  32-bit unsigned value stored in specified I/O port
  * @note The input port must be 32-bit aligned
  */
#define inp32(port)         ((*((volatile unsigned int *)(port))))

/**
  * @brief Set a 16-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 16-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 16-bit aligned
  */
#define outp16(port,value)  (*((volatile unsigned short *)(port))=(value))

/**
  * @brief Get a 16-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 16-bit data from
  * @return  16-bit unsigned value stored in specified I/O port
  * @note The input port must be 16-bit aligned
  */
#define inp16(port)         ((*((volatile unsigned short *)(port))))

/**
  * @brief Set a 8-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 8-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  */
#define outp8(port,value)   (*((volatile unsigned char *)(port))=(value))

/**
  * @brief Get a 8-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 8-bit data from
  * @return  8-bit unsigned value stored in specified I/O port
  */
#define inp8(port)          ((*((volatile unsigned char *)(port))))

/** @} end of group IO_ROUTINE */


/******************************************************************************/
/*                Legacy Constants                                            */
/******************************************************************************/

/** @addtogroup Legacy_Constants Legacy Constants
  Legacy Constants
  @{
*/

#define E_SUCCESS     (0)

#ifndef NULL
    #define NULL      (0)                  ///< NULL pointer
#endif

#define TRUE          (1UL)                ///< Boolean true, define to use in API parameters or return value
#define FALSE         (0UL)                ///< Boolean false, define to use in API parameters or return value

#define ENABLE        (1UL)                ///< Enable, define to use in API parameters
#define DISABLE       (0UL)                ///< Disable, define to use in API parameters

/* Define one bit mask */
#define BIT0          (0x00000001UL)       ///< Bit 0 mask of an 32 bit integer
#define BIT1          (0x00000002UL)       ///< Bit 1 mask of an 32 bit integer
#define BIT2          (0x00000004UL)       ///< Bit 2 mask of an 32 bit integer
#define BIT3          (0x00000008UL)       ///< Bit 3 mask of an 32 bit integer
#define BIT4          (0x00000010UL)       ///< Bit 4 mask of an 32 bit integer
#define BIT5          (0x00000020UL)       ///< Bit 5 mask of an 32 bit integer
#define BIT6          (0x00000040UL)       ///< Bit 6 mask of an 32 bit integer
#define BIT7          (0x00000080UL)       ///< Bit 7 mask of an 32 bit integer
#define BIT8          (0x00000100UL)       ///< Bit 8 mask of an 32 bit integer
#define BIT9          (0x00000200UL)       ///< Bit 9 mask of an 32 bit integer
#define BIT10         (0x00000400UL)       ///< Bit 10 mask of an 32 bit integer
#define BIT11         (0x00000800UL)       ///< Bit 11 mask of an 32 bit integer
#define BIT12         (0x00001000UL)       ///< Bit 12 mask of an 32 bit integer
#define BIT13         (0x00002000UL)       ///< Bit 13 mask of an 32 bit integer
#define BIT14         (0x00004000UL)       ///< Bit 14 mask of an 32 bit integer
#define BIT15         (0x00008000UL)       ///< Bit 15 mask of an 32 bit integer
#define BIT16         (0x00010000UL)       ///< Bit 16 mask of an 32 bit integer
#define BIT17         (0x00020000UL)       ///< Bit 17 mask of an 32 bit integer
#define BIT18         (0x00040000UL)       ///< Bit 18 mask of an 32 bit integer
#define BIT19         (0x00080000UL)       ///< Bit 19 mask of an 32 bit integer
#define BIT20         (0x00100000UL)       ///< Bit 20 mask of an 32 bit integer
#define BIT21         (0x00200000UL)       ///< Bit 21 mask of an 32 bit integer
#define BIT22         (0x00400000UL)       ///< Bit 22 mask of an 32 bit integer
#define BIT23         (0x00800000UL)       ///< Bit 23 mask of an 32 bit integer
#define BIT24         (0x01000000UL)       ///< Bit 24 mask of an 32 bit integer
#define BIT25         (0x02000000UL)       ///< Bit 25 mask of an 32 bit integer
#define BIT26         (0x04000000UL)       ///< Bit 26 mask of an 32 bit integer
#define BIT27         (0x08000000UL)       ///< Bit 27 mask of an 32 bit integer
#define BIT28         (0x10000000UL)       ///< Bit 28 mask of an 32 bit integer
#define BIT29         (0x20000000UL)       ///< Bit 29 mask of an 32 bit integer
#define BIT30         (0x40000000UL)       ///< Bit 30 mask of an 32 bit integer
#define BIT31         (0x80000000UL)       ///< Bit 31 mask of an 32 bit integer

/* Byte Mask Definitions */
#define BYTE0_Msk     (0x000000FFUL)       ///< Mask to get bit0~bit7 from a 32 bit integer
#define BYTE1_Msk     (0x0000FF00UL)       ///< Mask to get bit8~bit15 from a 32 bit integer
#define BYTE2_Msk     (0x00FF0000UL)       ///< Mask to get bit16~bit23 from a 32 bit integer
#define BYTE3_Msk     (0xFF000000UL)       ///< Mask to get bit24~bit31 from a 32 bit integer

#define GET_BYTE0(u32Param)    (((u32Param) & BYTE0_Msk)      ) /*!< Extract Byte 0 (Bit  0~ 7) from parameter u32Param */
#define GET_BYTE1(u32Param)    (((u32Param) & BYTE1_Msk) >>  8) /*!< Extract Byte 1 (Bit  8~15) from parameter u32Param */
#define GET_BYTE2(u32Param)    (((u32Param) & BYTE2_Msk) >> 16) /*!< Extract Byte 2 (Bit 16~23) from parameter u32Param */
#define GET_BYTE3(u32Param)    (((u32Param) & BYTE3_Msk) >> 24) /*!< Extract Byte 3 (Bit 24~31) from parameter u32Param */

/** @} end of group Legacy_Constants */


/******************************************************************************/
/*                         Peripheral header files                            */
/******************************************************************************/
#include "sys.h"
#include "clk.h"
#include "uart.h"
#include "fmc.h"
#include "bpwm.h"
#include "canfd.h"
#include "crc.h"
#include "eadc.h"
#include "gpio.h"
#include "i2c.h"
#include "pdma.h"
#include "rtc.h"
#include "spi.h"
#include "timer.h"
#include "usci_i2c.h"
#include "usci_spi.h"
#include "usci_uart.h"
#include "usbd.h"
#include "wdt.h"
#include "wwdt.h"


#endif  /* __M253_H__ */
