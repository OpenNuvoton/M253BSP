/**************************************************************************//**
 * @file     wdt.h
 * @version  V0.10
 * @brief    M253 series WDT driver header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __WDT_H__
#define __WDT_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup WDT_Driver WDT Driver
  @{
*/

/** @addtogroup WDT_EXPORTED_CONSTANTS WDT Exported Constants
  @{
*/
/*---------------------------------------------------------------------------------------------------------*/
/*  WDT Time-out Interval Period Constant Definitions                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define WDT_TIMEOUT_2POW4           (0UL << WDT_CTL_TOUTSEL_Pos) /*!< Setting WDT time-out interval to  2^4 * WDT clocks */
#define WDT_TIMEOUT_2POW6           (1UL << WDT_CTL_TOUTSEL_Pos) /*!< Setting WDT time-out interval to  2^6 * WDT clocks */
#define WDT_TIMEOUT_2POW8           (2UL << WDT_CTL_TOUTSEL_Pos) /*!< Setting WDT time-out interval to  2^8 * WDT clocks */
#define WDT_TIMEOUT_2POW10          (3UL << WDT_CTL_TOUTSEL_Pos) /*!< Setting WDT time-out interval to 2^10 * WDT clocks */
#define WDT_TIMEOUT_2POW12          (4UL << WDT_CTL_TOUTSEL_Pos) /*!< Setting WDT time-out interval to 2^12 * WDT clocks */
#define WDT_TIMEOUT_2POW14          (5UL << WDT_CTL_TOUTSEL_Pos) /*!< Setting WDT time-out interval to 2^14 * WDT clocks */
#define WDT_TIMEOUT_2POW16          (6UL << WDT_CTL_TOUTSEL_Pos) /*!< Setting WDT time-out interval to 2^16 * WDT clocks */
#define WDT_TIMEOUT_2POW18          (7UL << WDT_CTL_TOUTSEL_Pos) /*!< Setting WDT time-out interval to 2^18 * WDT clocks */
#define WDT_TIMEOUT_2POW20          (8UL << WDT_CTL_TOUTSEL_Pos) /*!< Setting WDT time-out interval to 2^20 * WDT clocks */

/*---------------------------------------------------------------------------------------------------------*/
/*  WDT  Reset Delay Period Constant Definitions                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define WDT_RESET_DELAY_1026CLK     (0UL << WDT_ALTCTL_RSTDSEL_Pos) /*!< Setting WDT reset delay period to 1026 * WDT clocks */
#define WDT_RESET_DELAY_130CLK      (1UL << WDT_ALTCTL_RSTDSEL_Pos) /*!< Setting WDT reset delay period to  130 * WDT clocks */
#define WDT_RESET_DELAY_18CLK       (2UL << WDT_ALTCTL_RSTDSEL_Pos) /*!< Setting WDT reset delay period to   18 * WDT clocks */
#define WDT_RESET_DELAY_3CLK        (3UL << WDT_ALTCTL_RSTDSEL_Pos) /*!< Setting WDT reset delay period to    3 * WDT clocks */

/*---------------------------------------------------------------------------------------------------------*/
/*  WDT Free Reset Counter Keyword Constant Definitions                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define WDT_FREE_RESET_COUNTER_KEY (0x00005AA5)                 /*!< Fill this value to WDT_RSTCNT register to free reset WDT counter */

/** @} end of group WDT_EXPORTED_CONSTANTS */


/** @addtogroup WDT_EXPORTED_FUNCTIONS WDT Exported Functions
  @{
*/

/**
  * @brief      Clear WDT Reset System Flag
  *
  *
  *
  * @details    This macro clears WDT time-out reset system flag.
  *
  * \hideinitializer
  */
#define WDT_CLEAR_RESET_FLAG()          (WDT->CTL = (WDT->CTL & ~(WDT_CTL_IF_Msk | WDT_CTL_WKF_Msk)) | WDT_CTL_RSTF_Msk)

/**
  * @brief      Clear WDT Time-out Interrupt Flag
  *
  *
  *
  * @details    This macro clears WDT time-out interrupt flag.
  *
  * \hideinitializer
  */
#define WDT_CLEAR_TIMEOUT_INT_FLAG()    (WDT->CTL = (WDT->CTL & ~(WDT_CTL_RSTF_Msk | WDT_CTL_WKF_Msk)) | WDT_CTL_IF_Msk)

/**
  * @brief      Clear WDT Wake-up Flag
  *
  *
  *
  * @details    This macro clears WDT time-out wake-up system flag.
  *
  * \hideinitializer
  */
#define WDT_CLEAR_TIMEOUT_WAKEUP_FLAG() (WDT->CTL = (WDT->CTL & ~(WDT_CTL_RSTF_Msk | WDT_CTL_IF_Msk)) | WDT_CTL_WKF_Msk)

/**
  * @brief      Get WDT Time-out Reset Flag
  *
  *
  * @retval     0   WDT time-out reset system did not occur
  * @retval     1   WDT time-out reset system occurred
  *
  * @details    This macro indicates system has been reset by WDT time-out reset or not.
  *
  * \hideinitializer
  */
#define WDT_GET_RESET_FLAG()            ((WDT->CTL & WDT_CTL_RSTF_Msk)? 1UL : 0UL)

/**
  * @brief      Get WDT Time-out Interrupt Flag
  *
  *
  * @retval     0   WDT time-out interrupt did not occur
  * @retval     1   WDT time-out interrupt occurred
  *
  * @details    This macro indicates WDT time-out interrupt occurred or not.
  *
  * \hideinitializer
  */
#define WDT_GET_TIMEOUT_INT_FLAG()      ((WDT->CTL & WDT_CTL_IF_Msk)? 1UL : 0UL)

/**
  * @brief      Get WDT Time-out Wake-up Flag
  *
  *
  * @retval     0   WDT time-out interrupt does not cause CPU wake-up
  * @retval     1   WDT time-out interrupt event cause CPU wake-up
  *
  * @details    This macro indicates WDT time-out interrupt event has waked up system or not.
  *
  * \hideinitializer
  */
#define WDT_GET_TIMEOUT_WAKEUP_FLAG()   ((WDT->CTL & WDT_CTL_WKF_Msk)? 1UL : 0UL)

/**
  * @brief      Reset WDT Counter
  *
  *
  *
  * @details    This macro is used to reset the internal 18-bit WDT up counter value.
  * @note       If WDT is activated and time-out reset system function is enabled also, user should \n
  *             reset the 18-bit WDT up counter value to avoid generate WDT time-out reset signal to \n
  *             reset system before the WDT time-out reset delay period expires.
  *
  * \hideinitializer
  */
#define WDT_RESET_COUNTER()             (WDT->RSTCNT = WDT_FREE_RESET_COUNTER_KEY)
/* Declare these inline functions here to avoid MISRA C 2004 rule 8.1 error */
__STATIC_INLINE void WDT_Close(void);
__STATIC_INLINE void WDT_EnableInt(void);
__STATIC_INLINE void WDT_DisableInt(void);

/**
  * @brief      Stop WDT Counting
  *
  *
  *
  * @details    This function will stop WDT counting and disable WDT module.
  */
__STATIC_INLINE void WDT_Close(void)
{
    WDT->CTL = 0UL;
    return;
}

/**
  * @brief      Enable WDT Time-out Interrupt
  *
  *
  *
  * @details    This function will enable the WDT time-out interrupt function.
  */
__STATIC_INLINE void WDT_EnableInt(void)
{
    WDT->CTL |= WDT_CTL_INTEN_Msk;
    return;
}

/**
  * @brief      Disable WDT Time-out Interrupt
  *
  *
  *
  * @details    This function will disable the WDT time-out interrupt function.
  */
__STATIC_INLINE void WDT_DisableInt(void)
{
    /* Do not touch another write 1 clear bits */
    WDT->CTL &= ~(WDT_CTL_INTEN_Msk | WDT_CTL_RSTF_Msk | WDT_CTL_IF_Msk | WDT_CTL_WKF_Msk);
    return;
}

void WDT_Open(uint32_t u32TimeoutInterval, uint32_t u32ResetDelay, uint32_t u32EnableReset, uint32_t u32EnableWakeup);

/** @} end of group WDT_EXPORTED_FUNCTIONS */

/** @} end of group WDT_Driver */

/** @} end of group Standard_Driver */

#ifdef __cplusplus
}

#endif

#endif /* __WDT_H__ */

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
