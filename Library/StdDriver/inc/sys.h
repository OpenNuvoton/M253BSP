/****************************************************************************
 * @file     sys.h
 * @version  V1.10
 * @brief    M253 series SYS driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __SYS_H__
#define __SYS_H__


#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup SYS_Driver SYS Driver
  @{
*/

/** @addtogroup SYS_EXPORTED_CONSTANTS SYS Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Module Reset Control Resister constant definitions.                                                    */
/*---------------------------------------------------------------------------------------------------------*/

#define PDMA_RST    ((0x0<<24)|SYS_IPRST0_PDMARST_Pos)      /*!< PDMA reset is one of the SYS_ResetModule parameter */
#define CRC_RST     ((0x0<<24)|SYS_IPRST0_CRCRST_Pos)       /*!< CRC reset is one of the SYS_ResetModule parameter */
#define GPIO_RST    ((0x4<<24)|SYS_IPRST1_GPIORST_Pos)      /*!< GPIO reset is one of the SYS_ResetModule parameter */
#define TMR0_RST    ((0x4<<24)|SYS_IPRST1_TMR0RST_Pos)      /*!< TMR0 reset is one of the SYS_ResetModule parameter */
#define TMR1_RST    ((0x4<<24)|SYS_IPRST1_TMR1RST_Pos)      /*!< TMR1 reset is one of the SYS_ResetModule parameter */
#define TMR2_RST    ((0x4<<24)|SYS_IPRST1_TMR2RST_Pos)      /*!< TMR2 reset is one of the SYS_ResetModule parameter */
#define TMR3_RST    ((0x4<<24)|SYS_IPRST1_TMR3RST_Pos)      /*!< TMR3 reset is one of the SYS_ResetModule parameter */
#define I2C0_RST    ((0x4<<24)|SYS_IPRST1_I2C0RST_Pos)      /*!< I2C0 reset is one of the SYS_ResetModule parameter */
#define I2C1_RST    ((0x4<<24)|SYS_IPRST1_I2C1RST_Pos)      /*!< I2C1 reset is one of the SYS_ResetModule parameter */
#define SPI0_RST    ((0x4<<24)|SYS_IPRST1_SPI0RST_Pos)      /*!< SPI0 reset is one of the SYS_ResetModule parameter */
#define UART0_RST   ((0x4<<24)|SYS_IPRST1_UART0RST_Pos)     /*!< UART0 reset is one of the SYS_ResetModule parameter */
#define UART1_RST   ((0x4<<24)|SYS_IPRST1_UART1RST_Pos)     /*!< UART1 reset is one of the SYS_ResetModule parameter */
#define UART2_RST   ((0x4<<24)|SYS_IPRST1_UART2RST_Pos)     /*!< UART2 reset is one of the SYS_ResetModule parameter */
#define UART3_RST   ((0x4<<24)|SYS_IPRST1_UART3RST_Pos)     /*!< UART3 reset is one of the SYS_ResetModule parameter */
#define UART4_RST   ((0x4<<24)|SYS_IPRST1_UART4RST_Pos)     /*!< UART4 reset is one of the SYS_ResetModule parameter */
#define CANFD0_RST  ((0x4<<24)|SYS_IPRST1_CANFD0RST_Pos)    /*!< CANFD0 reset is one of the SYS_ResetModule parameter */
#define USBD_RST    ((0x4<<24)|SYS_IPRST1_USBDRST_Pos)      /*!< USBD reset is one of the SYS_ResetModule parameter */
#define EADC_RST    ((0x4<<24)|SYS_IPRST1_EADCRST_Pos)      /*!< EADC reset is one of the SYS_ResetModule parameter */

#define USCI0_RST   ((0x8<<24)|SYS_IPRST2_USCI0RST_Pos)     /*!< USCI0 reset is one of the SYS_ResetModule parameter */
#define BPWM0_RST   ((0x8<<24)|SYS_IPRST2_BPWM0RST_Pos)     /*!< BPWM0 reset is one of the SYS_ResetModule parameter */

/*---------------------------------------------------------------------------------------------------------*/
/*  Brown Out Detector Threshold Voltage Selection constant definitions.                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_BODCTL_BOD_RST_EN           (1UL<<SYS_BODCTL_BODRSTEN_Pos)      /*!< Brown-out Reset Enable */
#define SYS_BODCTL_BOD_INTERRUPT_EN     (0UL<<SYS_BODCTL_BODRSTEN_Pos)      /*!< Brown-out Interrupt Enable */
#define SYS_BODCTL_BODVL_4_4V           (0x7UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 4.4V */
#define SYS_BODCTL_BODVL_3_7V           (0x6UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 3.7V */
#define SYS_BODCTL_BODVL_3_0V           (0x5UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 3.0V */
#define SYS_BODCTL_BODVL_2_7V           (0x4UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.7V */
#define SYS_BODCTL_BODVL_2_4V           (0x3UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.4V */
#define SYS_BODCTL_BODVL_2_0V           (0x2UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.0V */
#define SYS_BODCTL_BODVL_1_8V           (0x1UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 1.8V */

#define SYS_BODCTL_LVRDGSEL_0CLK       (0x0UL<<SYS_BODCTL_LVRDGSEL_Pos)        /*!LVR Output De-glitch Time Without de-glitch function. */
#define SYS_BODCTL_LVRDGSEL_4CLK       (0x1UL<<SYS_BODCTL_LVRDGSEL_Pos)        /*!LVR Output De-glitch Time is selected 4MIRC clock*/
#define SYS_BODCTL_LVRDGSEL_8CLK       (0x2UL<<SYS_BODCTL_LVRDGSEL_Pos)        /*!LVR Output De-glitch Time is selected 8MIRC clock*/
#define SYS_BODCTL_LVRDGSEL_16CLK      (0x3UL<<SYS_BODCTL_LVRDGSEL_Pos)        /*!LVR Output De-glitch Time is selected 16MIRC clock*/
#define SYS_BODCTL_LVRDGSEL_32CLK      (0x4UL<<SYS_BODCTL_LVRDGSEL_Pos)        /*!LVR Output De-glitch Time is selected 32MIRC clock*/
#define SYS_BODCTL_LVRDGSEL_64CLK      (0x5UL<<SYS_BODCTL_LVRDGSEL_Pos)        /*!LVR Output De-glitch Time is selected 64MIRC clock*/
#define SYS_BODCTL_LVRDGSEL_128CLK     (0x6UL<<SYS_BODCTL_LVRDGSEL_Pos)        /*!LVR Output De-glitch Time is selected 128MIRC clock*/
#define SYS_BODCTL_LVRDGSEL_256CLK     (0x7UL<<SYS_BODCTL_LVRDGSEL_Pos)        /*!LVR Output De-glitch Time is selected 256MIRC clock*/

#define SYS_BODCTL_BODDGSEL_0CLK       (0x0UL<<SYS_BODCTL_BODDGSEL_Pos)        /*!BOD Output De-glitch Time is sampled by RC10K clock. */
#define SYS_BODCTL_BODDGSEL_4CLK       (0x1UL<<SYS_BODCTL_BODDGSEL_Pos)        /*!BOD Output De-glitch Time is selected 4HCLK clock*/
#define SYS_BODCTL_BODDGSEL_8CLK       (0x2UL<<SYS_BODCTL_BODDGSEL_Pos)        /*!BOD Output De-glitch Time is selected 8HCLK clock*/
#define SYS_BODCTL_BODDGSEL_16CLK      (0x3UL<<SYS_BODCTL_BODDGSEL_Pos)        /*!BOD Output De-glitch Time is selected 16HCLK clock*/
#define SYS_BODCTL_BODDGSEL_32CLK      (0x4UL<<SYS_BODCTL_BODDGSEL_Pos)        /*!BOD Output De-glitch Time is selected 32HCLK clock*/
#define SYS_BODCTL_BODDGSEL_64CLK      (0x5UL<<SYS_BODCTL_BODDGSEL_Pos)        /*!BOD Output De-glitch Time is selected 64HCLK clock*/
#define SYS_BODCTL_BODDGSEL_128CLK     (0x6UL<<SYS_BODCTL_BODDGSEL_Pos)        /*!BOD Output De-glitch Time is selected 128HCLK clock*/
#define SYS_BODCTL_BODDGSEL_256CLK     (0x7UL<<SYS_BODCTL_BODDGSEL_Pos)        /*!BOD Output De-glitch Time is selected 256HCLK clock*/

/*---------------------------------------------------------------------------------------------------------*/
/*  PLCTL constant definitions. (Write-Protection Register)                                                */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_PLCTL_PLSEL_PL0     (0x0UL<<SYS_PLCTL_PLSEL_Pos)   /*!< Set power level to power level 0 */
#define SYS_PLCTL_PLSEL_PL3     (0x3UL<<SYS_PLCTL_PLSEL_Pos)   /*!< Set power level to power level 3 */

/*---------------------------------------------------------------------------------------------------------*/
/*  PLSTS constant definitions. (Write-Protection Register)                                                */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_PLSTS_CURPL_PL0     (0x0UL<<SYS_PLSTS_CURPL_Pos)  /*!< Current Power Level is power level 0 */
#define SYS_PLSTS_CURPL_PL3     (0x3UL<<SYS_PLSTS_CURPL_Pos)  /*!< Current Power Level is power level 3 */

/*---------------------------------------------------------------------------------------------------------*/
/*  Multi-Function constant definitions.                                                                   */
/*---------------------------------------------------------------------------------------------------------*/

/* How to use below #define?

Example: If user want to set PA.1 as UART0_TXD and PA.0 as UART0_RXD in initial function,
         user can issue following command to achieve it.

         SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA0MFP_Msk) | SYS_GPA_MFPL_PA0MFP_UART0_RXD;
         SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA1MFP_Msk) | SYS_GPA_MFPL_PA1MFP_UART0_TXD;
*/
/* PA.0 MFP */
#define SYS_GPA_MFPL_PA0MFP_GPIO          (0x00UL<<SYS_GPA_MFPL_PA0MFP_Pos) /*!< GPA_MFPL PA0 setting for GPIO          */
#define SYS_GPA_MFPL_PA0MFP_SPI0_MOSI     (0x04UL<<SYS_GPA_MFPL_PA0MFP_Pos) /*!< GPA_MFPL PA0 setting for SPI0_MOSI     */
#define SYS_GPA_MFPL_PA0MFP_UART0_RXD     (0x07UL<<SYS_GPA_MFPL_PA0MFP_Pos) /*!< GPA_MFPL PA0 setting for UART0_RXD     */
#define SYS_GPA_MFPL_PA0MFP_UART1_nRTS    (0x08UL<<SYS_GPA_MFPL_PA0MFP_Pos) /*!< GPA_MFPL PA0 setting for UART1_nRTS    */
#define SYS_GPA_MFPL_PA0MFP_BPWM0_CH0     (0x0cUL<<SYS_GPA_MFPL_PA0MFP_Pos) /*!< GPA_MFPL PA0 setting for BPWM0_CH0     */

/* PA.1 MFP */
#define SYS_GPA_MFPL_PA1MFP_GPIO          (0x00UL<<SYS_GPA_MFPL_PA1MFP_Pos) /*!< GPA_MFPL PA1 setting for GPIO          */
#define SYS_GPA_MFPL_PA1MFP_SPI0_MISO     (0x04UL<<SYS_GPA_MFPL_PA1MFP_Pos) /*!< GPA_MFPL PA1 setting for SPI0_MISO     */
#define SYS_GPA_MFPL_PA1MFP_UART0_TXD     (0x07UL<<SYS_GPA_MFPL_PA1MFP_Pos) /*!< GPA_MFPL PA1 setting for UART0_TXD     */
#define SYS_GPA_MFPL_PA1MFP_UART1_nCTS    (0x08UL<<SYS_GPA_MFPL_PA1MFP_Pos) /*!< GPA_MFPL PA1 setting for UART1_nCTS    */
#define SYS_GPA_MFPL_PA1MFP_BPWM0_CH1     (0x0cUL<<SYS_GPA_MFPL_PA1MFP_Pos) /*!< GPA_MFPL PA1 setting for BPWM0_CH1     */

/* PA.2 MFP */
#define SYS_GPA_MFPL_PA2MFP_GPIO          (0x00UL<<SYS_GPA_MFPL_PA2MFP_Pos) /*!< GPA_MFPL PA2 setting for GPIO          */
#define SYS_GPA_MFPL_PA2MFP_UART4_RXD     (0x02UL<<SYS_GPA_MFPL_PA2MFP_Pos) /*!< GPA_MFPL PA2 setting for UART4_RXD     */
#define SYS_GPA_MFPL_PA2MFP_SPI0_CLK      (0x04UL<<SYS_GPA_MFPL_PA2MFP_Pos) /*!< GPA_MFPL PA2 setting for SPI0_CLK      */
#define SYS_GPA_MFPL_PA2MFP_UART1_RXD     (0x08UL<<SYS_GPA_MFPL_PA2MFP_Pos) /*!< GPA_MFPL PA2 setting for UART1_RXD     */
#define SYS_GPA_MFPL_PA2MFP_I2C1_SDA      (0x09UL<<SYS_GPA_MFPL_PA2MFP_Pos) /*!< GPA_MFPL PA2 setting for I2C1_SDA      */
#define SYS_GPA_MFPL_PA2MFP_BPWM0_CH2     (0x0cUL<<SYS_GPA_MFPL_PA2MFP_Pos) /*!< GPA_MFPL PA2 setting for BPWM0_CH2     */

/* PA.3 MFP */
#define SYS_GPA_MFPL_PA3MFP_GPIO          (0x00UL<<SYS_GPA_MFPL_PA3MFP_Pos) /*!< GPA_MFPL PA3 setting for GPIO          */
#define SYS_GPA_MFPL_PA3MFP_UART4_TXD     (0x02UL<<SYS_GPA_MFPL_PA3MFP_Pos) /*!< GPA_MFPL PA3 setting for UART4_TXD     */
#define SYS_GPA_MFPL_PA3MFP_SPI0_SS       (0x04UL<<SYS_GPA_MFPL_PA3MFP_Pos) /*!< GPA_MFPL PA3 setting for SPI0_SS       */
#define SYS_GPA_MFPL_PA3MFP_UART1_TXD     (0x08UL<<SYS_GPA_MFPL_PA3MFP_Pos) /*!< GPA_MFPL PA3 setting for UART1_TXD     */
#define SYS_GPA_MFPL_PA3MFP_I2C1_SCL      (0x09UL<<SYS_GPA_MFPL_PA3MFP_Pos) /*!< GPA_MFPL PA3 setting for I2C1_SCL      */
#define SYS_GPA_MFPL_PA3MFP_BPWM0_CH3     (0x0cUL<<SYS_GPA_MFPL_PA3MFP_Pos) /*!< GPA_MFPL PA3 setting for BPWM0_CH3     */
#define SYS_GPA_MFPL_PA3MFP_CLKO          (0x0eUL<<SYS_GPA_MFPL_PA3MFP_Pos) /*!< GPA_MFPL PA3 setting for CLKO          */

/* PA.4 MFP */
#define SYS_GPA_MFPL_PA4MFP_GPIO          (0x00UL<<SYS_GPA_MFPL_PA4MFP_Pos) /*!< GPA_MFPL PA4 setting for GPIO          */
#define SYS_GPA_MFPL_PA4MFP_CAN0_RXD      (0x02UL<<SYS_GPA_MFPL_PA4MFP_Pos) /*!< GPA_MFPL PA4 setting for CAN0_RXD      */
#define SYS_GPA_MFPL_PA4MFP_SPI0_I2SMCLK  (0x04UL<<SYS_GPA_MFPL_PA4MFP_Pos) /*!< GPA_MFPL PA4 setting for SPI0_I2SMCLK  */
#define SYS_GPA_MFPL_PA4MFP_UART0_nRTS    (0x07UL<<SYS_GPA_MFPL_PA4MFP_Pos) /*!< GPA_MFPL PA4 setting for UART0_nRTS    */
#define SYS_GPA_MFPL_PA4MFP_UART0_RXD     (0x08UL<<SYS_GPA_MFPL_PA4MFP_Pos) /*!< GPA_MFPL PA4 setting for UART0_RXD     */
#define SYS_GPA_MFPL_PA4MFP_I2C0_SDA      (0x09UL<<SYS_GPA_MFPL_PA4MFP_Pos) /*!< GPA_MFPL PA4 setting for I2C0_SDA      */
#define SYS_GPA_MFPL_PA4MFP_BPWM0_CH4     (0x0cUL<<SYS_GPA_MFPL_PA4MFP_Pos) /*!< GPA_MFPL PA4 setting for BPWM0_CH4     */

/* PA.5 MFP */
#define SYS_GPA_MFPL_PA5MFP_GPIO          (0x00UL<<SYS_GPA_MFPL_PA5MFP_Pos) /*!< GPA_MFPL PA5 setting for GPIO          */
#define SYS_GPA_MFPL_PA5MFP_CAN0_TXD      (0x02UL<<SYS_GPA_MFPL_PA5MFP_Pos) /*!< GPA_MFPL PA5 setting for CAN0_TXD      */
#define SYS_GPA_MFPL_PA5MFP_UART0_nCTS    (0x07UL<<SYS_GPA_MFPL_PA5MFP_Pos) /*!< GPA_MFPL PA5 setting for UART0_nCTS    */
#define SYS_GPA_MFPL_PA5MFP_UART0_TXD     (0x08UL<<SYS_GPA_MFPL_PA5MFP_Pos) /*!< GPA_MFPL PA5 setting for UART0_TXD     */
#define SYS_GPA_MFPL_PA5MFP_I2C0_SCL      (0x09UL<<SYS_GPA_MFPL_PA5MFP_Pos) /*!< GPA_MFPL PA5 setting for I2C0_SCL      */
#define SYS_GPA_MFPL_PA5MFP_BPWM0_CH5     (0x0cUL<<SYS_GPA_MFPL_PA5MFP_Pos) /*!< GPA_MFPL PA5 setting for BPWM0_CH5     */

/* PA.6 MFP */
#define SYS_GPA_MFPL_PA6MFP_GPIO          (0x00UL<<SYS_GPA_MFPL_PA6MFP_Pos) /*!< GPA_MFPL PA6 setting for GPIO          */
#define SYS_GPA_MFPL_PA6MFP_UART0_RXD     (0x07UL<<SYS_GPA_MFPL_PA6MFP_Pos) /*!< GPA_MFPL PA6 setting for UART0_RXD     */
#define SYS_GPA_MFPL_PA6MFP_I2C1_SDA      (0x08UL<<SYS_GPA_MFPL_PA6MFP_Pos) /*!< GPA_MFPL PA6 setting for I2C1_SDA      */
#define SYS_GPA_MFPL_PA6MFP_TM3           (0x0eUL<<SYS_GPA_MFPL_PA6MFP_Pos) /*!< GPA_MFPL PA6 setting for TM3           */
#define SYS_GPA_MFPL_PA6MFP_INT0          (0x0fUL<<SYS_GPA_MFPL_PA6MFP_Pos) /*!< GPA_MFPL PA6 setting for INT0          */

/* PA.7 MFP */
#define SYS_GPA_MFPL_PA7MFP_GPIO          (0x00UL<<SYS_GPA_MFPL_PA7MFP_Pos) /*!< GPA_MFPL PA7 setting for GPIO          */
#define SYS_GPA_MFPL_PA7MFP_UART0_TXD     (0x07UL<<SYS_GPA_MFPL_PA7MFP_Pos) /*!< GPA_MFPL PA7 setting for UART0_TXD     */
#define SYS_GPA_MFPL_PA7MFP_I2C1_SCL      (0x08UL<<SYS_GPA_MFPL_PA7MFP_Pos) /*!< GPA_MFPL PA7 setting for I2C1_SCL      */
#define SYS_GPA_MFPL_PA7MFP_TM2           (0x0eUL<<SYS_GPA_MFPL_PA7MFP_Pos) /*!< GPA_MFPL PA7 setting for TM2           */
#define SYS_GPA_MFPL_PA7MFP_INT1          (0x0fUL<<SYS_GPA_MFPL_PA7MFP_Pos) /*!< GPA_MFPL PA7 setting for INT1          */

/* PA.8 MFP */
#define SYS_GPA_MFPH_PA8MFP_GPIO          (0x00UL<<SYS_GPA_MFPH_PA8MFP_Pos) /*!< GPA_MFPH PA8 setting for GPIO          */
#define SYS_GPA_MFPH_PA8MFP_USCI0_CTL1    (0x06UL<<SYS_GPA_MFPH_PA8MFP_Pos) /*!< GPA_MFPH PA8 setting for USCI0_CTL1    */
#define SYS_GPA_MFPH_PA8MFP_UART1_RXD     (0x07UL<<SYS_GPA_MFPH_PA8MFP_Pos) /*!< GPA_MFPH PA8 setting for UART1_RXD     */
#define SYS_GPA_MFPH_PA8MFP_BPWM0_CH3     (0x09UL<<SYS_GPA_MFPH_PA8MFP_Pos) /*!< GPA_MFPH PA8 setting for BPWM0_CH3     */
#define SYS_GPA_MFPH_PA8MFP_TM3_EXT       (0x0dUL<<SYS_GPA_MFPH_PA8MFP_Pos) /*!< GPA_MFPH PA8 setting for TM3_EXT       */
#define SYS_GPA_MFPH_PA8MFP_INT4          (0x0fUL<<SYS_GPA_MFPH_PA8MFP_Pos) /*!< GPA_MFPH PA8 setting for INT4          */

/* PA.9 MFP */
#define SYS_GPA_MFPH_PA9MFP_GPIO          (0x00UL<<SYS_GPA_MFPH_PA9MFP_Pos) /*!< GPA_MFPH PA9 setting for GPIO          */
#define SYS_GPA_MFPH_PA9MFP_USCI0_DAT1    (0x06UL<<SYS_GPA_MFPH_PA9MFP_Pos) /*!< GPA_MFPH PA9 setting for USCI0_DAT1    */
#define SYS_GPA_MFPH_PA9MFP_UART1_TXD     (0x07UL<<SYS_GPA_MFPH_PA9MFP_Pos) /*!< GPA_MFPH PA9 setting for UART1_TXD     */
#define SYS_GPA_MFPH_PA9MFP_BPWM0_CH2     (0x09UL<<SYS_GPA_MFPH_PA9MFP_Pos) /*!< GPA_MFPH PA9 setting for BPWM0_CH2     */
#define SYS_GPA_MFPH_PA9MFP_TM2_EXT       (0x0dUL<<SYS_GPA_MFPH_PA9MFP_Pos) /*!< GPA_MFPH PA9 setting for TM2_EXT       */

/* PA.10 MFP */
#define SYS_GPA_MFPH_PA10MFP_GPIO         (0x00UL<<SYS_GPA_MFPH_PA10MFP_Pos)/*!< GPA_MFPH PA10 setting for GPIO         */
#define SYS_GPA_MFPH_PA10MFP_USCI0_DAT0   (0x06UL<<SYS_GPA_MFPH_PA10MFP_Pos)/*!< GPA_MFPH PA10 setting for USCI0_DAT0   */
#define SYS_GPA_MFPH_PA10MFP_BPWM0_CH1    (0x09UL<<SYS_GPA_MFPH_PA10MFP_Pos)/*!< GPA_MFPH PA10 setting for BPWM0_CH1    */
#define SYS_GPA_MFPH_PA10MFP_TM1_EXT      (0x0dUL<<SYS_GPA_MFPH_PA10MFP_Pos)/*!< GPA_MFPH PA10 setting for TM1_EXT      */

/* PA.11 MFP */
#define SYS_GPA_MFPH_PA11MFP_GPIO         (0x00UL<<SYS_GPA_MFPH_PA11MFP_Pos)/*!< GPA_MFPH PA11 setting for GPIO         */
#define SYS_GPA_MFPH_PA11MFP_USCI0_CLK    (0x06UL<<SYS_GPA_MFPH_PA11MFP_Pos)/*!< GPA_MFPH PA11 setting for USCI0_CLK    */
#define SYS_GPA_MFPH_PA11MFP_BPWM0_CH0    (0x09UL<<SYS_GPA_MFPH_PA11MFP_Pos)/*!< GPA_MFPH PA11 setting for BPWM0_CH0    */
#define SYS_GPA_MFPH_PA11MFP_TM0_EXT      (0x0dUL<<SYS_GPA_MFPH_PA11MFP_Pos)/*!< GPA_MFPH PA11 setting for TM0_EXT      */

/* PB.0 MFP */
#define SYS_GPB_MFPL_PB0MFP_GPIO          (0x00UL<<SYS_GPB_MFPL_PB0MFP_Pos) /*!< GPB_MFPL PB0 setting for GPIO          */
#define SYS_GPB_MFPL_PB0MFP_EADC0_CH0     (0x01UL<<SYS_GPB_MFPL_PB0MFP_Pos) /*!< GPB_MFPL PB0 setting for EADC0_CH0     */
#define SYS_GPB_MFPL_PB0MFP_UART2_RXD     (0x07UL<<SYS_GPB_MFPL_PB0MFP_Pos) /*!< GPB_MFPL PB0 setting for UART2_RXD     */
#define SYS_GPB_MFPL_PB0MFP_SPI0_I2SMCLK  (0x08UL<<SYS_GPB_MFPL_PB0MFP_Pos) /*!< GPB_MFPL PB0 setting for SPI0_I2SMCLK  */
#define SYS_GPB_MFPL_PB0MFP_I2C1_SDA      (0x09UL<<SYS_GPB_MFPL_PB0MFP_Pos) /*!< GPB_MFPL PB0 setting for I2C1_SDA      */

/* PB.1 MFP */
#define SYS_GPB_MFPL_PB1MFP_GPIO          (0x00UL<<SYS_GPB_MFPL_PB1MFP_Pos) /*!< GPB_MFPL PB1 setting for GPIO          */
#define SYS_GPB_MFPL_PB1MFP_EADC0_CH1     (0x01UL<<SYS_GPB_MFPL_PB1MFP_Pos) /*!< GPB_MFPL PB1 setting for EADC0_CH1     */
#define SYS_GPB_MFPL_PB1MFP_UART2_TXD     (0x07UL<<SYS_GPB_MFPL_PB1MFP_Pos) /*!< GPB_MFPL PB1 setting for UART2_TXD     */
#define SYS_GPB_MFPL_PB1MFP_I2C1_SCL      (0x09UL<<SYS_GPB_MFPL_PB1MFP_Pos) /*!< GPB_MFPL PB1 setting for I2C1_SCL      */

/* PB.2 MFP */
#define SYS_GPB_MFPL_PB2MFP_GPIO          (0x00UL<<SYS_GPB_MFPL_PB2MFP_Pos) /*!< GPB_MFPL PB2 setting for GPIO          */
#define SYS_GPB_MFPL_PB2MFP_EADC0_CH2     (0x01UL<<SYS_GPB_MFPL_PB2MFP_Pos) /*!< GPB_MFPL PB2 setting for EADC0_CH2     */
#define SYS_GPB_MFPL_PB2MFP_I2C1_SDA      (0x04UL<<SYS_GPB_MFPL_PB2MFP_Pos) /*!< GPB_MFPL PB2 setting for I2C1_SDA      */
#define SYS_GPB_MFPL_PB2MFP_UART1_RXD     (0x06UL<<SYS_GPB_MFPL_PB2MFP_Pos) /*!< GPB_MFPL PB2 setting for UART1_RXD     */
#define SYS_GPB_MFPL_PB2MFP_TM3           (0x0eUL<<SYS_GPB_MFPL_PB2MFP_Pos) /*!< GPB_MFPL PB2 setting for TM3           */
#define SYS_GPB_MFPL_PB2MFP_INT3          (0x0fUL<<SYS_GPB_MFPL_PB2MFP_Pos) /*!< GPB_MFPL PB2 setting for INT3          */

/* PB.3 MFP */
#define SYS_GPB_MFPL_PB3MFP_GPIO          (0x00UL<<SYS_GPB_MFPL_PB3MFP_Pos) /*!< GPB_MFPL PB3 setting for GPIO          */
#define SYS_GPB_MFPL_PB3MFP_EADC0_CH3     (0x01UL<<SYS_GPB_MFPL_PB3MFP_Pos) /*!< GPB_MFPL PB3 setting for EADC0_CH3     */
#define SYS_GPB_MFPL_PB3MFP_I2C1_SCL      (0x04UL<<SYS_GPB_MFPL_PB3MFP_Pos) /*!< GPB_MFPL PB3 setting for I2C1_SCL      */
#define SYS_GPB_MFPL_PB3MFP_UART1_TXD     (0x06UL<<SYS_GPB_MFPL_PB3MFP_Pos) /*!< GPB_MFPL PB3 setting for UART1_TXD     */
#define SYS_GPB_MFPL_PB3MFP_TM2           (0x0eUL<<SYS_GPB_MFPL_PB3MFP_Pos) /*!< GPB_MFPL PB3 setting for TM2           */
#define SYS_GPB_MFPL_PB3MFP_INT2          (0x0fUL<<SYS_GPB_MFPL_PB3MFP_Pos) /*!< GPB_MFPL PB3 setting for INT2          */

/* PB.4 MFP */
#define SYS_GPB_MFPL_PB4MFP_GPIO          (0x00UL<<SYS_GPB_MFPL_PB4MFP_Pos) /*!< GPB_MFPL PB4 setting for GPIO          */
#define SYS_GPB_MFPL_PB4MFP_EADC0_CH4     (0x01UL<<SYS_GPB_MFPL_PB4MFP_Pos) /*!< GPB_MFPL PB4 setting for EADC0_CH4     */
#define SYS_GPB_MFPL_PB4MFP_I2C0_SDA      (0x06UL<<SYS_GPB_MFPL_PB4MFP_Pos) /*!< GPB_MFPL PB4 setting for I2C0_SDA      */
#define SYS_GPB_MFPL_PB4MFP_UART2_RXD     (0x0dUL<<SYS_GPB_MFPL_PB4MFP_Pos) /*!< GPB_MFPL PB4 setting for UART2_RXD     */
#define SYS_GPB_MFPL_PB4MFP_TM1           (0x0eUL<<SYS_GPB_MFPL_PB4MFP_Pos) /*!< GPB_MFPL PB4 setting for TM1           */
#define SYS_GPB_MFPL_PB4MFP_INT1          (0x0fUL<<SYS_GPB_MFPL_PB4MFP_Pos) /*!< GPB_MFPL PB4 setting for INT1          */

/* PB.5 MFP */
#define SYS_GPB_MFPL_PB5MFP_GPIO          (0x00UL<<SYS_GPB_MFPL_PB5MFP_Pos) /*!< GPB_MFPL PB5 setting for GPIO          */
#define SYS_GPB_MFPL_PB5MFP_EADC0_CH5     (0x01UL<<SYS_GPB_MFPL_PB5MFP_Pos) /*!< GPB_MFPL PB5 setting for EADC0_CH5     */
#define SYS_GPB_MFPL_PB5MFP_I2C0_SCL      (0x06UL<<SYS_GPB_MFPL_PB5MFP_Pos) /*!< GPB_MFPL PB5 setting for I2C0_SCL      */
#define SYS_GPB_MFPL_PB5MFP_UART2_TXD     (0x0dUL<<SYS_GPB_MFPL_PB5MFP_Pos) /*!< GPB_MFPL PB5 setting for UART2_TXD     */
#define SYS_GPB_MFPL_PB5MFP_TM0           (0x0eUL<<SYS_GPB_MFPL_PB5MFP_Pos) /*!< GPB_MFPL PB5 setting for TM0           */
#define SYS_GPB_MFPL_PB5MFP_INT0          (0x0fUL<<SYS_GPB_MFPL_PB5MFP_Pos) /*!< GPB_MFPL PB5 setting for INT0          */

/* PB.6 MFP */
#define SYS_GPB_MFPL_PB6MFP_GPIO          (0x00UL<<SYS_GPB_MFPL_PB6MFP_Pos) /*!< GPB_MFPL PB6 setting for GPIO          */
#define SYS_GPB_MFPL_PB6MFP_EADC0_CH6     (0x01UL<<SYS_GPB_MFPL_PB6MFP_Pos) /*!< GPB_MFPL PB6 setting for EADC0_CH6     */
#define SYS_GPB_MFPL_PB6MFP_UART1_RXD     (0x06UL<<SYS_GPB_MFPL_PB6MFP_Pos) /*!< GPB_MFPL PB6 setting for UART1_RXD     */
#define SYS_GPB_MFPL_PB6MFP_INT4          (0x0dUL<<SYS_GPB_MFPL_PB6MFP_Pos) /*!< GPB_MFPL PB6 setting for INT4          */

/* PB.7 MFP */
#define SYS_GPB_MFPL_PB7MFP_GPIO          (0x00UL<<SYS_GPB_MFPL_PB7MFP_Pos) /*!< GPB_MFPL PB7 setting for GPIO          */
#define SYS_GPB_MFPL_PB7MFP_EADC0_CH7     (0x01UL<<SYS_GPB_MFPL_PB7MFP_Pos) /*!< GPB_MFPL PB7 setting for EADC0_CH7     */
#define SYS_GPB_MFPL_PB7MFP_UART1_TXD     (0x06UL<<SYS_GPB_MFPL_PB7MFP_Pos) /*!< GPB_MFPL PB7 setting for UART1_TXD     */
#define SYS_GPB_MFPL_PB7MFP_INT5          (0x0dUL<<SYS_GPB_MFPL_PB7MFP_Pos) /*!< GPB_MFPL PB7 setting for INT5          */

/* PB.12 MFP */
#define SYS_GPB_MFPH_PB12MFP_GPIO         (0x00UL<<SYS_GPB_MFPH_PB12MFP_Pos)/*!< GPB_MFPH PB12 setting for GPIO         */
#define SYS_GPB_MFPH_PB12MFP_EADC0_CH12   (0x01UL<<SYS_GPB_MFPH_PB12MFP_Pos)/*!< GPB_MFPH PB12 setting for EADC0_CH12   */
#define SYS_GPB_MFPH_PB12MFP_SPI0_MOSI    (0x04UL<<SYS_GPB_MFPH_PB12MFP_Pos)/*!< GPB_MFPH PB12 setting for SPI0_MOSI    */
#define SYS_GPB_MFPH_PB12MFP_USCI0_CLK    (0x05UL<<SYS_GPB_MFPH_PB12MFP_Pos)/*!< GPB_MFPH PB12 setting for USCI0_CLK    */
#define SYS_GPB_MFPH_PB12MFP_UART0_RXD    (0x06UL<<SYS_GPB_MFPH_PB12MFP_Pos)/*!< GPB_MFPH PB12 setting for UART0_RXD    */
#define SYS_GPB_MFPH_PB12MFP_UART3_nCTS   (0x07UL<<SYS_GPB_MFPH_PB12MFP_Pos)/*!< GPB_MFPH PB12 setting for UART3_nCTS   */
#define SYS_GPB_MFPH_PB12MFP_CAN0_RXD     (0x09UL<<SYS_GPB_MFPH_PB12MFP_Pos)/*!< GPB_MFPH PB12 setting for CAN0_RXD     */
#define SYS_GPB_MFPH_PB12MFP_TM3_EXT      (0x0dUL<<SYS_GPB_MFPH_PB12MFP_Pos)/*!< GPB_MFPH PB12 setting for TM3_EXT      */

/* PB.13 MFP */
#define SYS_GPB_MFPH_PB13MFP_GPIO         (0x00UL<<SYS_GPB_MFPH_PB13MFP_Pos)/*!< GPB_MFPH PB13 setting for GPIO         */
#define SYS_GPB_MFPH_PB13MFP_EADC0_CH13   (0x01UL<<SYS_GPB_MFPH_PB13MFP_Pos)/*!< GPB_MFPH PB13 setting for EADC0_CH13   */
#define SYS_GPB_MFPH_PB13MFP_SPI0_MISO    (0x04UL<<SYS_GPB_MFPH_PB13MFP_Pos)/*!< GPB_MFPH PB13 setting for SPI0_MISO    */
#define SYS_GPB_MFPH_PB13MFP_USCI0_DAT0   (0x05UL<<SYS_GPB_MFPH_PB13MFP_Pos)/*!< GPB_MFPH PB13 setting for USCI0_DAT0   */
#define SYS_GPB_MFPH_PB13MFP_UART0_TXD    (0x06UL<<SYS_GPB_MFPH_PB13MFP_Pos)/*!< GPB_MFPH PB13 setting for UART0_TXD    */
#define SYS_GPB_MFPH_PB13MFP_UART3_nRTS   (0x07UL<<SYS_GPB_MFPH_PB13MFP_Pos)/*!< GPB_MFPH PB13 setting for UART3_nRTS   */
#define SYS_GPB_MFPH_PB13MFP_CAN0_TXD     (0x09UL<<SYS_GPB_MFPH_PB13MFP_Pos)/*!< GPB_MFPH PB13 setting for CAN0_TXD     */
#define SYS_GPB_MFPH_PB13MFP_TM2_EXT      (0x0dUL<<SYS_GPB_MFPH_PB13MFP_Pos)/*!< GPB_MFPH PB13 setting for TM2_EXT      */

/* PB.14 MFP */
#define SYS_GPB_MFPH_PB14MFP_GPIO         (0x00UL<<SYS_GPB_MFPH_PB14MFP_Pos)/*!< GPB_MFPH PB14 setting for GPIO         */
#define SYS_GPB_MFPH_PB14MFP_EADC0_CH14   (0x01UL<<SYS_GPB_MFPH_PB14MFP_Pos)/*!< GPB_MFPH PB14 setting for EADC0_CH14   */
#define SYS_GPB_MFPH_PB14MFP_SPI0_CLK     (0x04UL<<SYS_GPB_MFPH_PB14MFP_Pos)/*!< GPB_MFPH PB14 setting for SPI0_CLK     */
#define SYS_GPB_MFPH_PB14MFP_USCI0_DAT1   (0x05UL<<SYS_GPB_MFPH_PB14MFP_Pos)/*!< GPB_MFPH PB14 setting for USCI0_DAT1   */
#define SYS_GPB_MFPH_PB14MFP_UART0_nRTS   (0x06UL<<SYS_GPB_MFPH_PB14MFP_Pos)/*!< GPB_MFPH PB14 setting for UART0_nRTS   */
#define SYS_GPB_MFPH_PB14MFP_UART3_RXD    (0x07UL<<SYS_GPB_MFPH_PB14MFP_Pos)/*!< GPB_MFPH PB14 setting for UART3_RXD    */
#define SYS_GPB_MFPH_PB14MFP_TM1_EXT      (0x0dUL<<SYS_GPB_MFPH_PB14MFP_Pos)/*!< GPB_MFPH PB14 setting for TM1_EXT      */
#define SYS_GPB_MFPH_PB14MFP_CLKO         (0x0eUL<<SYS_GPB_MFPH_PB14MFP_Pos)/*!< GPB_MFPH PB14 setting for CLKO         */

/* PB.15 MFP */
#define SYS_GPB_MFPH_PB15MFP_GPIO         (0x00UL<<SYS_GPB_MFPH_PB15MFP_Pos)/*!< GPB_MFPH PB15 setting for GPIO         */
#define SYS_GPB_MFPH_PB15MFP_EADC0_CH15   (0x01UL<<SYS_GPB_MFPH_PB15MFP_Pos)/*!< GPB_MFPH PB15 setting for EADC0_CH15   */
#define SYS_GPB_MFPH_PB15MFP_SPI0_SS      (0x04UL<<SYS_GPB_MFPH_PB15MFP_Pos)/*!< GPB_MFPH PB15 setting for SPI0_SS      */
#define SYS_GPB_MFPH_PB15MFP_USCI0_CTL1   (0x05UL<<SYS_GPB_MFPH_PB15MFP_Pos)/*!< GPB_MFPH PB15 setting for USCI0_CTL1   */
#define SYS_GPB_MFPH_PB15MFP_UART0_nCTS   (0x06UL<<SYS_GPB_MFPH_PB15MFP_Pos)/*!< GPB_MFPH PB15 setting for UART0_nCTS   */
#define SYS_GPB_MFPH_PB15MFP_UART3_TXD    (0x07UL<<SYS_GPB_MFPH_PB15MFP_Pos)/*!< GPB_MFPH PB15 setting for UART3_TXD    */
#define SYS_GPB_MFPH_PB15MFP_TM0_EXT      (0x0dUL<<SYS_GPB_MFPH_PB15MFP_Pos)/*!< GPB_MFPH PB15 setting for TM0_EXT      */

/* PC.0 MFP */
#define SYS_GPC_MFPL_PC0MFP_GPIO          (0x00UL<<SYS_GPC_MFPL_PC0MFP_Pos) /*!< GPC_MFPL PC0 setting for GPIO          */
#define SYS_GPC_MFPL_PC0MFP_UART2_RXD     (0x08UL<<SYS_GPC_MFPL_PC0MFP_Pos) /*!< GPC_MFPL PC0 setting for UART2_RXD     */
#define SYS_GPC_MFPL_PC0MFP_I2C0_SDA      (0x09UL<<SYS_GPC_MFPL_PC0MFP_Pos) /*!< GPC_MFPL PC0 setting for I2C0_SDA      */

/* PC.1 MFP */
#define SYS_GPC_MFPL_PC1MFP_GPIO          (0x00UL<<SYS_GPC_MFPL_PC1MFP_Pos) /*!< GPC_MFPL PC1 setting for GPIO          */
#define SYS_GPC_MFPL_PC1MFP_UART2_TXD     (0x08UL<<SYS_GPC_MFPL_PC1MFP_Pos) /*!< GPC_MFPL PC1 setting for UART2_TXD     */
#define SYS_GPC_MFPL_PC1MFP_I2C0_SCL      (0x09UL<<SYS_GPC_MFPL_PC1MFP_Pos) /*!< GPC_MFPL PC1 setting for I2C0_SCL      */

/* PC.2 MFP */
#define SYS_GPC_MFPL_PC2MFP_GPIO          (0x00UL<<SYS_GPC_MFPL_PC2MFP_Pos) /*!< GPC_MFPL PC2 setting for GPIO          */
#define SYS_GPC_MFPL_PC2MFP_UART2_nCTS    (0x08UL<<SYS_GPC_MFPL_PC2MFP_Pos) /*!< GPC_MFPL PC2 setting for UART2_nCTS    */
#define SYS_GPC_MFPL_PC2MFP_UART3_RXD     (0x0bUL<<SYS_GPC_MFPL_PC2MFP_Pos) /*!< GPC_MFPL PC2 setting for UART3_RXD     */

/* PC.3 MFP */
#define SYS_GPC_MFPL_PC3MFP_GPIO          (0x00UL<<SYS_GPC_MFPL_PC3MFP_Pos) /*!< GPC_MFPL PC3 setting for GPIO          */
#define SYS_GPC_MFPL_PC3MFP_UART2_nRTS    (0x08UL<<SYS_GPC_MFPL_PC3MFP_Pos) /*!< GPC_MFPL PC3 setting for UART2_nRTS    */
#define SYS_GPC_MFPL_PC3MFP_UART3_TXD     (0x0bUL<<SYS_GPC_MFPL_PC3MFP_Pos) /*!< GPC_MFPL PC3 setting for UART3_TXD     */

/* PC.4 MFP */
#define SYS_GPC_MFPL_PC4MFP_GPIO          (0x00UL<<SYS_GPC_MFPL_PC4MFP_Pos) /*!< GPC_MFPL PC4 setting for GPIO          */
#define SYS_GPC_MFPL_PC4MFP_UART2_RXD     (0x08UL<<SYS_GPC_MFPL_PC4MFP_Pos) /*!< GPC_MFPL PC4 setting for UART2_RXD     */
#define SYS_GPC_MFPL_PC4MFP_I2C1_SDA      (0x09UL<<SYS_GPC_MFPL_PC4MFP_Pos) /*!< GPC_MFPL PC4 setting for I2C1_SDA      */
#define SYS_GPC_MFPL_PC4MFP_CAN0_RXD      (0x0aUL<<SYS_GPC_MFPL_PC4MFP_Pos) /*!< GPC_MFPL PC4 setting for CAN0_RXD      */
#define SYS_GPC_MFPL_PC4MFP_UART4_RXD     (0x0bUL<<SYS_GPC_MFPL_PC4MFP_Pos) /*!< GPC_MFPL PC4 setting for UART4_RXD     */

/* PC.5 MFP */
#define SYS_GPC_MFPL_PC5MFP_GPIO          (0x00UL<<SYS_GPC_MFPL_PC5MFP_Pos) /*!< GPC_MFPL PC5 setting for GPIO          */
#define SYS_GPC_MFPL_PC5MFP_UART2_TXD     (0x08UL<<SYS_GPC_MFPL_PC5MFP_Pos) /*!< GPC_MFPL PC5 setting for UART2_TXD     */
#define SYS_GPC_MFPL_PC5MFP_I2C1_SCL      (0x09UL<<SYS_GPC_MFPL_PC5MFP_Pos) /*!< GPC_MFPL PC5 setting for I2C1_SCL      */
#define SYS_GPC_MFPL_PC5MFP_CAN0_TXD      (0x0aUL<<SYS_GPC_MFPL_PC5MFP_Pos) /*!< GPC_MFPL PC5 setting for CAN0_TXD      */
#define SYS_GPC_MFPL_PC5MFP_UART4_TXD     (0x0bUL<<SYS_GPC_MFPL_PC5MFP_Pos) /*!< GPC_MFPL PC5 setting for UART4_TXD     */

/* PC.14 MFP */
#define SYS_GPC_MFPH_PC14MFP_GPIO         (0x00UL<<SYS_GPC_MFPH_PC14MFP_Pos)/*!< GPC_MFPH PC14 setting for GPIO         */
#define SYS_GPC_MFPH_PC14MFP_SPI0_I2SMCLK (0x04UL<<SYS_GPC_MFPH_PC14MFP_Pos)/*!< GPC_MFPH PC14 setting for SPI0_I2SMCLK */
#define SYS_GPC_MFPH_PC14MFP_USCI0_CTL0   (0x05UL<<SYS_GPC_MFPH_PC14MFP_Pos)/*!< GPC_MFPH PC14 setting for USCI0_CTL0   */
#define SYS_GPC_MFPH_PC14MFP_TM1          (0x0dUL<<SYS_GPC_MFPH_PC14MFP_Pos)/*!< GPC_MFPH PC14 setting for TM1          */

/* PF.0 MFP */
#define SYS_GPF_MFPL_PF0MFP_GPIO          (0x00UL<<SYS_GPF_MFPL_PF0MFP_Pos) /*!< GPF_MFPL PF0 setting for GPIO          */
#define SYS_GPF_MFPL_PF0MFP_UART1_TXD     (0x02UL<<SYS_GPF_MFPL_PF0MFP_Pos) /*!< GPF_MFPL PF0 setting for UART1_TXD     */
#define SYS_GPF_MFPL_PF0MFP_I2C1_SCL      (0x03UL<<SYS_GPF_MFPL_PF0MFP_Pos) /*!< GPF_MFPL PF0 setting for I2C1_SCL      */
#define SYS_GPF_MFPL_PF0MFP_UART0_TXD     (0x04UL<<SYS_GPF_MFPL_PF0MFP_Pos) /*!< GPF_MFPL PF0 setting for UART0_TXD     */
#define SYS_GPF_MFPL_PF0MFP_ICE_DAT       (0x0eUL<<SYS_GPF_MFPL_PF0MFP_Pos) /*!< GPF_MFPL PF0 setting for ICE_DAT       */

/* PF.1 MFP */
#define SYS_GPF_MFPL_PF1MFP_GPIO          (0x00UL<<SYS_GPF_MFPL_PF1MFP_Pos) /*!< GPF_MFPL PF1 setting for GPIO          */
#define SYS_GPF_MFPL_PF1MFP_UART1_RXD     (0x02UL<<SYS_GPF_MFPL_PF1MFP_Pos) /*!< GPF_MFPL PF1 setting for UART1_RXD     */
#define SYS_GPF_MFPL_PF1MFP_I2C1_SDA      (0x03UL<<SYS_GPF_MFPL_PF1MFP_Pos) /*!< GPF_MFPL PF1 setting for I2C1_SDA      */
#define SYS_GPF_MFPL_PF1MFP_UART0_RXD     (0x04UL<<SYS_GPF_MFPL_PF1MFP_Pos) /*!< GPF_MFPL PF1 setting for UART0_RXD     */
#define SYS_GPF_MFPL_PF1MFP_ICE_CLK       (0x0eUL<<SYS_GPF_MFPL_PF1MFP_Pos) /*!< GPF_MFPL PF1 setting for ICE_CLK       */

/* PF.2 MFP */
#define SYS_GPF_MFPL_PF2MFP_GPIO          (0x00UL<<SYS_GPF_MFPL_PF2MFP_Pos) /*!< GPF_MFPL PF2 setting for GPIO          */
#define SYS_GPF_MFPL_PF2MFP_UART0_RXD     (0x03UL<<SYS_GPF_MFPL_PF2MFP_Pos) /*!< GPF_MFPL PF2 setting for UART0_RXD     */
#define SYS_GPF_MFPL_PF2MFP_I2C0_SDA      (0x04UL<<SYS_GPF_MFPL_PF2MFP_Pos) /*!< GPF_MFPL PF2 setting for I2C0_SDA      */
#define SYS_GPF_MFPL_PF2MFP_XT1_OUT       (0x0aUL<<SYS_GPF_MFPL_PF2MFP_Pos) /*!< GPF_MFPL PF2 setting for XT1_OUT       */

/* PF.3 MFP */
#define SYS_GPF_MFPL_PF3MFP_GPIO          (0x00UL<<SYS_GPF_MFPL_PF3MFP_Pos) /*!< GPF_MFPL PF3 setting for GPIO          */
#define SYS_GPF_MFPL_PF3MFP_UART0_TXD     (0x03UL<<SYS_GPF_MFPL_PF3MFP_Pos) /*!< GPF_MFPL PF3 setting for UART0_TXD     */
#define SYS_GPF_MFPL_PF3MFP_I2C0_SCL      (0x04UL<<SYS_GPF_MFPL_PF3MFP_Pos) /*!< GPF_MFPL PF3 setting for I2C0_SCL      */
#define SYS_GPF_MFPL_PF3MFP_XT1_IN        (0x0aUL<<SYS_GPF_MFPL_PF3MFP_Pos) /*!< GPF_MFPL PF3 setting for XT1_IN        */

/* PF.4 MFP */
#define SYS_GPF_MFPL_PF4MFP_GPIO          (0x00UL<<SYS_GPF_MFPL_PF4MFP_Pos) /*!< GPF_MFPL PF4 setting for GPIO          */
#define SYS_GPF_MFPL_PF4MFP_UART2_TXD     (0x02UL<<SYS_GPF_MFPL_PF4MFP_Pos) /*!< GPF_MFPL PF4 setting for UART2_TXD     */
#define SYS_GPF_MFPL_PF4MFP_UART2_nRTS    (0x04UL<<SYS_GPF_MFPL_PF4MFP_Pos) /*!< GPF_MFPL PF4 setting for UART2_nRTS    */
#define SYS_GPF_MFPL_PF4MFP_BPWM0_CH5     (0x08UL<<SYS_GPF_MFPL_PF4MFP_Pos) /*!< GPF_MFPL PF4 setting for BPWM0_CH5     */
#define SYS_GPF_MFPL_PF4MFP_X32_OUT       (0x0aUL<<SYS_GPF_MFPL_PF4MFP_Pos) /*!< GPF_MFPL PF4 setting for X32_OUT       */

/* PF.5 MFP */
#define SYS_GPF_MFPL_PF5MFP_GPIO          (0x00UL<<SYS_GPF_MFPL_PF5MFP_Pos) /*!< GPF_MFPL PF5 setting for GPIO          */
#define SYS_GPF_MFPL_PF5MFP_UART2_RXD     (0x02UL<<SYS_GPF_MFPL_PF5MFP_Pos) /*!< GPF_MFPL PF5 setting for UART2_RXD     */
#define SYS_GPF_MFPL_PF5MFP_UART2_nCTS    (0x04UL<<SYS_GPF_MFPL_PF5MFP_Pos) /*!< GPF_MFPL PF5 setting for UART2_nCTS    */
#define SYS_GPF_MFPL_PF5MFP_BPWM0_CH4     (0x08UL<<SYS_GPF_MFPL_PF5MFP_Pos) /*!< GPF_MFPL PF5 setting for BPWM0_CH4     */
#define SYS_GPF_MFPL_PF5MFP_X32_IN        (0x0aUL<<SYS_GPF_MFPL_PF5MFP_Pos) /*!< GPF_MFPL PF5 setting for X32_IN        */
#define SYS_GPF_MFPL_PF5MFP_EADC0_ST      (0x0bUL<<SYS_GPF_MFPL_PF5MFP_Pos) /*!< GPF_MFPL PF5 setting for EADC0_ST      */

/* PF.15 MFP */
#define SYS_GPF_MFPH_PF15MFP_GPIO         (0x00UL<<SYS_GPF_MFPH_PF15MFP_Pos)/*!< GPF_MFPH PF15 setting for GPIO         */
#define SYS_GPF_MFPH_PF15MFP_TM2          (0x0dUL<<SYS_GPF_MFPH_PF15MFP_Pos)/*!< GPF_MFPH PF15 setting for TM2          */
#define SYS_GPF_MFPH_PF15MFP_CLKO         (0x0eUL<<SYS_GPF_MFPH_PF15MFP_Pos)/*!< GPF_MFPH PF15 setting for CLKO         */
#define SYS_GPF_MFPH_PF15MFP_INT4         (0x0fUL<<SYS_GPF_MFPH_PF15MFP_Pos)/*!< GPF_MFPH PF15 setting for INT4         */

/*@}*/ /* end of group SYS_EXPORTED_CONSTANTS */

/** @addtogroup SYS_EXPORTED_FUNCTIONS SYS Exported Functions
  @{
*/


/**
  * @brief      Clear Brown-out detector interrupt flag
  * @param      None
  * @return     None
  * @details    This macro clear Brown-out detector interrupt flag.
  * \hideinitializer
  */
#define SYS_CLEAR_BOD_INT_FLAG()        (SYS->BODCTL |= SYS_BODCTL_BODIF_Msk)

/**
  * @brief      Set Brown-out detector function to normal mode
  * @param      None
  * @return     None
  * @details    This macro set Brown-out detector to normal mode.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_CLEAR_BOD_LPM()             (SYS->BODCTL &= ~SYS_BODCTL_BODLPM_Msk)

/**
  * @brief      Disable Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro disable Brown-out detector function.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_DISABLE_BOD()               (SYS->BODCTL &= ~SYS_BODCTL_BODEN_Msk)

/**
  * @brief      Enable Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detector function.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_ENABLE_BOD()                (SYS->BODCTL |= SYS_BODCTL_BODEN_Msk)

/**
  * @brief      Get Brown-out detector interrupt flag
  * @param      None
  * @retval     0   Brown-out detect interrupt flag is not set.
  * @retval     >=1 Brown-out detect interrupt flag is set.
  * @details    This macro get Brown-out detector interrupt flag.
  * \hideinitializer
  */
#define SYS_GET_BOD_INT_FLAG()          (SYS->BODCTL & SYS_BODCTL_BODIF_Msk)

/**
  * @brief      Get Brown-out detector status
  * @param      None
  * @retval     0   System voltage is higher than BOD threshold voltage setting or BOD function is disabled.
  * @retval     >=1 System voltage is lower than BOD threshold voltage setting.
  * @details    This macro get Brown-out detector output status.
  *             If the BOD function is disabled, this function always return 0.
  * \hideinitializer
  */
#define SYS_GET_BOD_OUTPUT()            (SYS->BODCTL & SYS_BODCTL_BODOUT_Msk)

/**
  * @brief      Disable Brown-out detector interrupt function
  * @param      None
  * @return     None
  * @details    This macro disable Brown-out detector interrupt function.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_DISABLE_BOD_RST()           (SYS->BODCTL &= ~SYS_BODCTL_BODRSTEN_Msk)

/**
  * @brief      Enable Brown-out detector reset function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detect reset function.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_ENABLE_BOD_RST()            (SYS->BODCTL |= SYS_BODCTL_BODRSTEN_Msk)

/**
  * @brief      Set Brown-out detector function low power mode
  * @param      None
  * @return     None
  * @details    This macro set Brown-out detector to low power mode.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_SET_BOD_LPM()               (SYS->BODCTL |= SYS_BODCTL_BODLPM_Msk)

/**
  * @brief      Set Brown-out detector voltage level
  * @param[in]  u32Level is Brown-out voltage level. Including :
  *             - \ref SYS_BODCTL_BODVL_4_4V
  *             - \ref SYS_BODCTL_BODVL_3_7V
  *             - \ref SYS_BODCTL_BODVL_3_0V
  *             - \ref SYS_BODCTL_BODVL_2_7V
  *             - \ref SYS_BODCTL_BODVL_2_4V
  *             - \ref SYS_BODCTL_BODVL_2_0V
  *             - \ref SYS_BODCTL_BODVL_1_8V
  * @return     None
  * @details    This macro set Brown-out detector voltage level.
  *             The write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_SET_BOD_LEVEL(u32Level)     (SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCTL_BODVL_Msk) | (u32Level))

/**
  * @brief      Get reset source is from Brown-out detector reset
  * @param      None
  * @retval     0   Previous reset source is not from Brown-out detector reset
  * @retval     >=1 Previous reset source is from Brown-out detector reset
  * @details    This macro get previous reset source is from Brown-out detect reset or not.
  * \hideinitializer
  */
#define SYS_IS_BOD_RST()                (SYS->RSTSTS & SYS_RSTSTS_BODRF_Msk)

/**
  * @brief      Get reset source is from CPU Lockup reset
  * @param      None
  * @retval     0   Previous reset source is not from CPU Lockup reset
  * @retval     >=1 Previous reset source is from CPU Lockup reset
  * @details    This macro get previous reset source is from CPU Lockup reset or not.
  * \hideinitializer
  */
#define SYS_IS_CPULK_RST()                (SYS->RSTSTS & SYS_RSTSTS_CPULKRF_Msk)

/**
  * @brief      Get reset source is from CPU reset
  * @param      None
  * @retval     0   Previous reset source is not from CPU reset
  * @retval     >=1 Previous reset source is from CPU reset
  * @details    This macro get previous reset source is from CPU reset.
  * \hideinitializer
  */
#define SYS_IS_CPU_RST()                (SYS->RSTSTS & SYS_RSTSTS_CPURF_Msk)

/**
  * @brief      Get reset source is from LVR Reset
  * @param      None
  * @retval     0   Previous reset source is not from Low-Voltage-Reset
  * @retval     >=1 Previous reset source is from Low-Voltage-Reset
  * @details    This macro get previous reset source is from Low-Voltage-Reset.
  * \hideinitializer
  */
#define SYS_IS_LVR_RST()                (SYS->RSTSTS & SYS_RSTSTS_LVRF_Msk)

/**
  * @brief      Get reset source is from Power-on Reset
  * @param      None
  * @retval     0   Previous reset source is not from Power-on Reset
  * @retval     >=1 Previous reset source is from Power-on Reset
  * @details    This macro get previous reset source is from Power-on Reset.
  * \hideinitializer
  */
#define SYS_IS_POR_RST()                (SYS->RSTSTS & SYS_RSTSTS_PORF_Msk)

/**
  * @brief      Get reset source is from reset pin reset
  * @param      None
  * @retval     0   Previous reset source is not from reset pin reset
  * @retval     >=1 Previous reset source is from reset pin reset
  * @details    This macro get previous reset source is from reset pin reset.
  * \hideinitializer
  */
#define SYS_IS_RSTPIN_RST()             (SYS->RSTSTS & SYS_RSTSTS_PINRF_Msk)

/**
  * @brief      Get reset source is from system reset
  * @param      None
  * @retval     0   Previous reset source is not from system reset
  * @retval     >=1 Previous reset source is from system reset
  * @details    This macro get previous reset source is from system reset.
  * \hideinitializer
  */
#define SYS_IS_SYSTEM_RST()             (SYS->RSTSTS & SYS_RSTSTS_SYSRF_Msk)

/**
  * @brief      Get reset source is from window watch dog reset
  * @param      None
  * @retval     0   Previous reset source is not from window watch dog reset
  * @retval     >=1 Previous reset source is from window watch dog reset
  * @details    This macro get previous reset source is from window watch dog reset.
  * \hideinitializer
  */
#define SYS_IS_WDT_RST()                (SYS->RSTSTS & SYS_RSTSTS_WDTRF_Msk)

/**
  * @brief      Disable Low-Voltage-Reset function
  * @param      None
  * @return     None
  * @details    This macro disable Low-Voltage-Reset function.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_DISABLE_LVR()               (SYS->BODCTL &= ~SYS_BODCTL_LVREN_Msk)

/**
  * @brief      Enable Low-Voltage-Reset function
  * @param      None
  * @return     None
  * @details    This macro enable Low-Voltage-Reset function.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_ENABLE_LVR()                (SYS->BODCTL |= SYS_BODCTL_LVREN_Msk)

/**
  * @brief      Disable Power-on Reset function
  * @param      None
  * @return     None
  * @details    This macro disable Power-on Reset function.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_DISABLE_POR()               (SYS->PORCTL0 = 0x5AA5)

/**
  * @brief      Enable Power-on Reset function
  * @param      None
  * @return     None
  * @details    This macro enable Power-on Reset function.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_ENABLE_POR()                (SYS->PORCTL0 = 0)

/**
  * @brief      Clear reset source flag
  * @param[in]  u32RstSrc is reset source. Including :
  *             - \ref SYS_RSTSTS_PORF_Msk
  *             - \ref SYS_RSTSTS_PINRF_Msk
  *             - \ref SYS_RSTSTS_WDTRF_Msk
  *             - \ref SYS_RSTSTS_LVRF_Msk
  *             - \ref SYS_RSTSTS_BODRF_Msk
  *             - \ref SYS_RSTSTS_SYSRF_Msk
  *             - \ref SYS_RSTSTS_CPURF_Msk
  *             - \ref SYS_RSTSTS_CPULKRF_Msk
  * @return     None
  * @details    This macro clear reset source flag.
  * \hideinitializer
  */
#define SYS_CLEAR_RST_SOURCE(u32RstSrc) ((SYS->RSTSTS) = (u32RstSrc) )


/*---------------------------------------------------------------------------------------------------------*/
/* static inline functions                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
/* Declare these inline functions here to avoid MISRA C 2004 rule 8.1 error */
__STATIC_INLINE void SYS_UnlockReg(void);
__STATIC_INLINE void SYS_LockReg(void);

/**
  * @brief      Disable register write-protection function
  * @param      None
  * @return     None
  * @details    This function disable register write-protection function.
  *             To unlock the protected register to allow write access.
  */
__STATIC_INLINE void SYS_UnlockReg(void)
{
    do
    {
        SYS->REGLCTL = 0x59UL;
        SYS->REGLCTL = 0x16UL;
        SYS->REGLCTL = 0x88UL;
    } while (SYS->REGLCTL == 0UL);
}

/**
  * @brief      Enable register write-protection function
  * @param      None
  * @return     None
  * @details    This function is used to enable register write-protection function.
  *             To lock the protected register to forbid write access.
  */
__STATIC_INLINE void SYS_LockReg(void)
{
    SYS->REGLCTL = 0x0UL;
}


void SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetBODStatus(void);
uint32_t SYS_GetResetSrc(void);
uint32_t SYS_IsRegLocked(void);
uint32_t SYS_ReadPDID(void);
void SYS_ResetChip(void);
void SYS_ResetCPU(void);
void SYS_ResetModule(uint32_t u32ModuleIndex);
void SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel);
void SYS_DisableBOD(void);
void SYS_SetPowerLevel(uint32_t u32PowerLevel);


/*@}*/ /* end of group SYS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group SYS_Driver */

/*@}*/ /* end of group Standard_Driver */


#ifdef __cplusplus
}
#endif

#endif /* __SYS_H__ */
