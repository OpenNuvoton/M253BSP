/******************************************************************************//**
 * @file     massstorage.h
 * @version  V0.10
 * @brief    USB mass storage header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __USBD_MASS_H__
#define __USBD_MASS_H__

/* Define the vendor id and product id */
#define USBD_VID        0x0416
#define USBD_PID        0xB009

/* Define EP maximum packet size */
#define EP0_MAX_PKT_SIZE    64
#define EP1_MAX_PKT_SIZE    EP0_MAX_PKT_SIZE
#define EP2_MAX_PKT_SIZE    64
#define EP3_MAX_PKT_SIZE    64

#define SETUP_BUF_BASE      0
#define SETUP_BUF_LEN       8
#define EP0_BUF_BASE        (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP0_BUF_LEN         EP0_MAX_PKT_SIZE
#define EP1_BUF_BASE        (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP1_BUF_LEN         EP1_MAX_PKT_SIZE
#define EP2_BUF_BASE        (EP1_BUF_BASE + EP1_BUF_LEN)
#define EP2_BUF_LEN         EP2_MAX_PKT_SIZE
#define EP3_BUF_BASE        (EP2_BUF_BASE + EP2_BUF_LEN)
#define EP3_BUF_LEN         EP3_MAX_PKT_SIZE

/* Define the interrupt In EP number */
#define BULK_IN_EP_NUM      0x02
#define BULK_OUT_EP_NUM     0x03

/* Define Descriptor information */
#define USBD_SELF_POWERED               0
#define USBD_REMOTE_WAKEUP              0
#define USBD_MAX_POWER                  50  /* The unit is in 2mA. ex: 50 * 2mA = 100mA */

#define LEN_CONFIG_AND_SUBORDINATE      (LEN_CONFIG+LEN_INTERFACE+LEN_ENDPOINT*2)

/*!<Define Mass Storage Class Specific Request */
#define BULK_ONLY_MASS_STORAGE_RESET    0xFF
#define GET_MAX_LUN                     0xFE

/*!<Define Mass Storage Signature */
#define CBW_SIGNATURE       0x43425355
#define CSW_SIGNATURE       0x53425355

/*!<Define Mass Storage UFI Command */
#define UFI_TEST_UNIT_READY                     0x00
#define UFI_REQUEST_SENSE                       0x03
#define UFI_INQUIRY                             0x12
#define UFI_MODE_SELECT_6                       0x15
#define UFI_MODE_SENSE_6                        0x1A
#define UFI_START_STOP                          0x1B
#define UFI_PREVENT_ALLOW_MEDIUM_REMOVAL        0x1E
#define UFI_READ_FORMAT_CAPACITY                0x23
#define UFI_READ_CAPACITY                       0x25
#define UFI_READ_10                             0x28
#define UFI_READ_12                             0xA8
#define UFI_READ_16                             0x9E
#define UFI_WRITE_10                            0x2A
#define UFI_WRITE_12                            0xAA
#define UFI_VERIFY_10                           0x2F
#define UFI_MODE_SELECT_10                      0x55
#define UFI_MODE_SENSE_10                       0x5A
#define UFI_READ_TOC                            0x43
#define UFI_GET_CONFIGURATION                   0x46
#define UFI_GET_EVENT_STATUS_NOTIFICATION       0x4A
#define UFI_SET_CDROM_SPEED                     0xBB
#define UFI_READ_CD                             0xBE

/*-----------------------------------------*/
#define BULK_CBW  0x00
#define BULK_IN   0x01
#define BULK_OUT  0x02
#define BULK_CSW  0x04
#define BULK_NORMAL 0xFF

static __INLINE uint32_t get_be32(uint8_t *buf)
{
    return ((uint32_t) buf[0] << 24) | ((uint32_t) buf[1] << 16) |
           ((uint32_t) buf[2] << 8) | ((uint32_t) buf[3]);
}

static __INLINE uint16_t get_be16(uint8_t *buf)
{
    return (((uint16_t) buf[0] << 8) | ((uint16_t) buf[1]));
}


/****************************************************************************//**/
/*                USBD Mass Storage Structure                                 */
/****************************************************************************//**/
/** @addtogroup USBD_Mass_Exported_Struct USBD Mass Exported Struct
    USBD Mass Specific Struct
  @{
*/

/*!<USB Mass Storage Class - Command Block Wrapper Structure */
struct CBW
{
    uint32_t  dCBWSignature;
    uint32_t  dCBWTag;
    uint32_t  dCBWDataTransferLength;
    uint8_t   bmCBWFlags;
    uint8_t   bCBWLUN;
    uint8_t   bCBWCBLength;
    uint8_t   u8OPCode;
    uint8_t   u8LUN;
    uint8_t   au8Data[14];
};

/*!<USB Mass Storage Class - Command Status Wrapper Structure */
struct CSW
{
    uint32_t  dCSWSignature;
    uint32_t  dCSWTag;
    uint32_t  dCSWDataResidue;
    uint8_t   bCSWStatus;
};

/*-------------------------------------------------------------*/
/* MSC Disk Image Definitions */
#define MSC_ImageSize   0x0000B000

extern const unsigned char eprom[MSC_ImageSize];   /* Disk Image */

#define MSC_MemorySize  MSC_ImageSize

#define DATA_FLASH_STORAGE_SIZE (MSC_ImageSize) /* Configure the DATA FLASH storage size */
#define MASS_BUFFER_SIZE    256               /* Mass Storage command buffer size */
#define STORAGE_BUFFER_SIZE 2048                /* Data transfer buffer size in 2048 bytes alignment */
#define CDROM_BLOCK_SIZE    2048                /* logic sector size */

extern uint32_t g_au32MassBlock[];
extern uint32_t g_au32StorageBlock[];

#define MassCMD_BUF        ((uint32_t)&g_au32MassBlock[0])
#define STORAGE_DATA_BUF   ((uint32_t)&g_au32StorageBlock[0])

/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
void DataFlashWrite(uint32_t addr, uint32_t size, uint32_t buffer);
void DataFlashRead(uint32_t addr, uint32_t size, uint32_t buffer);
void MSC_Init(void);
void MSC_RequestSense(void);
void MSC_ReadFormatCapacity(void);
void MSC_Read(void);
void MSC_ReadCapacity(void);
void MSC_Write(void);
uint32_t MSC_ModeSense10(void);
void MSC_ReadTrig(void);
void MSC_ClassRequest(void);

void MSC_ReadMedia(uint32_t addr, uint32_t size, uint8_t *buffer);
void MSC_WriteMedia(uint32_t addr, uint32_t size, uint8_t *buffer);

/*-------------------------------------------------------------*/
void MSC_AckCmd(void);
void MSC_ProcessCmd(void);
void EP2_Handler(void);
void EP3_Handler(void);

extern uint8_t volatile g_u8Suspend;

#endif  /* __USBD_MASS_H_ */
