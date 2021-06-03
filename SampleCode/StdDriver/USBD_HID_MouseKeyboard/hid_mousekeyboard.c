/******************************************************************************//**
 * @file     hid_mousekeyboard.c
 * @version  V0.10
 * @brief    M253 USBD mouse and keyboard sample file
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "NuMicro.h"
#include "hid_mousekeyboard.h"

int8_t g_i8MouseTable[] = { -16, -16, -16, 0, 16, 16, 16, 0};
uint8_t g_u8MouseIdx = 0;
uint8_t g_u8MoveLen, g_u8MouseMode = 1;

uint8_t volatile g_u8EP2Ready = 0;
uint8_t volatile g_u8EP3Ready = 0;
uint8_t volatile g_u8Suspend = 0;
uint8_t g_u8Idle = 0, g_u8Protocol = 0;

void USBD_IRQHandler(void)
{
    uint32_t u32IntSts = USBD_GET_INT_FLAG();
    uint32_t u32State = USBD_GET_BUS_STATE();

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_FLDET)
    {
        // Floating detect
        USBD_CLR_INT_FLAG(USBD_INTSTS_FLDET);

        if (USBD_IS_ATTACHED())
        {
            /* USB Plug In */
            USBD_ENABLE_USB();
        }
        else
        {
            /* USB Un-plug */
            USBD_DISABLE_USB();
        }
    }

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_BUS)
    {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_BUS);

        if (u32State & USBD_STATE_USBRST)
        {
            /* Bus reset */
            USBD_ENABLE_USB();
            USBD_SwReset();
            g_u8Suspend = 0;
        }

        if (u32State & USBD_STATE_SUSPEND)
        {
            /* Enter power down to wait USB attached */
            g_u8Suspend = 1;

            /* Enable USB but disable PHY */
            USBD_DISABLE_PHY();
        }

        if (u32State & USBD_STATE_RESUME)
        {
            /* Enable USB and enable PHY */
            USBD_ENABLE_USB();

            g_u8Suspend = 0;
        }

#ifdef SUPPORT_LPM

        if (u32State & USBD_STATE_L1SUSPEND)
        {
            /*
               TODO: Implement LPM SUSPEND flag here.
                     Recommend implementing the power-saving function in main loop.
            */
        }

        if (u32State & USBD_STATE_L1RESUME)
        {
            /*
               TODO: Implement LPM RESUME flag here.
            */
        }

#endif

    }

    if (u32IntSts & USBD_INTSTS_NEVWKIF_Msk)
    {
        /*Clear no-event wake up interrupt */
        USBD_CLR_INT_FLAG(USBD_INTSTS_NEVWKIF_Msk);
        /*
           TODO: Implement the function that will be executed when device is woken by non-USB event.
        */
    }

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_USB)
    {
        // USB event
        if (u32IntSts & USBD_INTSTS_SETUP)
        {
            // Setup packet
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SETUP);

            /* Clear the data IN/OUT ready flag of control end-points */
            USBD_STOP_TRANSACTION(EP0);
            USBD_STOP_TRANSACTION(EP1);

            USBD_ProcessSetupPacket();
        }

        // EP events
        if (u32IntSts & USBD_INTSTS_EP0)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP0);
            // control IN
            USBD_CtrlIn();
        }

        if (u32IntSts & USBD_INTSTS_EP1)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP1);

            // control OUT
            USBD_CtrlOut();
        }

        if (u32IntSts & USBD_INTSTS_EP2)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP2);
            // Interrupt IN
            EP2_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP3)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP3);
            // Interrupt IN
            EP3_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP4)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP4);
        }

        if (u32IntSts & USBD_INTSTS_EP5)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP5);
        }

        if (u32IntSts & USBD_INTSTS_EP6)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP6);
        }

        if (u32IntSts & USBD_INTSTS_EP7)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP7);
        }

        if (u32IntSts & USBD_INTSTS_EP8)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP8);
        }

        if (u32IntSts & USBD_INTSTS_EP9)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP9);
        }

        if (u32IntSts & USBD_INTSTS_EP10)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP10);
        }

        if (u32IntSts & USBD_INTSTS_EP11)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP11);
        }

    }

}

void EP2_Handler(void)  /* Interrupt IN handler */
{
    g_u8EP2Ready = 1;
}

void EP3_Handler(void)  /* Interrupt IN handler */
{
    g_u8EP3Ready = 1;
}


/*--------------------------------------------------------------------------*/
/**
  * @brief  USBD Endpoint Config.
  * @param  None.
  * @retval None.
  */
void HID_Init(void)
{
    /* Init setup packet buffer */
    /* Buffer range for setup packet -> [0 ~ 0x7] */
    USBD->STBUFSEG = SETUP_BUF_BASE;

    /*****************************************************/
    /* EP0 ==> control IN endpoint, address 0 */
    USBD_CONFIG_EP(EP0, USBD_CFG_CSTALL | USBD_CFG_EPMODE_IN | 0);
    /* Buffer range for EP0 */
    USBD_SET_EP_BUF_ADDR(EP0, EP0_BUF_BASE);

    /* EP1 ==> control OUT endpoint, address 0 */
    USBD_CONFIG_EP(EP1, USBD_CFG_CSTALL | USBD_CFG_EPMODE_OUT | 0);
    /* Buffer range for EP1 */
    USBD_SET_EP_BUF_ADDR(EP1, EP1_BUF_BASE);

    /*****************************************************/
    /* EP2 ==> Interrupt IN endpoint, address 1 */
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | HID_MOUSE_EP_NUM);
    /* Buffer range for EP2 */
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);

    /*****************************************************/
    /* EP3 ==> Interrupt IN endpoint, address 2 */
    USBD_CONFIG_EP(EP3, USBD_CFG_EPMODE_IN | HID_KB_EP_NUM);
    /* Buffer range for EP3 */
    USBD_SET_EP_BUF_ADDR(EP3, EP3_BUF_BASE);

    /* Start to send IN data */
    g_u8EP2Ready = 1;
    g_u8EP3Ready = 1;

}

void HID_ClassRequest(void)
{
    uint8_t buf[8];

    USBD_GetSetupPacket(buf);

    if (buf[0] & 0x80)   /* request data transfer direction */
    {
        /* Device to host */
        switch (buf[1])
        {
            case GET_IDLE:
            {
                USBD_SET_PAYLOAD_LEN(EP1, buf[6]);
                /* Data stage */
                USBD_PrepareCtrlIn(&g_u8Idle, buf[6]);
                /* Status stage */
                USBD_PrepareCtrlOut(0, 0);
                break;
            }

            case GET_PROTOCOL:
            {
                USBD_SET_PAYLOAD_LEN(EP1, buf[6]);
                /* Data stage */
                USBD_PrepareCtrlIn(&g_u8Protocol, buf[6]);
                /* Status stage */
                USBD_PrepareCtrlOut(0, 0);
                break;
            }

            case GET_REPORT:

            //             {
            //                 break;
            //             }
            default:
            {
                /* Setup error, stall the device */
                USBD_SetStall(EP0);
                USBD_SetStall(EP1);
                break;
            }
        }
    }
    else
    {
        /* Host to device */
        switch (buf[1])
        {
            case SET_REPORT:
            {
                if (buf[3] == 3)
                {
                    /* Request Type = Feature */
                    USBD_SET_DATA1(EP1);
                    USBD_SET_PAYLOAD_LEN(EP1, 0);
                }

                break;
            }

            case SET_IDLE:
            {
                g_u8Idle = buf[3];
                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                break;
            }

            case SET_PROTOCOL:
            {
                g_u8Protocol = buf[2];
                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                break;
            }

            default:
            {
                /* Stall */
                /* Setup error, stall the device */
                USBD_SetStall(EP0);
                USBD_SetStall(EP1);
                break;
            }
        }
    }
}

void HID_UpdateMouseData(void)
{
    uint8_t *pu8Buf;

    if (g_u8EP2Ready)
    {
        pu8Buf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2));
        g_u8MouseMode ^= 1;

        if (g_u8MouseMode)
        {
            if (g_u8MoveLen > 14)
            {
                /* Update new report data */
                pu8Buf[0] = 0x00;
                pu8Buf[1] = g_i8MouseTable[g_u8MouseIdx & 0x07];
                pu8Buf[2] = g_i8MouseTable[(g_u8MouseIdx + 2) & 0x07];
                pu8Buf[3] = 0x00;
                g_u8MouseIdx++;
                g_u8MoveLen = 0;
            }
        }
        else
        {
            pu8Buf[0] = pu8Buf[1] = pu8Buf[2] = pu8Buf[3] = 0;
        }

        g_u8MoveLen++;
        g_u8EP2Ready = 0;
        /* Set transfer length and trigger IN transfer */
        USBD_SET_PAYLOAD_LEN(EP2, 4);
    }
}

void HID_UpdateKbData(void)
{

    if (g_u8EP3Ready)
    {
        static uint32_t u32PreKey;
        uint8_t *pu8Buf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP3));

        /* If PB.15 = 0, just report it is key 'a' */
        uint32_t u32Key = (PB->PIN & (1 << 15)) ? 0 : 1;

        if (u32Key == 0)
        {
            int32_t i32i;

            for (i32i = 0; i32i < 8; i32i++)
            {
                pu8Buf[i32i] = 0;
            }

            if (u32Key != u32PreKey)
            {
                /* Trigger to note key release */
                USBD_SET_PAYLOAD_LEN(EP3, 8);
            }
        }
        else
        {
            u32PreKey = u32Key;
            pu8Buf[2] = 0x04; /* Key 'a' */
            USBD_SET_PAYLOAD_LEN(EP3, 8);
        }
    }
}
