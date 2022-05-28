/**
 * @brief usb-cdc-device.c merged four classes: 
 *          1. usbd_cdc_if
 *          2. usb_device
 *          3. usbd_cdc
 *          4. usbd_core
 *          5. usbd_ctlreq
 *          6. usbd_ioreq
 *          7. usbd_conf
 */

/* Includes ------------------------------------------------------------------*/
#include "usb-cdc-device.h"
// #include "usbd_desc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"


#define USBD_VID     1155
#define USBD_LANGID_STRING     1033
#define USBD_MANUFACTURER_STRING     "STMicroelectronics"
#define USBD_PID_FS     22336
#define USBD_PRODUCT_STRING_FS     "STM32 Virtual ComPort"
#define USBD_CONFIGURATION_STRING_FS     "CDC Config"
#define USBD_INTERFACE_STRING_FS     "CDC Interface"
#define USB_SIZ_BOS_DESC            0x0C


/* usbd_cdc_if.c BEGIN---- ---- ---- ---- */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USB Device Core handle declaration. */
USBD_Handle_t hUsbDeviceFS;

/**
  * @brief Device qualifier
  * @note  The device_qualifier descriptor describes information about 
  *          a high-speed capable device that would change if the device 
  *          were operating at the other speed. 
  *         For example, if the device is currently operating at full-speed, 
  *           the device_qualifier returns information about how it would 
  *           operate at high-speed and vice-versa. 
  */
/* USB Standard Device Descriptor */
__ALIGN_BEGIN uint8_t USBD_CDC_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  0x0A,
  0x06,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};


/* CDC interface class callbacks structure */
USBD_ClassTypeDef USBD_CDC =
{
  USBD_CDC_Init,
  USBD_CDC_DeInit,
  USBD_CDC_Setup,
  NULL,                 /* EP0_TxSent */
  USBD_CDC_EP0_RxReady,
  USBD_CDC_DataIn,
  USBD_CDC_DataOut,
  NULL,
  NULL,
  NULL,
  USBD_CDC_GetHSCfgDesc,
  USBD_CDC_GetFSCfgDesc,
  USBD_CDC_GetOtherSpeedCfgDesc,
  USBD_CDC_GetDeviceQualifierDescriptor,
};

/* USB CDC device Configuration Descriptor */
__ALIGN_BEGIN uint8_t USBD_CDC_CfgDesc[USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END =
{
  /* Configuration Descriptor */
  0x09,                                       /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,                /* bDescriptorType: Configuration */
  USB_CDC_CONFIG_DESC_SIZ,                    /* wTotalLength */
  0x00,
  0x02,                                       /* bNumInterfaces: 2 interfaces */
  0x01,                                       /* bConfigurationValue: Configuration value */
  0x00,                                       /* iConfiguration: Index of string descriptor */
  0xC0,                                       /* bmAttributes: Bus is self-powered. */
  USBD_MAX_POWER,                             /* MaxPower (mA) */

  /* Interface Descriptor */
  0x09,                                       /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: Interface */
  /* Interface descriptor type */
  0x00,                                       /* bInterfaceNumber: Number of Interface */
  0x00,                                       /* bAlternateSetting: Alternate setting */
  0x01,                                       /* bNumEndpoints: One endpoint used */
  0x02,                                       /* bInterfaceClass: Communication Interface Class */
  0x02,                                       /* bInterfaceSubClass: Abstract Control Model */
  0x01,                                       /* bInterfaceProtocol: Common AT commands */
  0x00,                                       /* iInterface */

  /* Header Functional Descriptor */
  0x05,                                       /* bLength: Endpoint Descriptor size */
  0x24,                                       /* bDescriptorType: CS_INTERFACE */
  0x00,                                       /* bDescriptorSubtype: Header Func Desc */
  0x10,                                       /* bcdCDC: spec release number */
  0x01,

  /* Call Management Functional Descriptor */
  0x05,                                       /* bFunctionLength */
  0x24,                                       /* bDescriptorType: CS_INTERFACE */
  0x01,                                       /* bDescriptorSubtype: Call Management Func Desc */
  0x00,                                       /* bmCapabilities: D0+D1 */
  0x01,                                       /* bDataInterface */

  /* ACM Functional Descriptor */
  0x04,                                       /* bFunctionLength */
  0x24,                                       /* bDescriptorType: CS_INTERFACE */
  0x02,                                       /* bDescriptorSubtype: Abstract Control Management desc */
  0x02,                                       /* bmCapabilities */

  /* Union Functional Descriptor */
  0x05,                                       /* bFunctionLength */
  0x24,                                       /* bDescriptorType: CS_INTERFACE */
  0x06,                                       /* bDescriptorSubtype: Union func desc */
  0x00,                                       /* bMasterInterface: Communication class interface */
  0x01,                                       /* bSlaveInterface0: Data Class Interface */

  /* Endpoint 2 Descriptor */
  0x07,                                       /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
  CDC_CMD_EP,                                 /* bEndpointAddress */
  0x03,                                       /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),                /* wMaxPacketSize */
  HIBYTE(CDC_CMD_PACKET_SIZE),
  CDC_FS_BINTERVAL,                           /* bInterval */
  /*---------------------------------------------------------------------------*/

  /* Data class interface descriptor */
  0x09,                                       /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: */
  0x01,                                       /* bInterfaceNumber: Number of Interface */
  0x00,                                       /* bAlternateSetting: Alternate setting */
  0x02,                                       /* bNumEndpoints: Two endpoints used */
  0x0A,                                       /* bInterfaceClass: CDC */
  0x00,                                       /* bInterfaceSubClass */
  0x00,                                       /* bInterfaceProtocol */
  0x00,                                       /* iInterface */

  /* Endpoint OUT Descriptor */
  0x07,                                       /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
  CDC_OUT_EP,                                 /* bEndpointAddress */
  0x02,                                       /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),        /* wMaxPacketSize */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
  0x00,                                       /* bInterval */

  /* Endpoint IN Descriptor */
  0x07,                                       /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
  CDC_IN_EP,                                  /* bEndpointAddress */
  0x02,                                       /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),        /* wMaxPacketSize */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
  0x00                                        /* bInterval */
};


USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t CDC_DeInit_FS(void)
{
  return (USBD_OK);
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:
    break;
    case CDC_GET_ENCAPSULATED_RESPONSE:
    break;
    case CDC_SET_COMM_FEATURE:
    break;
    case CDC_GET_COMM_FEATURE:
    break;
    case CDC_CLEAR_COMM_FEATURE:
    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:
    break;
    case CDC_GET_LINE_CODING:
    break;
    case CDC_SET_CONTROL_LINE_STATE:
    break;
    case CDC_SEND_BREAK:
    break;
  default:
    break;
  }
  return (USBD_OK);
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  uint8_t received_msg[64];
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);

  memset (received_msg, '\0', 64);  // clear the buffer
  uint8_t len = (uint8_t)*Len;
  memcpy(received_msg, Buf, len);  // copy the data to the buffer

  strcat((char*) received_msg, (char *)" echoed.\n");
  memset(Buf, '\0', len);   // clear the Buf also
  usbd_transmit(received_msg, sizeof(received_msg));
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  Trasmit Data to send over USB IN endpoint are sent over CDC interface
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval void
  */
void usbd_transmit(uint8_t* Buf, uint16_t Len)
{
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);

  // start transmission.
  USBD_Handle_t* p_usbd_instance = &hUsbDeviceFS;
  
  USBD_CDC_Buffer_t *hcdc = (USBD_CDC_Buffer_t *)p_usbd_instance->pClassDataCmsit;
  if (hcdc->TxState == 0U)
  {
    /* Tx Transfer in progress */
    hcdc->TxState = 1U;
    /* Update the packet total length */
    p_usbd_instance->ep_in[CDC_IN_EP & 0xFU].total_length = hcdc->TxLength;
    /* Transmit next packet */
    USBD_LL_Transmit(p_usbd_instance, CDC_IN_EP, hcdc->TxBuffer, hcdc->TxLength);
  }
}

/**
  * @brief  Data transmitted callback
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  /* USER CODE END 13 */
  return result;
}


/**
  * @brief Init USB device Library, add supported class and start the library
  */
void usb_device_init(void)
{
  USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS);
  USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC); // usbd_core.c
  USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS);
  USBD_Start(&hUsbDeviceFS);  /* p_usbd_handle->pData has type void */
}

/**
  * @brief  Initialize the CDC interface
  * @param  p_usbd_instance: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
uint8_t USBD_CDC_Init(USBD_Handle_t *p_usbd_instance, uint8_t cfgidx)
{
  UNUSED(cfgidx);
  USBD_CDC_Buffer_t *hcdc;

  hcdc = (USBD_CDC_Buffer_t *)USBD_malloc(sizeof(USBD_CDC_Buffer_t));

  if (hcdc == NULL)
  {
    p_usbd_instance->pClassDataCmsit = NULL;
    return (uint8_t)USBD_EMEM;
  }

  memset(hcdc, 0, sizeof(USBD_CDC_Buffer_t));

  p_usbd_instance->pClassDataCmsit = (void *)hcdc;
  p_usbd_instance->pClassData = p_usbd_instance->pClassDataCmsit;

  /* High Speed */
  if (p_usbd_instance->dev_speed == USBD_SPEED_HIGH)
  {
    /* Open EP IN */
    USBD_LL_OpenEP(p_usbd_instance, CDC_IN_EP, USBD_EP_TYPE_BULK,
                         CDC_DATA_HS_IN_PACKET_SIZE);

    p_usbd_instance->ep_in[CDC_IN_EP & 0xFU].is_used = 1U;

    /* Open EP OUT */
    USBD_LL_OpenEP(p_usbd_instance, CDC_OUT_EP, USBD_EP_TYPE_BULK,
                         CDC_DATA_HS_OUT_PACKET_SIZE);

    p_usbd_instance->ep_out[CDC_OUT_EP & 0xFU].is_used = 1U;

    /* Set bInterval for CDC CMD Endpoint */
    p_usbd_instance->ep_in[CDC_CMD_EP & 0xFU].bInterval = CDC_HS_BINTERVAL;
  }
  /* Low/Full Speed */
  else
  {
    /* Open EP IN */
    USBD_LL_OpenEP(p_usbd_instance, CDC_IN_EP, USBD_EP_TYPE_BULK,
                         CDC_DATA_FS_IN_PACKET_SIZE);

    p_usbd_instance->ep_in[CDC_IN_EP & 0xFU].is_used = 1U;

    /* Open EP OUT */
    USBD_LL_OpenEP(p_usbd_instance, CDC_OUT_EP, USBD_EP_TYPE_BULK,
                         CDC_DATA_FS_OUT_PACKET_SIZE);

    p_usbd_instance->ep_out[CDC_OUT_EP & 0xFU].is_used = 1U;

    /* Set bInterval for CMD Endpoint */
    p_usbd_instance->ep_in[CDC_CMD_EP & 0xFU].bInterval = CDC_FS_BINTERVAL;
  }

  /* Open Command IN EP */
  USBD_LL_OpenEP(p_usbd_instance, CDC_CMD_EP, USBD_EP_TYPE_INTR, CDC_CMD_PACKET_SIZE);
  p_usbd_instance->ep_in[CDC_CMD_EP & 0xFU].is_used = 1U;

  hcdc->RxBuffer = NULL;

  /* Init  physical Interface components */
  ((USBD_CDC_ItfTypeDef *)p_usbd_instance->pUserData)->Init();

  /* Init Xfer states */
  hcdc->TxState = 0U;
  hcdc->RxState = 0U;

  if (hcdc->RxBuffer == NULL)
  {
    return (uint8_t)USBD_EMEM;
  }

  if (p_usbd_instance->dev_speed == USBD_SPEED_HIGH)
  {
    /* Prepare Out endpoint to receive next packet */
    USBD_LL_PrepareReceive(p_usbd_instance, CDC_OUT_EP, hcdc->RxBuffer,
                                 CDC_DATA_HS_OUT_PACKET_SIZE);
  }
  else
  {
    /* Prepare Out endpoint to receive next packet */
    USBD_LL_PrepareReceive(p_usbd_instance, CDC_OUT_EP, hcdc->RxBuffer,
                                 CDC_DATA_FS_OUT_PACKET_SIZE);
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_Init
  *         DeInitialize the CDC layer
  * @param  p_usbd_instance: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
uint8_t USBD_CDC_DeInit(USBD_Handle_t *p_usbd_instance, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  /* Close EP IN */
  USBD_LL_CloseEP(p_usbd_instance, CDC_IN_EP);
  p_usbd_instance->ep_in[CDC_IN_EP & 0xFU].is_used = 0U;

  /* Close EP OUT */
  USBD_LL_CloseEP(p_usbd_instance, CDC_OUT_EP);
  p_usbd_instance->ep_out[CDC_OUT_EP & 0xFU].is_used = 0U;

  /* Close Command IN EP */
  USBD_LL_CloseEP(p_usbd_instance, CDC_CMD_EP);
  p_usbd_instance->ep_in[CDC_CMD_EP & 0xFU].is_used = 0U;
  p_usbd_instance->ep_in[CDC_CMD_EP & 0xFU].bInterval = 0U;

  /* DeInit  physical Interface components */
  if (p_usbd_instance->pClassDataCmsit != NULL)
  {
    ((USBD_CDC_ItfTypeDef *)p_usbd_instance->pUserData)->DeInit();
    USBD_static_free(p_usbd_instance->pClassDataCmsit);
    p_usbd_instance->pClassDataCmsit = NULL;
    p_usbd_instance->pClassData = NULL;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_Setup
  *         Handle the CDC specific requests
  * @param  p_usbd_instance: instance
  * @param  req: usb requests
  * @retval status
  */
uint8_t USBD_CDC_Setup(USBD_Handle_t *p_usbd_instance,
                              USBD_SetupReqTypedef *req)
{
  USBD_CDC_Buffer_t *hcdc = (USBD_CDC_Buffer_t *)p_usbd_instance->pClassDataCmsit;
  uint16_t len;
  uint8_t ifalt = 0U;
  uint16_t status_info = 0U;
  USBD_StatusTypeDef ret = USBD_OK;

  if (hcdc == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
      if (req->wLength != 0U)
      {
        if ((req->bmRequest & 0x80U) != 0U)
        {
          ((USBD_CDC_ItfTypeDef *)p_usbd_instance->pUserData)->Control(req->bRequest,
                                                                           (uint8_t *)hcdc->data,
                                                                           req->wLength);

          len = MIN(CDC_REQ_MAX_DATA_SIZE, req->wLength);
          USBD_CtlSendData(p_usbd_instance, (uint8_t *)hcdc->data, len);
        }
        else
        {
          hcdc->CmdOpCode = req->bRequest;
          hcdc->CmdLength = (uint8_t)MIN(req->wLength, USB_MAX_EP0_SIZE);

          USBD_CtlPrepareRx(p_usbd_instance, (uint8_t *)hcdc->data, hcdc->CmdLength);
        }
      }
      else
      {
        ((USBD_CDC_ItfTypeDef *)p_usbd_instance->pUserData)->Control(req->bRequest,
                                                                         (uint8_t *)req, 0U);
      }
      break;

    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        case USB_REQ_GET_STATUS:
          if (p_usbd_instance->dev_state == USBD_STATE_CONFIGURED)
          {
            USBD_CtlSendData(p_usbd_instance, (uint8_t *)&status_info, 2U);
          }
          else
          {
            USBD_CtlError(p_usbd_instance, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_GET_INTERFACE:
          if (p_usbd_instance->dev_state == USBD_STATE_CONFIGURED)
          {
            USBD_CtlSendData(p_usbd_instance, &ifalt, 1U);
          }
          else
          {
            USBD_CtlError(p_usbd_instance, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_SET_INTERFACE:
          if (p_usbd_instance->dev_state != USBD_STATE_CONFIGURED)
          {
            USBD_CtlError(p_usbd_instance, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_CLEAR_FEATURE:
          break;

        default:
          USBD_CtlError(p_usbd_instance, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    default:
      USBD_CtlError(p_usbd_instance, req);
      ret = USBD_FAIL;
      break;
  }

  return (uint8_t)ret;
}


/**
  * @brief  USBD_CDC_DataIn
  *         Data sent on non-control IN endpoint
  * @param  p_usbd_instance: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
uint8_t USBD_CDC_DataIn(USBD_Handle_t *p_usbd_instance, uint8_t epnum)
{
  USBD_CDC_Buffer_t *hcdc;
  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef *)p_usbd_instance->pData;

  if (p_usbd_instance->pClassDataCmsit == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  hcdc = (USBD_CDC_Buffer_t *)p_usbd_instance->pClassDataCmsit;

  if ((p_usbd_instance->ep_in[epnum & 0xFU].total_length > 0U) &&
      ((p_usbd_instance->ep_in[epnum & 0xFU].total_length % hpcd->IN_ep[epnum & 0xFU].maxpacket) == 0U))
  {
    /* Update the packet total length */
    p_usbd_instance->ep_in[epnum & 0xFU].total_length = 0U;

    /* Send ZLP */
    USBD_LL_Transmit(p_usbd_instance, epnum, NULL, 0U);
  }
  else
  {
    hcdc->TxState = 0U;

    if (((USBD_CDC_ItfTypeDef *)p_usbd_instance->pUserData)->TransmitCplt != NULL)
    {
      ((USBD_CDC_ItfTypeDef *)p_usbd_instance->pUserData)->TransmitCplt(hcdc->TxBuffer, &hcdc->TxLength, epnum);
    }
  }

  return (uint8_t)USBD_OK;
}



/**
  * @brief  USBD_CDC_DataOut
  *         Data received on non-control Out endpoint
  * @param  p_usbd_instance: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
uint8_t USBD_CDC_DataOut(USBD_Handle_t *p_usbd_instance, uint8_t epnum)
{
  USBD_CDC_Buffer_t *hcdc = (USBD_CDC_Buffer_t *)p_usbd_instance->pClassDataCmsit;

  if (p_usbd_instance->pClassDataCmsit == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  /* Get the received data length */
  hcdc->RxLength = USBD_LL_GetRxDataSize(p_usbd_instance, epnum);

  /* USB data will be immediately processed, this allow next USB traffic being
  NAKed till the end of the application Xfer */

  ((USBD_CDC_ItfTypeDef *)p_usbd_instance->pUserData)->Receive(hcdc->RxBuffer, &hcdc->RxLength);

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_EP0_RxReady
  *         Handle EP0 Rx Ready event
  * @param  p_usbd_instance: device instance
  * @retval status
  */
uint8_t USBD_CDC_EP0_RxReady(USBD_Handle_t *p_usbd_instance)
{
  USBD_CDC_Buffer_t *hcdc = (USBD_CDC_Buffer_t *)p_usbd_instance->pClassDataCmsit;

  if (hcdc == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if ((p_usbd_instance->pUserData != NULL) && (hcdc->CmdOpCode != 0xFFU))
  {
    ((USBD_CDC_ItfTypeDef *)p_usbd_instance->pUserData)->Control(hcdc->CmdOpCode,
                                                                     (uint8_t *)hcdc->data,
                                                                     (uint16_t)hcdc->CmdLength);
    hcdc->CmdOpCode = 0xFFU;
  }

  return (uint8_t)USBD_OK;
}
/**
  * @brief  Get full speed configuration descriptor.
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
uint8_t *USBD_CDC_GetFSCfgDesc(uint16_t *length)
{
  USBD_EpDescTypeDef *pEpCmdDesc = USBD_GetEpDesc(USBD_CDC_CfgDesc, CDC_CMD_EP);
  USBD_EpDescTypeDef *pEpOutDesc = USBD_GetEpDesc(USBD_CDC_CfgDesc, CDC_OUT_EP);
  USBD_EpDescTypeDef *pEpInDesc = USBD_GetEpDesc(USBD_CDC_CfgDesc, CDC_IN_EP);

  if (pEpCmdDesc != NULL)
  {
    pEpCmdDesc->bInterval = CDC_FS_BINTERVAL;
  }

  if (pEpOutDesc != NULL)
  {
    pEpOutDesc->wMaxPacketSize = CDC_DATA_FS_MAX_PACKET_SIZE;
  }

  if (pEpInDesc != NULL)
  {
    pEpInDesc->wMaxPacketSize = CDC_DATA_FS_MAX_PACKET_SIZE;
  }

  *length = (uint16_t)sizeof(USBD_CDC_CfgDesc);
  return USBD_CDC_CfgDesc;
}

/**
  * @brief  Return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
uint8_t *USBD_CDC_GetHSCfgDesc(uint16_t *length)
{
  USBD_EpDescTypeDef *pEpCmdDesc = USBD_GetEpDesc(USBD_CDC_CfgDesc, CDC_CMD_EP);
  USBD_EpDescTypeDef *pEpOutDesc = USBD_GetEpDesc(USBD_CDC_CfgDesc, CDC_OUT_EP);
  USBD_EpDescTypeDef *pEpInDesc = USBD_GetEpDesc(USBD_CDC_CfgDesc, CDC_IN_EP);

  if (pEpCmdDesc != NULL)
  {
    pEpCmdDesc->bInterval = CDC_HS_BINTERVAL;
  }

  if (pEpOutDesc != NULL)
  {
    pEpOutDesc->wMaxPacketSize = CDC_DATA_HS_MAX_PACKET_SIZE;
  }

  if (pEpInDesc != NULL)
  {
    pEpInDesc->wMaxPacketSize = CDC_DATA_HS_MAX_PACKET_SIZE;
  }

  *length = (uint16_t)sizeof(USBD_CDC_CfgDesc);
  return USBD_CDC_CfgDesc;
}

/**
  * @brief  USBD_CDC_GetOtherSpeedCfgDesc
  *         Return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
uint8_t *USBD_CDC_GetOtherSpeedCfgDesc(uint16_t *length)
{
  USBD_EpDescTypeDef *pEpCmdDesc = USBD_GetEpDesc(USBD_CDC_CfgDesc, CDC_CMD_EP);
  USBD_EpDescTypeDef *pEpOutDesc = USBD_GetEpDesc(USBD_CDC_CfgDesc, CDC_OUT_EP);
  USBD_EpDescTypeDef *pEpInDesc = USBD_GetEpDesc(USBD_CDC_CfgDesc, CDC_IN_EP);

  if (pEpCmdDesc != NULL)
  {
    pEpCmdDesc->bInterval = CDC_FS_BINTERVAL;
  }

  if (pEpOutDesc != NULL)
  {
    pEpOutDesc->wMaxPacketSize = CDC_DATA_FS_MAX_PACKET_SIZE;
  }

  if (pEpInDesc != NULL)
  {
    pEpInDesc->wMaxPacketSize = CDC_DATA_FS_MAX_PACKET_SIZE;
  }

  *length = (uint16_t)sizeof(USBD_CDC_CfgDesc);
  return USBD_CDC_CfgDesc;
}

/**
  * @brief  Get Device Qualifier descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
uint8_t *USBD_CDC_GetDeviceQualifierDescriptor(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_CDC_DeviceQualifierDesc);

  return USBD_CDC_DeviceQualifierDesc;
}

/**
  * @brief  USBD_CDC_RegisterInterface
  * @param  p_usbd_instance: device instance
  * @param  fops: CD  Interface callback
  * @retval status
  */
uint8_t USBD_CDC_RegisterInterface(USBD_Handle_t *p_usbd_instance,
                                   USBD_CDC_ItfTypeDef *fops)
{
  if (fops == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  p_usbd_instance->pUserData = fops;

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_SetTxBuffer
  * @param  p_usbd_instance: device instance
  * @param  pbuff: Tx Buffer
  * @param  length: Tx Buffer length
  * @retval status
  */
uint8_t USBD_CDC_SetTxBuffer(USBD_Handle_t *p_usbd_instance,
                             uint8_t *pbuff, uint32_t length)
{
  USBD_CDC_Buffer_t *hcdc = (USBD_CDC_Buffer_t *)p_usbd_instance->pClassDataCmsit;

  if (hcdc == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  hcdc->TxBuffer = pbuff;
  hcdc->TxLength = length;

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_SetRxBuffer
  * @param  p_usbd_instance: device instance
  * @param  pbuff: Rx Buffer
  * @retval status
  */
uint8_t USBD_CDC_SetRxBuffer(USBD_Handle_t *p_usbd_instance, uint8_t *pbuff)
{
  USBD_CDC_Buffer_t *hcdc = (USBD_CDC_Buffer_t *)p_usbd_instance->pClassDataCmsit;

  if (hcdc == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  hcdc->RxBuffer = pbuff;

  return (uint8_t)USBD_OK;
}


/**
  * @brief  USBD_CDC_ReceivePacket
  *         prepare OUT Endpoint for reception
  * @param  p_usbd_instance: device instance
  * @retval status
  */
uint8_t USBD_CDC_ReceivePacket(USBD_Handle_t *p_usbd_instance)
{
  USBD_CDC_Buffer_t *hcdc = (USBD_CDC_Buffer_t *)p_usbd_instance->pClassDataCmsit;

  if (p_usbd_instance->pClassDataCmsit == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (p_usbd_instance->dev_speed == USBD_SPEED_HIGH)
  {
    /* Prepare Out endpoint to receive next packet */
    USBD_LL_PrepareReceive(p_usbd_instance, CDC_OUT_EP, hcdc->RxBuffer,
                                 CDC_DATA_HS_OUT_PACKET_SIZE);
  }
  else
  {
    /* Prepare Out endpoint to receive next packet */
    USBD_LL_PrepareReceive(p_usbd_instance, CDC_OUT_EP, hcdc->RxBuffer,
                                 CDC_DATA_FS_OUT_PACKET_SIZE);
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  Initializes the device stack and load the class driver
  * @param  p_usbd_instance: device instance
  * @param  pdesc: Descriptor structure address
  * @param  id: Low level core index
  * @retval None
  */
USBD_StatusTypeDef USBD_Init(USBD_Handle_t *p_usbd_instance,
                             USBD_DescriptorsTypeDef *pdesc, uint8_t id)
{
  USBD_StatusTypeDef ret;
  /* Check whether the USB Host handle is valid */
  if (p_usbd_instance == NULL)
  {
    return USBD_FAIL;
  }
  /* Unlink previous class*/
  p_usbd_instance->pClass = NULL;
  p_usbd_instance->pUserData = NULL;
  p_usbd_instance->pConfDesc = NULL;

  /* Assign USBD Descriptors */
  if (pdesc != NULL)
  {
    p_usbd_instance->pDesc = pdesc;
  }
  /* Set Device initial State */
  p_usbd_instance->dev_state = USBD_STATE_DEFAULT;  // DEFAULT - ADDRESSED - CONFIGURED - SUSPEND
  p_usbd_instance->id = id; // DEVICE_FS (0)
  /* Initialize low level driver */
  ret = USBD_LL_Init(p_usbd_instance); // configure the TX or the RX FIFO of the new defined endport

  return ret;
}

/**
  * @brief  USBD_DeInit
  *         Re-Initialize the device library
  * @param  p_usbd_instance: device instance
  * @retval status: status
  */
USBD_StatusTypeDef USBD_DeInit(USBD_Handle_t *p_usbd_instance)
{
  USBD_StatusTypeDef ret;

  /* Disconnect the USB Device */
  USBD_LL_Stop(p_usbd_instance);

  /* Set Default State */
  p_usbd_instance->dev_state = USBD_STATE_DEFAULT;

  /* Free Class Resources */
  if (p_usbd_instance->pClass != NULL)
  {
    p_usbd_instance->pClass->DeInit(p_usbd_instance, (uint8_t)p_usbd_instance->dev_config);
  }

  p_usbd_instance->pUserData = NULL;

  /* Free Device descriptors resources */
  p_usbd_instance->pDesc = NULL;
  p_usbd_instance->pConfDesc = NULL;

  /* DeInitialize low level driver */
  ret = USBD_LL_DeInit(p_usbd_instance);

  return ret;
}

/**
  * @brief  USBD_RegisterClass
  *         Link class driver to Device Core.
  * @param  p_usbd_instanceice : Device Handle
  * @param  pclass: Class handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_RegisterClass(USBD_Handle_t *p_usbd_instance, USBD_ClassTypeDef *pclass)
{
  uint16_t len = 0U;

  if (pclass == NULL)
  {
    return USBD_FAIL;
  }
  /* link the class to the USB Device handle */
  p_usbd_instance->pClass = pclass;

  /* Get Device Configuration Descriptor */
  if (p_usbd_instance->pClass->GetFSConfigDescriptor != NULL)
  {
    p_usbd_instance->pConfDesc = (void *)p_usbd_instance->pClass->GetFSConfigDescriptor(&len);
  }
  /* Increment the NumClasses */
  p_usbd_instance->NumClasses ++;
  return USBD_OK;
}


/**
  * @brief  USBD_Start
  *         Start the USB Device Core.
  * @param  p_usbd_instance: Device Handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_Start(USBD_Handle_t *p_usbd_instance)
{
  /* Start the low level driver  */
  // return USBD_LL_Start(p_usbd_instance);
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  /* Enable USB Transciver */
  hal_status = HAL_PCD_Start(p_usbd_instance->pData);

  // usb_status =  USBD_Get_USB_Status(hal_status); /* defined in usbd_conf.c */
  switch (hal_status)
  {
    case HAL_OK :
      usb_status = USBD_OK;
    break;
    case HAL_ERROR :
      usb_status = USBD_FAIL;
    break;
    case HAL_BUSY :
      usb_status = USBD_BUSY;
    break;
    case HAL_TIMEOUT :
      usb_status = USBD_FAIL;
    break;
    default :
      usb_status = USBD_FAIL;
    break;
  }

  return usb_status;
}


/**
  * @brief  USBD_Stop
  *         Stop the USB Device Core.
  * @param  p_usbd_instance: Device Handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_Stop(USBD_Handle_t *p_usbd_instance)
{
  /* Disconnect USB Device */
  USBD_LL_Stop(p_usbd_instance);

  /* Free Class Resources */
  if (p_usbd_instance->pClass != NULL)
  {
    (void)p_usbd_instance->pClass->DeInit(p_usbd_instance, (uint8_t)p_usbd_instance->dev_config);
  }

  return USBD_OK;
}

/**
  * @brief  USBD_SetClassConfig
  *        Configure device and start the interface
  * @param  p_usbd_instance: device instance
  * @param  cfgidx: configuration index
  * @retval None.
  */

void USBD_SetClassConfig(USBD_Handle_t *p_usbd_instance, uint8_t cfgidx)
{
  if (p_usbd_instance->pClass != NULL)
  {
    /* Set configuration and Start the Class */
    p_usbd_instance->pClass->Init(p_usbd_instance, cfgidx);
  }
}

/**
  * @brief  USBD_ClrClassConfig
  *         Clear current configuration
  * @param  p_usbd_instance: device instance
  * @param  cfgidx: configuration index
  * @retval status: USBD_StatusTypeDef
  */
USBD_StatusTypeDef USBD_ClrClassConfig(USBD_Handle_t *p_usbd_instance, uint8_t cfgidx)
{
  USBD_StatusTypeDef ret = USBD_OK;

  /* Clear configuration  and De-initialize the Class process */
  if (p_usbd_instance->pClass->DeInit(p_usbd_instance, cfgidx) != 0U)
  {
    ret = USBD_FAIL;
  }

  return ret;
}


/**
  * @brief  Handle the setup stage
  * @param  p_usbd_instance: device instance
  * @retval None.
  */
void USBD_LL_SetupStage(USBD_Handle_t *p_usbd_instance, uint8_t *psetup)
{

  USBD_ParseSetupRequest(&p_usbd_instance->request, psetup);

  p_usbd_instance->ep0_state = USBD_EP0_SETUP;

  p_usbd_instance->ep0_data_len = p_usbd_instance->request.wLength;

  switch (p_usbd_instance->request.bmRequest & 0x1FU)
  {
    case USB_REQ_RECIPIENT_DEVICE:
      USBD_StdDevReq(p_usbd_instance, &p_usbd_instance->request);
      break;

    case USB_REQ_RECIPIENT_INTERFACE:
      USBD_StdItfReq(p_usbd_instance, &p_usbd_instance->request);
      break;

    case USB_REQ_RECIPIENT_ENDPOINT:
      USBD_StdEPReq(p_usbd_instance, &p_usbd_instance->request);
      break;

    default:
      USBD_LL_StallEP(p_usbd_instance, (p_usbd_instance->request.bmRequest & 0x80U));
      break;
  }
}

/**
  * @brief  USBD_LL_DataOutStage
  *         Handle data OUT stage
  * @param  p_usbd_instance: device instance
  * @param  epnum: endpoint index
  * @param  pdata: data pointer
  * @retval status
  */
USBD_StatusTypeDef USBD_LL_DataOutStage(USBD_Handle_t *p_usbd_instance,
                                        uint8_t epnum, uint8_t *pdata)
{
  USBD_EndpointTypeDef *pep;
  USBD_StatusTypeDef ret = USBD_OK;
  uint8_t idx;

  if (epnum == 0U)
  {
    pep = &p_usbd_instance->ep_out[0];

    if (p_usbd_instance->ep0_state == USBD_EP0_DATA_OUT)
    {
      if (pep->rem_length > pep->maxpacket)
      {
        pep->rem_length -= pep->maxpacket;

        (void)USBD_CtlContinueRx(p_usbd_instance, pdata, MIN(pep->rem_length, pep->maxpacket));
      }
      else
      {
        /* Find the class ID relative to the current request */
        switch (p_usbd_instance->request.bmRequest & 0x1FU)
        {
          case USB_REQ_RECIPIENT_DEVICE:
            /* Device requests must be managed by the first instantiated class
               (or duplicated by all classes for simplicity) */
            idx = 0U;
            break;

          case USB_REQ_RECIPIENT_INTERFACE:
            idx = 0x00U;
            break;

          case USB_REQ_RECIPIENT_ENDPOINT:
            idx = USBD_CoreFindEP(p_usbd_instance, LOBYTE(p_usbd_instance->request.wIndex));
            break;

          default:
            /* Back to the first class in case of doubt */
            idx = 0U;
            break;
        }

        /* Setup the class ID and route the request to the relative class function */
        if (p_usbd_instance->dev_state == USBD_STATE_CONFIGURED)
        {
          if (p_usbd_instance->pClass->EP0_RxReady != NULL)
          {
            // p_usbd_instance->classId = idx;
            p_usbd_instance->pClass->EP0_RxReady(p_usbd_instance);
          }
        }

        USBD_CtlSendStatus(p_usbd_instance);
      }
    }
    else
    {
    }
  }
  else
  {
    /* Get the class index relative to this interface */
    idx = USBD_CoreFindEP(p_usbd_instance, (epnum & 0x7FU));

    if ((uint16_t)idx != 0xFFU)
    {
      /* Call the class data out function to manage the request */
      if (p_usbd_instance->dev_state == USBD_STATE_CONFIGURED)
      {
        if (p_usbd_instance->pClass->DataOut != NULL)
        {
          // p_usbd_instance->classId = idx;
          ret = (USBD_StatusTypeDef)p_usbd_instance->pClass->DataOut(p_usbd_instance, epnum);
        }
      }
      if (ret != USBD_OK)
      {
        return ret;
      }
    }
  }

  return USBD_OK;
}

/**
  * @brief  USBD_LL_DataInStage
  *         Handle data in stage
  * @param  p_usbd_instance: device instance
  * @param  epnum: endpoint index
  * @retval void.
  */
void USBD_LL_DataInStage(USBD_Handle_t *p_usbd_instance,
                                       uint8_t epnum, uint8_t *pdata)
{
  USBD_EndpointTypeDef *pep;
  uint8_t idx;

  if (epnum == 0U)
  {
    pep = &p_usbd_instance->ep_in[0];

    if (p_usbd_instance->ep0_state == USBD_EP0_DATA_IN)
    {
      if (pep->rem_length > pep->maxpacket)
      {
        pep->rem_length -= pep->maxpacket;

        USBD_CtlContinueSendData(p_usbd_instance, pdata, pep->rem_length);

        /* Prepare endpoint for premature end of transfer */
        USBD_LL_PrepareReceive(p_usbd_instance, 0U, NULL, 0U);
      }
      else
      {
        /* last packet is MPS multiple, so send ZLP packet */
        if ((pep->maxpacket == pep->rem_length) &&
            (pep->total_length >= pep->maxpacket) &&
            (pep->total_length < p_usbd_instance->ep0_data_len))
        {
          USBD_CtlContinueSendData(p_usbd_instance, NULL, 0U);
          p_usbd_instance->ep0_data_len = 0U;

          /* Prepare endpoint for premature end of transfer */
          USBD_LL_PrepareReceive(p_usbd_instance, 0U, NULL, 0U);
        }
        else
        {
          if (p_usbd_instance->dev_state == USBD_STATE_CONFIGURED)
          {
            if (p_usbd_instance->pClass->EP0_TxSent != NULL)
            {
              // p_usbd_instance->classId = 0U;
              p_usbd_instance->pClass->EP0_TxSent(p_usbd_instance);
            }
          }
          (void)USBD_LL_StallEP(p_usbd_instance, 0x80U);
          (void)USBD_CtlReceiveStatus(p_usbd_instance);
        }
      }
    }
  }
  else
  {
    /* Get the class index relative to this interface */
    idx = USBD_CoreFindEP(p_usbd_instance, ((uint8_t)epnum | 0x80U));

    if ((uint16_t)idx != 0xFFU)
    {
      /* Call the class data out function to manage the request */
      if (p_usbd_instance->dev_state == USBD_STATE_CONFIGURED)
      {
        if (p_usbd_instance->pClass->DataIn != NULL)
        {
          // p_usbd_instance->classId = idx;
          p_usbd_instance->pClass->DataIn(p_usbd_instance, epnum);
        }
      }
    }
  }
}

/**
  * @brief  USBD_LL_Reset
  *         Handle Reset event
  * @param  p_usbd_instance: device instance
  * @retval status
  */

USBD_StatusTypeDef USBD_LL_Reset(USBD_Handle_t *p_usbd_instance)
{
  USBD_StatusTypeDef ret = USBD_OK;

  /* Upon Reset call user call back */
  p_usbd_instance->dev_state = USBD_STATE_DEFAULT;
  p_usbd_instance->ep0_state = USBD_EP0_IDLE;
  p_usbd_instance->dev_config = 0U;
  p_usbd_instance->dev_remote_wakeup = 0U;

  if (p_usbd_instance->pClass != NULL)
  {
    if (p_usbd_instance->pClass->DeInit != NULL)
    {
      if (p_usbd_instance->pClass->DeInit(p_usbd_instance, (uint8_t)p_usbd_instance->dev_config) != USBD_OK)
      {
        ret = USBD_FAIL;
      }
    }
  }

  /* Open EP0 OUT */
  USBD_LL_OpenEP(p_usbd_instance, 0x00U, USBD_EP_TYPE_CTRL, USB_MAX_EP0_SIZE);
  p_usbd_instance->ep_out[0x00U & 0xFU].is_used = 1U;

  p_usbd_instance->ep_out[0].maxpacket = USB_MAX_EP0_SIZE;

  /* Open EP0 IN */
  USBD_LL_OpenEP(p_usbd_instance, 0x80U, USBD_EP_TYPE_CTRL, USB_MAX_EP0_SIZE);
  p_usbd_instance->ep_in[0x80U & 0xFU].is_used = 1U;

  p_usbd_instance->ep_in[0].maxpacket = USB_MAX_EP0_SIZE;

  return ret;
}

/**
  * @brief  USBD_LL_SetSpeed
  *         Handle Reset event
  * @param  p_usbd_instance: device instance
  * @retval None.
  */
void USBD_LL_SetSpeed(USBD_Handle_t *p_usbd_instance,
                                    USBD_SpeedTypeDef speed)
{
  p_usbd_instance->dev_speed = speed;
}

/**
  * @brief  USBD_LL_Suspend
  *         Handle Suspend event
  * @param  p_usbd_instance: device instance
  * @retval status
  */

USBD_StatusTypeDef USBD_LL_Suspend(USBD_Handle_t *p_usbd_instance)
{
  p_usbd_instance->dev_old_state = p_usbd_instance->dev_state;
  p_usbd_instance->dev_state = USBD_STATE_SUSPENDED;
  return USBD_OK;
}

/**
  * @brief  USBD_LL_Resume
  *         Handle Resume event
  * @param  p_usbd_instance: device instance
  * @retval status
  */
USBD_StatusTypeDef USBD_LL_Resume(USBD_Handle_t *p_usbd_instance)
{
  if (p_usbd_instance->dev_state == USBD_STATE_SUSPENDED)
  {
    p_usbd_instance->dev_state = p_usbd_instance->dev_old_state;
  }

  return USBD_OK;
}

/**
  * @brief  USBD_LL_SOF
  *         Handle SOF event
  * @param  p_usbd_instance: device instance
  * @retval status
  */

USBD_StatusTypeDef USBD_LL_SOF(USBD_Handle_t *p_usbd_instance)
{
  /* The SOF event can be distributed for all classes that support it */
  if (p_usbd_instance->dev_state == USBD_STATE_CONFIGURED)
  {
    if (p_usbd_instance->pClass != NULL)
    {
      if (p_usbd_instance->pClass->SOF != NULL)
      {
        (void)p_usbd_instance->pClass->SOF(p_usbd_instance);
      }
    }
  }

  return USBD_OK;
}

/**
  * @brief  USBD_LL_IsoINIncomplete
  *         Handle iso in incomplete event
  * @param  p_usbd_instance: device instance
  * @retval status
  */
USBD_StatusTypeDef USBD_LL_IsoINIncomplete(USBD_Handle_t *p_usbd_instance,
                                           uint8_t epnum)
{
  if (p_usbd_instance->pClass == NULL)
  {
    return USBD_FAIL;
  }

  if (p_usbd_instance->dev_state == USBD_STATE_CONFIGURED)
  {
    if (p_usbd_instance->pClass->IsoINIncomplete != NULL)
    {
      (void)p_usbd_instance->pClass->IsoINIncomplete(p_usbd_instance, epnum);
    }
  }

  return USBD_OK;
}

/**
  * @brief  Handle iso out incomplete event.
  * @param  p_usbd_instance: device instance
  * @retval None.
  */
void USBD_LL_IsoOUTIncomplete(USBD_Handle_t *p_usbd_instance,
                                            uint8_t epnum)
{
  if (p_usbd_instance->pClass == NULL)
  {
    return;
  }

  if (p_usbd_instance->dev_state == USBD_STATE_CONFIGURED)
  {
    if (p_usbd_instance->pClass->IsoOUTIncomplete != NULL)
    {
      (void)p_usbd_instance->pClass->IsoOUTIncomplete(p_usbd_instance, epnum);
    }
  }
}

/**
  * @brief  USBD_LL_DevConnected
  *         Handle device connection event
  * @param  p_usbd_instance: device instance
  * @retval status
  */
USBD_StatusTypeDef USBD_LL_DevConnected(USBD_Handle_t *p_usbd_instance)
{
  /* Prevent unused argument compilation warning */
  UNUSED(p_usbd_instance);
  return USBD_OK;
}

/**
  * @brief  USBD_LL_DevDisconnected
  *         Handle device disconnection event
  * @param  p_usbd_instance: device instance
  * @retval status
  */
USBD_StatusTypeDef USBD_LL_DevDisconnected(USBD_Handle_t *p_usbd_instance)
{
  USBD_StatusTypeDef   ret = USBD_OK;

  /* Free Class Resources */
  p_usbd_instance->dev_state = USBD_STATE_DEFAULT;

  if (p_usbd_instance->pClass != NULL)
  {
    if (p_usbd_instance->pClass->DeInit(p_usbd_instance, (uint8_t)p_usbd_instance->dev_config) != 0U)
    {
      ret = USBD_FAIL;
    }
  }

  return ret;
}

/**
  * @brief  Returns the class index relative to the selected endpoint
  * @param  p_usbd_instance: device instance
  * @param  index : selected endpoint number
  * @retval index of the class using the selected endpoint number. 0xFF if no class found.
  */
uint8_t USBD_CoreFindEP(USBD_Handle_t *p_usbd_instance, uint8_t index)
{
  UNUSED(p_usbd_instance);
  UNUSED(index);
  return 0x00U;
}

/**
  * @brief  Returns the Endpoint descriptor
  * @param  p_usbd_instance: device instance
  * @param  pConfDesc:  pointer to Bos descriptor
  * @param  EpAddr:  endpoint address
  * @retval pointer to video endpoint descriptor
  */
void *USBD_GetEpDesc(uint8_t *pConfDesc, uint8_t EpAddr)
{
  USBD_DescHeaderTypeDef *pdesc = (USBD_DescHeaderTypeDef *)(void *)pConfDesc;
  USBD_ConfigDescTypeDef *desc = (USBD_ConfigDescTypeDef *)(void *)pConfDesc;
  USBD_EpDescTypeDef *pEpDesc = NULL;
  uint16_t ptr;

  if (desc->wTotalLength > desc->bLength)
  {
    ptr = desc->bLength;
    while (ptr < desc->wTotalLength)
    {
      pdesc = USBD_GetNextDesc((uint8_t *)pdesc, &ptr);
      if (pdesc->bDescriptorType == USB_DESC_TYPE_ENDPOINT)
      {
        pEpDesc = (USBD_EpDescTypeDef *)(void *)pdesc;

        if (pEpDesc->bEndpointAddress == EpAddr)
        {
          break;
        }
        else
        {
          pEpDesc = NULL;
        }
      }
    }
  }

  return (void *)pEpDesc;
}

/**
  * @brief  Returns the next descriptor header
  * @param  buf: Buffer where the descriptor is available
  * @param  ptr: data pointer inside the descriptor
  * @retval next header
  */
USBD_DescHeaderTypeDef *USBD_GetNextDesc(uint8_t *pbuf, uint16_t *ptr)
{
  USBD_DescHeaderTypeDef *pnext = (USBD_DescHeaderTypeDef *)(void *)pbuf;

  *ptr += pnext->bLength;
  pnext = (USBD_DescHeaderTypeDef *)(void *)(pbuf + pnext->bLength);
  return (pnext);
}

/**
  * @brief  USBD_StdDevReq
  *         Handle standard usb device requests
  * @param  p_usbd_instance: device instance
  * @param  req: usb request
  * @retval status
  */
USBD_StatusTypeDef USBD_StdDevReq(USBD_Handle_t *p_usbd_instance, USBD_SetupReqTypedef *req)
{
  USBD_StatusTypeDef ret = USBD_OK;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
    case USB_REQ_TYPE_VENDOR:
      ret = (USBD_StatusTypeDef)p_usbd_instance->pClass->Setup(p_usbd_instance, req);
      break;

    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        case USB_REQ_GET_DESCRIPTOR:
          USBD_GetDescriptor(p_usbd_instance, req);
          break;

        case USB_REQ_SET_ADDRESS:
          USBD_SetAddress(p_usbd_instance, req);
          break;

        case USB_REQ_SET_CONFIGURATION:
          ret = USBD_SetConfig(p_usbd_instance, req);
          break;

        case USB_REQ_GET_CONFIGURATION:
          USBD_GetConfig(p_usbd_instance, req);
          break;

        case USB_REQ_GET_STATUS:
          USBD_GetStatus(p_usbd_instance, req);
          break;

        case USB_REQ_SET_FEATURE:
          USBD_SetFeature(p_usbd_instance, req);
          break;

        case USB_REQ_CLEAR_FEATURE:
          USBD_ClrFeature(p_usbd_instance, req);
          break;

        default:
          USBD_CtlError(p_usbd_instance, req);
          break;
      }
      break;

    default:
      USBD_CtlError(p_usbd_instance, req);
      break;
  }

  return ret;
}

/**
  * @brief  USBD_StdItfReq
  *         Handle standard usb interface requests
  * @param  p_usbd_instance: device instance
  * @param  req: usb request
  * @retval status
  */
USBD_StatusTypeDef USBD_StdItfReq(USBD_Handle_t *p_usbd_instance, USBD_SetupReqTypedef *req)
{
  USBD_StatusTypeDef ret = USBD_OK;
  uint8_t idx;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
    case USB_REQ_TYPE_VENDOR:
    case USB_REQ_TYPE_STANDARD:
      switch (p_usbd_instance->dev_state)
      {
        case USBD_STATE_DEFAULT:
        case USBD_STATE_ADDRESSED:
        case USBD_STATE_CONFIGURED:

          if (LOBYTE(req->wIndex) <= USBD_MAX_NUM_INTERFACES)
          {
            /* Get the class index relative to this interface */
            idx = 0x00U;
            if ((uint8_t)idx != 0xFFU)
            {
              /* Call the class data out function to manage the request */
              if (p_usbd_instance->pClass->Setup != NULL)
              {
                // p_usbd_instance->classId = idx;
                ret = (USBD_StatusTypeDef)(p_usbd_instance->pClass->Setup(p_usbd_instance, req));
              }
              else
              {
                /* should never reach this condition */
                ret = USBD_FAIL;
              }
            }
            else
            {
              /* No relative interface found */
              ret = USBD_FAIL;
            }

            if ((req->wLength == 0U) && (ret == USBD_OK))
            {
              USBD_CtlSendStatus(p_usbd_instance);
            }
          }
          else
          {
            USBD_CtlError(p_usbd_instance, req);
          }
          break;

        default:
          USBD_CtlError(p_usbd_instance, req);
          break;
      }
      break;

    default:
      USBD_CtlError(p_usbd_instance, req);
      break;
  }

  return ret;
}

/**
  * @brief  Handle standard usb endpoint requests
  * @param  p_usbd_instance: device instance
  * @param  req: usb request
  * @retval status
  */
USBD_StatusTypeDef USBD_StdEPReq(USBD_Handle_t *p_usbd_instance, USBD_SetupReqTypedef *req)
{
  USBD_EndpointTypeDef *pep;
  uint8_t ep_addr;
  uint8_t idx;
  USBD_StatusTypeDef ret = USBD_OK;

  ep_addr = LOBYTE(req->wIndex);

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
    case USB_REQ_TYPE_VENDOR:
      /* Get the class index relative to this endpoint */
      idx = USBD_CoreFindEP(p_usbd_instance, ep_addr);
      if ((uint8_t)idx != 0xFFU)
      {
        // p_usbd_instance->classId = idx;
        /* Call the class data out function to manage the request */
        if (p_usbd_instance->pClass->Setup != NULL)
        {
          ret = (USBD_StatusTypeDef)p_usbd_instance->pClass->Setup(p_usbd_instance, req);
        }
      }
      break;

    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        case USB_REQ_SET_FEATURE:
          switch (p_usbd_instance->dev_state)
          {
            case USBD_STATE_ADDRESSED:
              if ((ep_addr != 0x00U) && (ep_addr != 0x80U))
              {
                (void)USBD_LL_StallEP(p_usbd_instance, ep_addr);
                (void)USBD_LL_StallEP(p_usbd_instance, 0x80U);
              }
              else
              {
                USBD_CtlError(p_usbd_instance, req);
              }
              break;

            case USBD_STATE_CONFIGURED:
              if (req->wValue == USB_FEATURE_EP_HALT)
              {
                if ((ep_addr != 0x00U) && (ep_addr != 0x80U) && (req->wLength == 0x00U))
                {
                  (void)USBD_LL_StallEP(p_usbd_instance, ep_addr);
                }
              }
              USBD_CtlSendStatus(p_usbd_instance);

              break;

            default:
              USBD_CtlError(p_usbd_instance, req);
              break;
          }
          break;

        case USB_REQ_CLEAR_FEATURE:

          switch (p_usbd_instance->dev_state)
          {
            case USBD_STATE_ADDRESSED:
              if ((ep_addr != 0x00U) && (ep_addr != 0x80U))
              {
                (void)USBD_LL_StallEP(p_usbd_instance, ep_addr);
                (void)USBD_LL_StallEP(p_usbd_instance, 0x80U);
              }
              else
              {
                USBD_CtlError(p_usbd_instance, req);
              }
              break;

            case USBD_STATE_CONFIGURED:
              if (req->wValue == USB_FEATURE_EP_HALT)
              {
                if ((ep_addr & 0x7FU) != 0x00U)
                {
                  USBD_LL_ClearStallEP(p_usbd_instance, ep_addr);
                }
                USBD_CtlSendStatus(p_usbd_instance);

                /* Get the class index relative to this interface */
                idx = USBD_CoreFindEP(p_usbd_instance, ep_addr);
                if ((uint8_t)idx != 0xFFU)
                {
                  // p_usbd_instance->classId = idx;
                  /* Call the class data out function to manage the request */
                  if (p_usbd_instance->pClass->Setup != NULL)
                  {
                    ret = (USBD_StatusTypeDef)(p_usbd_instance->pClass->Setup(p_usbd_instance, req));
                  }
                }
              }
              break;

            default:
              USBD_CtlError(p_usbd_instance, req);
              break;
          }
          break;

        case USB_REQ_GET_STATUS:
          switch (p_usbd_instance->dev_state)
          {
            case USBD_STATE_ADDRESSED:
              if ((ep_addr != 0x00U) && (ep_addr != 0x80U))
              {
                USBD_CtlError(p_usbd_instance, req);
                break;
              }
              pep = ((ep_addr & 0x80U) == 0x80U) ? &p_usbd_instance->ep_in[ep_addr & 0x7FU] : \
                    &p_usbd_instance->ep_out[ep_addr & 0x7FU];

              pep->status = 0x0000U;

              USBD_CtlSendData(p_usbd_instance, (uint8_t *)&pep->status, 2U);
              break;

            case USBD_STATE_CONFIGURED:
              if ((ep_addr & 0x80U) == 0x80U)
              {
                if (p_usbd_instance->ep_in[ep_addr & 0xFU].is_used == 0U)
                {
                  USBD_CtlError(p_usbd_instance, req);
                  break;
                }
              }
              else
              {
                if (p_usbd_instance->ep_out[ep_addr & 0xFU].is_used == 0U)
                {
                  USBD_CtlError(p_usbd_instance, req);
                  break;
                }
              }

              pep = ((ep_addr & 0x80U) == 0x80U) ? &p_usbd_instance->ep_in[ep_addr & 0x7FU] : \
                    &p_usbd_instance->ep_out[ep_addr & 0x7FU];

              if ((ep_addr == 0x00U) || (ep_addr == 0x80U))
              {
                pep->status = 0x0000U;
              }
              else if (USBD_LL_IsStallEP(p_usbd_instance, ep_addr) != 0U)
              {
                pep->status = 0x0001U;
              }
              else
              {
                pep->status = 0x0000U;
              }

              USBD_CtlSendData(p_usbd_instance, (uint8_t *)&pep->status, 2U);
              break;

            default:
              USBD_CtlError(p_usbd_instance, req);
              break;
          }
          break;

        default:
          USBD_CtlError(p_usbd_instance, req);
          break;
      }
      break;

    default:
      USBD_CtlError(p_usbd_instance, req);
      break;
  }

  return ret;
}


/**
  * @brief  Handle Get Descriptor requests
  * @param  p_usbd_instance: device instance
  * @param  req: usb request
  * @retval status
  */
void USBD_GetDescriptor(USBD_Handle_t *p_usbd_instance, USBD_SetupReqTypedef *req)
{
  uint16_t len = 0U;
  uint8_t *pbuf = NULL;
  uint8_t err = 0U;

  switch (req->wValue >> 8)
  {
    case USB_DESC_TYPE_DEVICE:
      pbuf = p_usbd_instance->pDesc->GetDeviceDescriptor(p_usbd_instance->dev_speed, &len);
      break;

    case USB_DESC_TYPE_CONFIGURATION:
      if (p_usbd_instance->dev_speed == USBD_SPEED_HIGH)
      {
        pbuf = (uint8_t *)p_usbd_instance->pClass->GetHSConfigDescriptor(&len);
        pbuf[1] = USB_DESC_TYPE_CONFIGURATION;
      }
      else
      {
        pbuf   = (uint8_t *)p_usbd_instance->pClass->GetFSConfigDescriptor(&len);
        pbuf[1] = USB_DESC_TYPE_CONFIGURATION;
      }
      break;

    case USB_DESC_TYPE_STRING:
      switch ((uint8_t)(req->wValue))
      {
        case USBD_IDX_LANGID_STR:
          if (p_usbd_instance->pDesc->GetLangIDStrDescriptor != NULL)
          {
            pbuf = p_usbd_instance->pDesc->GetLangIDStrDescriptor(p_usbd_instance->dev_speed, &len);
          }
          else
          {
            USBD_CtlError(p_usbd_instance, req);
            err++;
          }
          break;

        case USBD_IDX_MFC_STR:
          if (p_usbd_instance->pDesc->GetManufacturerStrDescriptor != NULL)
          {
            pbuf = p_usbd_instance->pDesc->GetManufacturerStrDescriptor(p_usbd_instance->dev_speed, &len);
          }
          else
          {
            USBD_CtlError(p_usbd_instance, req);
            err++;
          }
          break;

        case USBD_IDX_PRODUCT_STR:
          if (p_usbd_instance->pDesc->GetProductStrDescriptor != NULL)
          {
            pbuf = p_usbd_instance->pDesc->GetProductStrDescriptor(p_usbd_instance->dev_speed, &len);
          }
          else
          {
            USBD_CtlError(p_usbd_instance, req);
            err++;
          }
          break;

        case USBD_IDX_SERIAL_STR:
          if (p_usbd_instance->pDesc->GetSerialStrDescriptor != NULL)
          {
            pbuf = p_usbd_instance->pDesc->GetSerialStrDescriptor(p_usbd_instance->dev_speed, &len);
          }
          else
          {
            USBD_CtlError(p_usbd_instance, req);
            err++;
          }
          break;

        case USBD_IDX_CONFIG_STR:
          if (p_usbd_instance->pDesc->GetConfigurationStrDescriptor != NULL)
          {
            pbuf = p_usbd_instance->pDesc->GetConfigurationStrDescriptor(p_usbd_instance->dev_speed, &len);
          }
          else
          {
            USBD_CtlError(p_usbd_instance, req);
            err++;
          }
          break;

        case USBD_IDX_INTERFACE_STR:
          if (p_usbd_instance->pDesc->GetInterfaceStrDescriptor != NULL)
          {
            pbuf = p_usbd_instance->pDesc->GetInterfaceStrDescriptor(p_usbd_instance->dev_speed, &len);
          }
          else
          {
            USBD_CtlError(p_usbd_instance, req);
            err++;
          }
          break;

        default:

#if ((USBD_CLASS_USER_STRING_DESC == 0U) && (USBD_SUPPORT_USER_STRING_DESC == 0U))
          USBD_CtlError(p_usbd_instance, req);
          err++;
#endif /* (USBD_CLASS_USER_STRING_DESC == 0U) && (USBD_SUPPORT_USER_STRING_DESC == 0U) */
          break;
      }
      break;

    case USB_DESC_TYPE_DEVICE_QUALIFIER:
      if (p_usbd_instance->dev_speed == USBD_SPEED_HIGH)
      {
        pbuf = (uint8_t *)p_usbd_instance->pClass->GetDeviceQualifierDescriptor(&len);
      }
      else
      {
        USBD_CtlError(p_usbd_instance, req);
        err++;
      }
      break;

    case USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION:
      if (p_usbd_instance->dev_speed == USBD_SPEED_HIGH)
      {
        pbuf = (uint8_t *)p_usbd_instance->pClass->GetOtherSpeedConfigDescriptor(&len);
        pbuf[1] = USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION;
      }
      else
      {
        USBD_CtlError(p_usbd_instance, req);
        err++;
      }
      break;

    default:
      USBD_CtlError(p_usbd_instance, req);
      err++;
      break;
  }

  if (err != 0U)
  {
    return;
  }

  if (req->wLength != 0U)
  {
    if (len != 0U)
    {
      len = MIN(len, req->wLength);
      USBD_CtlSendData(p_usbd_instance, pbuf, len);
    }
    else
    {
      USBD_CtlError(p_usbd_instance, req);
    }
  }
  else
  {
    USBD_CtlSendStatus(p_usbd_instance);
  }
}


/**
  * @brief  Set device address
  * @param  p_usbd_instance: device instance
  * @param  req: usb request
  * @retval status
  */
void USBD_SetAddress(USBD_Handle_t *p_usbd_instance, USBD_SetupReqTypedef *req)
{
  uint8_t  dev_addr;

  if ((req->wIndex == 0U) && (req->wLength == 0U) && (req->wValue < 128U))
  {
    dev_addr = (uint8_t)(req->wValue) & 0x7FU;

    if (p_usbd_instance->dev_state == USBD_STATE_CONFIGURED)
    {
      USBD_CtlError(p_usbd_instance, req);
    }
    else
    {
      p_usbd_instance->dev_address = dev_addr;
      (void)USBD_LL_SetUSBAddress(p_usbd_instance, dev_addr);
      USBD_CtlSendStatus(p_usbd_instance);

      if (dev_addr != 0U)
      {
        p_usbd_instance->dev_state = USBD_STATE_ADDRESSED;
      }
      else
      {
        p_usbd_instance->dev_state = USBD_STATE_DEFAULT;
      }
    }
  }
  else
  {
    USBD_CtlError(p_usbd_instance, req);
  }
}

/**
  * @brief  Handle Set device configuration request
  * @param  p_usbd_instance: device instance
  * @param  req: usb request
  * @retval status
  */
USBD_StatusTypeDef USBD_SetConfig(USBD_Handle_t *p_usbd_instance, USBD_SetupReqTypedef *req)
{
  USBD_StatusTypeDef ret = USBD_OK;
  uint8_t cfgidx;

  cfgidx = (uint8_t)(req->wValue);

  if (cfgidx > USBD_MAX_NUM_CONFIGURATION)
  {
    USBD_CtlError(p_usbd_instance, req);
    return USBD_FAIL;
  }

  switch (p_usbd_instance->dev_state)
  {
    case USBD_STATE_ADDRESSED:
      if (cfgidx != 0U)
      {
        p_usbd_instance->dev_config = cfgidx;

        USBD_SetClassConfig(p_usbd_instance, cfgidx);
        USBD_CtlSendStatus(p_usbd_instance);
        p_usbd_instance->dev_state = USBD_STATE_CONFIGURED;
      }
      else
      {
        USBD_CtlSendStatus(p_usbd_instance);
      }
      break;

    case USBD_STATE_CONFIGURED:
      if (cfgidx == 0U)
      {
        p_usbd_instance->dev_state = USBD_STATE_ADDRESSED;
        p_usbd_instance->dev_config = cfgidx;
        (void)USBD_ClrClassConfig(p_usbd_instance, cfgidx);
        USBD_CtlSendStatus(p_usbd_instance);
      }
      else if (cfgidx != p_usbd_instance->dev_config)
      {
        /* Clear old configuration */
        (void)USBD_ClrClassConfig(p_usbd_instance, (uint8_t)p_usbd_instance->dev_config);

        /* set new configuration */
        p_usbd_instance->dev_config = cfgidx;

        USBD_SetClassConfig(p_usbd_instance, cfgidx);
        USBD_CtlSendStatus(p_usbd_instance);
      }
      else
      {
        USBD_CtlSendStatus(p_usbd_instance);
      }
      break;

    default:
      USBD_CtlError(p_usbd_instance, req);
      (void)USBD_ClrClassConfig(p_usbd_instance, cfgidx);
      ret = USBD_FAIL;
      break;
  }

  return ret;
}

/**
  * @brief  Handle Get device configuration request
  * @param  p_usbd_instance: device instance
  * @param  req: usb request
  * @retval status
  */
void USBD_GetConfig(USBD_Handle_t *p_usbd_instance, USBD_SetupReqTypedef *req)
{
  if (req->wLength != 1U)
  {
    USBD_CtlError(p_usbd_instance, req);
  }
  else
  {
    switch (p_usbd_instance->dev_state)
    {
      case USBD_STATE_DEFAULT:
      case USBD_STATE_ADDRESSED:
        p_usbd_instance->dev_default_config = 0U;
        USBD_CtlSendData(p_usbd_instance, (uint8_t *)&p_usbd_instance->dev_default_config, 1U);
        break;

      case USBD_STATE_CONFIGURED:
        USBD_CtlSendData(p_usbd_instance, (uint8_t *)&p_usbd_instance->dev_config, 1U);
        break;

      default:
        USBD_CtlError(p_usbd_instance, req);
        break;
    }
  }
}

/**
  * @brief  USBD_GetStatus
  *         Handle Get Status request
  * @param  p_usbd_instance: device instance
  * @param  req: usb request
  * @retval status
  */
void USBD_GetStatus(USBD_Handle_t *p_usbd_instance, USBD_SetupReqTypedef *req)
{
  switch (p_usbd_instance->dev_state)
  {
    case USBD_STATE_DEFAULT:
    case USBD_STATE_ADDRESSED:
    case USBD_STATE_CONFIGURED:
      if (req->wLength != 0x2U)
      {
        USBD_CtlError(p_usbd_instance, req);
        break;
      }

#if (USBD_SELF_POWERED == 1U)
      p_usbd_instance->dev_config_status = USB_CONFIG_SELF_POWERED;
#else
      p_usbd_instance->dev_config_status = 0U;
#endif /* USBD_SELF_POWERED */

      if (p_usbd_instance->dev_remote_wakeup != 0U)
      {
        p_usbd_instance->dev_config_status |= USB_CONFIG_REMOTE_WAKEUP;
      }

      USBD_CtlSendData(p_usbd_instance, (uint8_t *)&p_usbd_instance->dev_config_status, 2U);
      break;

    default:
      USBD_CtlError(p_usbd_instance, req);
      break;
  }
}


/**
  * @brief  USBD_SetFeature
  *         Handle Set device feature request
  * @param  p_usbd_instance: device instance
  * @param  req: usb request
  * @retval status
  */
void USBD_SetFeature(USBD_Handle_t *p_usbd_instance, USBD_SetupReqTypedef *req)
{
  if (req->wValue == USB_FEATURE_REMOTE_WAKEUP)
  {
    p_usbd_instance->dev_remote_wakeup = 1U;
    USBD_CtlSendStatus(p_usbd_instance);
  }
  else
  {
    USBD_CtlError(p_usbd_instance, req);
  }
}


/**
  * @brief  USBD_ClrFeature
  *         Handle clear device feature request
  * @param  p_usbd_instance: device instance
  * @param  req: usb request
  * @retval status
  */
void USBD_ClrFeature(USBD_Handle_t *p_usbd_instance, USBD_SetupReqTypedef *req)
{
  switch (p_usbd_instance->dev_state)
  {
    case USBD_STATE_DEFAULT:
    case USBD_STATE_ADDRESSED:
    case USBD_STATE_CONFIGURED:
      if (req->wValue == USB_FEATURE_REMOTE_WAKEUP)
      {
        p_usbd_instance->dev_remote_wakeup = 0U;
        USBD_CtlSendStatus(p_usbd_instance);
      }
      break;

    default:
      USBD_CtlError(p_usbd_instance, req);
      break;
  }
}


/**
  * @brief  USBD_ParseSetupRequest
  *         Copy buffer into setup structure
  * @param  p_usbd_instance: device instance
  * @param  req: usb request
  * @retval None
  */
void USBD_ParseSetupRequest(USBD_SetupReqTypedef *req, uint8_t *pdata)
{
  uint8_t *pbuff = pdata;

  req->bmRequest = *(uint8_t *)(pbuff);

  pbuff++;
  req->bRequest = *(uint8_t *)(pbuff);

  pbuff++;
  req->wValue = SWAPBYTE(pbuff);

  pbuff++;
  pbuff++;
  req->wIndex = SWAPBYTE(pbuff);

  pbuff++;
  pbuff++;
  req->wLength = SWAPBYTE(pbuff);
}


/**
  * @brief  Handle USB low level Error
  * @param  p_usbd_instance: device instance
  * @param  req: usb request
  * @retval None
  */
void USBD_CtlError(USBD_Handle_t *p_usbd_instance, USBD_SetupReqTypedef *req)
{
  UNUSED(req);

  (void)USBD_LL_StallEP(p_usbd_instance, 0x80U);
  (void)USBD_LL_StallEP(p_usbd_instance, 0U);
}


/**
  * @brief  USBD_GetString
  *         Convert Ascii string into unicode one
  * @param  desc : descriptor buffer
  * @param  unicode : Formatted string buffer (unicode)
  * @param  len : descriptor length
  * @retval None
  */
void USBD_GetString(uint8_t *desc, uint8_t *unicode, uint16_t *len)
{
  uint8_t idx = 0U;
  uint8_t *pdesc;

  if (desc == NULL)
  {
    return;
  }

  pdesc = desc;
  *len = ((uint16_t)USBD_GetLen(pdesc) * 2U) + 2U;

  unicode[idx] = *(uint8_t *)len;
  idx++;
  unicode[idx] = USB_DESC_TYPE_STRING;
  idx++;

  while (*pdesc != (uint8_t)'\0')
  {
    unicode[idx] = *pdesc;
    pdesc++;
    idx++;

    unicode[idx] = 0U;
    idx++;
  }
}


/**
  * @brief  Return the string length
   * @param  buf : pointer to the ascii string buffer
  * @retval string length
  */
uint8_t USBD_GetLen(uint8_t *buf)
{
  uint8_t  len = 0U;
  uint8_t *pbuff = buf;

  while (*pbuff != (uint8_t)'\0')
  {
    len++;
    pbuff++;
  }

  return len;
}

/**
  * @brief  USBD_CtlSendData
  *         send data on the ctl pipe
  * @param  p_usbd_instance: device instance
  * @param  buff: pointer to data buffer
  * @param  len: length of data to be sent
  * @retval void
  */
void USBD_CtlSendData(USBD_Handle_t *p_usbd_instance,
                                    uint8_t *pbuf, uint32_t len)
{
  /* Set EP0 State */
  p_usbd_instance->ep0_state = USBD_EP0_DATA_IN;
  p_usbd_instance->ep_in[0].total_length = len;
  p_usbd_instance->ep_in[0].rem_length = len;
  /* Start the transfer */
  USBD_LL_Transmit(p_usbd_instance, 0x00U, pbuf, len);
}

/**
  * @brief  USBD_CtlContinueSendData
  *         continue sending data on the ctl pipe
  * @param  p_usbd_instance: device instance
  * @param  buff: pointer to data buffer
  * @param  len: length of data to be sent
  * @retval None.
  */
void USBD_CtlContinueSendData(USBD_Handle_t *p_usbd_instance,
                                            uint8_t *pbuf, uint32_t len)
{
  /* Start the next transfer */
  USBD_LL_Transmit(p_usbd_instance, 0x00U, pbuf, len);
}

/**
  * @brief  USBD_CtlPrepareRx
  *         receive data on the ctl pipe
  * @param  p_usbd_instance: device instance
  * @param  buff: pointer to data buffer
  * @param  len: length of data to be received
  * @retval None.
  */
void USBD_CtlPrepareRx(USBD_Handle_t *p_usbd_instance,
                                     uint8_t *pbuf, uint32_t len)
{
  /* Set EP0 State */
  p_usbd_instance->ep0_state = USBD_EP0_DATA_OUT;
  p_usbd_instance->ep_out[0].total_length = len;
  p_usbd_instance->ep_out[0].rem_length = len;

  /* Start the transfer */
  USBD_LL_PrepareReceive(p_usbd_instance, 0U, pbuf, len);
}

/**
  * @brief  USBD_CtlContinueRx
  *         continue receive data on the ctl pipe
  * @param  p_usbd_instance: device instance
  * @param  buff: pointer to data buffer
  * @param  len: length of data to be received
  * @retval status
  */
USBD_StatusTypeDef USBD_CtlContinueRx(USBD_Handle_t *p_usbd_instance,
                                      uint8_t *pbuf, uint32_t len)
{
  USBD_LL_PrepareReceive(p_usbd_instance, 0U, pbuf, len);

  return USBD_OK;
}

/**
  * @brief  USBD_CtlSendStatus
  *         send zero lzngth packet on the ctl pipe
  * @param  p_usbd_instance: device instance
  * @retval status
  */
void USBD_CtlSendStatus(USBD_Handle_t *p_usbd_instance)
{
  /* Set EP0 State */
  p_usbd_instance->ep0_state = USBD_EP0_STATUS_IN;

  /* Start the transfer */
  USBD_LL_Transmit(p_usbd_instance, 0x00U, NULL, 0U);
}

/**
  * @brief  USBD_CtlReceiveStatus
  *         receive zero lzngth packet on the ctl pipe
  * @param  p_usbd_instance: device instance
  * @retval status
  */
USBD_StatusTypeDef USBD_CtlReceiveStatus(USBD_Handle_t *p_usbd_instance)
{
  /* Set EP0 State */
  p_usbd_instance->ep0_state = USBD_EP0_STATUS_OUT;

  /* Start the transfer */
  USBD_LL_PrepareReceive(p_usbd_instance, 0U, NULL, 0U);

  return USBD_OK;
}

/**
  * @brief  USBD_GetRxCount
  *         returns the received data length
  * @param  p_usbd_instance: device instance
  * @param  ep_addr: endpoint address
  * @retval Rx Data blength
  */
uint32_t USBD_GetRxCount(USBD_Handle_t *p_usbd_instance, uint8_t ep_addr)
{
  return USBD_LL_GetRxDataSize(p_usbd_instance, ep_addr);
}

/* usbd_ioreq.c END ---- ---- ---- ---- */



PCD_HandleTypeDef hpcd_USB_OTG_FS;
void Error_Handler(void);

void SystemClock_Config(void);
USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef hal_status);

/*******************************************************************************
                       LL Driver Callbacks (PCD -> USB Device Library)
*******************************************************************************/
/* MSP Init */

void HAL_PCD_MspInit(PCD_HandleTypeDef* pcdHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(pcdHandle->Instance==USB_OTG_FS)
  {
  /* USER CODE BEGIN USB_OTG_FS_MspInit 0 */

  /* USER CODE END USB_OTG_FS_MspInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USB_OTG_FS GPIO Configuration
    PA11     ------> USB_OTG_FS_DM
    PA12     ------> USB_OTG_FS_DP
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
  /* USER CODE BEGIN USB_OTG_FS_MspInit 1 */

  /* USER CODE END USB_OTG_FS_MspInit 1 */
  }
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef* pcdHandle)
{
  if(pcdHandle->Instance==USB_OTG_FS)
  {
  /* USER CODE BEGIN USB_OTG_FS_MspDeInit 0 */

  /* USER CODE END USB_OTG_FS_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();

    /**USB_OTG_FS GPIO Configuration
    PA11     ------> USB_OTG_FS_DM
    PA12     ------> USB_OTG_FS_DP
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
  }
}

/**
  * @brief  Setup stage callback
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SetupStage((USBD_Handle_t*)hpcd->pData, (uint8_t *)hpcd->Setup);
}

/**
  * @brief  Data Out stage callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint number
  * @retval None
  */
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_DataOutStage((USBD_Handle_t*)hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}

/**
  * @brief  Data In stage callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint number
  * @retval None
  */
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_DataInStage((USBD_Handle_t*)hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}

/**
  * @brief  SOF callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SOF((USBD_Handle_t*)hpcd->pData);
}

/**
  * @brief  Reset callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_SpeedTypeDef speed = USBD_SPEED_FULL;

  if ( hpcd->Init.speed == PCD_SPEED_HIGH)
  {
    speed = USBD_SPEED_HIGH;
  }
  else if ( hpcd->Init.speed == PCD_SPEED_FULL)
  {
    speed = USBD_SPEED_FULL;
  }
  else
  {
    Error_Handler();
  }
    /* Set Speed. */
  USBD_LL_SetSpeed((USBD_Handle_t*)hpcd->pData, speed);

  /* Reset Device. */
  USBD_LL_Reset((USBD_Handle_t*)hpcd->pData);
}

/**
  * @brief  Suspend callback.
  * When Low power mode is enabled the debug cannot be used (IAR, Keil doesn't support it)
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
  /* Inform USB library that core enters in suspend Mode. */
  USBD_LL_Suspend((USBD_Handle_t*)hpcd->pData);
  __HAL_PCD_GATE_PHYCLOCK(hpcd);
  /* Enter in STOP mode. */
  /* USER CODE BEGIN 2 */
  if (hpcd->Init.low_power_enable)
  {
    /* Set SLEEPDEEP bit and SleepOnExit of Cortex System Control Register. */
    SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
  }
}


/**
  * @brief  Connect callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevConnected((USBD_Handle_t*)hpcd->pData);
}

/**
  * @brief  Disconnect callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevDisconnected((USBD_Handle_t*)hpcd->pData);
}

/*******************************************************************************
                       LL Driver Interface (USB Device Library --> PCD)
*******************************************************************************/

/**
  * @brief  Initializes the low level portion of the device driver.
  * @param  p_usbd_instance: Device handle
  * @retval USBD status
  */
USBD_StatusTypeDef USBD_LL_Init(USBD_Handle_t *p_usbd_instance)
{
  /* Init USB Ip. */
  if (p_usbd_instance->id == DEVICE_FS) {
  /* Link the driver to the stack. */
  hpcd_USB_OTG_FS.pData = p_usbd_instance;
  p_usbd_instance->pData = &hpcd_USB_OTG_FS;

  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;

  /* Initialize the hardware. */
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler( );
  }

  /* in stm32f4xx_hal_pcd_ex.h */ 
  HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 0x80);    /* set RX FIFO size 0x80*/
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 0x40); /* set TX FIFO 0 with size 0x40 */ 
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1, 0x80); /* set TX FIFO 1 with size 0x80 */
  }
  return USBD_OK;
}

/**
  * @brief  De-Initializes the low level portion of the device driver.
  * @param  p_usbd_instance: Device handle
  * @retval USBD status
  */
USBD_StatusTypeDef USBD_LL_DeInit(USBD_Handle_t *p_usbd_instance)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_DeInit(p_usbd_instance->pData);

  usb_status =  USBD_Get_USB_Status(hal_status);

  return usb_status;
}

/**
  * @brief  Starts the low level portion of the device driver.
  * @param  p_usbd_instance: Device handle
  * @retval USBD status
  */
USBD_StatusTypeDef USBD_LL_Start(USBD_Handle_t *p_usbd_instance)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_Start(p_usbd_instance->pData);

  usb_status =  USBD_Get_USB_Status(hal_status);

  return usb_status;
}

/**
  * @brief  Stops the low level portion of the device driver.
  * @param  p_usbd_instance: Device handle
  * @retval None.
  */
void USBD_LL_Stop(USBD_Handle_t *p_usbd_instance)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  hal_status = HAL_PCD_Stop(p_usbd_instance->pData);
  USBD_Get_USB_Status(hal_status);
}

/**
  * @brief  Opens an endpoint of the low level driver.
  * @param  p_usbd_instance: Device handle
  * @param  ep_addr: Endpoint number
  * @param  ep_type: Endpoint type
  * @param  ep_mps: Endpoint max packet size
  * @retval None.
  */
void USBD_LL_OpenEP(USBD_Handle_t *p_usbd_instance, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps)
{
  HAL_PCD_EP_Open(p_usbd_instance->pData, ep_addr, ep_mps, ep_type);
}

/**
  * @brief  Closes an endpoint of the low level driver.
  * @param  p_usbd_instance: Device handle
  * @param  ep_addr: Endpoint number
  * @retval None.
  */
void USBD_LL_CloseEP(USBD_Handle_t *p_usbd_instance, uint8_t ep_addr)
{
  HAL_PCD_EP_Close(p_usbd_instance->pData, ep_addr);
}

/**
  * @brief  Flushes an endpoint of the Low Level Driver.
  * @param  p_usbd_instance: Device handle
  * @param  ep_addr: Endpoint number
  * @retval None.
  */
void USBD_LL_FlushEP(USBD_Handle_t *p_usbd_instance, uint8_t ep_addr)
{
  HAL_PCD_EP_Flush(p_usbd_instance->pData, ep_addr);
}

/**
  * @brief  Sets a Stall condition on an endpoint of the Low Level Driver.
  * @param  p_usbd_instance: Device handle
  * @param  ep_addr: Endpoint number
  * @retval None
  */
void USBD_LL_StallEP(USBD_Handle_t *p_usbd_instance, uint8_t ep_addr)
{
  HAL_PCD_EP_SetStall(p_usbd_instance->pData, ep_addr);
}

/**
  * @brief  Clears a Stall condition on an endpoint of the Low Level Driver.
  * @param  p_usbd_instance: Device handle
  * @param  ep_addr: Endpoint number
  * @retval None.
  */
void USBD_LL_ClearStallEP(USBD_Handle_t *p_usbd_instance, uint8_t ep_addr)
{
  HAL_PCD_EP_ClrStall(p_usbd_instance->pData, ep_addr);
}

/**
  * @brief  Returns Stall condition.
  * @param  p_usbd_instance: Device handle
  * @param  ep_addr: Endpoint number
  * @retval Stall (1: Yes, 0: No)
  */
uint8_t USBD_LL_IsStallEP(USBD_Handle_t *p_usbd_instance, uint8_t ep_addr)
{
  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef*) p_usbd_instance->pData;

  if((ep_addr & 0x80) == 0x80)
  {
    return hpcd->IN_ep[ep_addr & 0x7F].is_stall;
  }
  else
  {
    return hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
  }
}

/**
  * @brief  Assigns a USB address to the device.
  * @param  p_usbd_instance: Device handle
  * @param  dev_addr: Device address
  * @retval void
  */
void USBD_LL_SetUSBAddress(USBD_Handle_t *p_usbd_instance, uint8_t dev_addr)
{
  HAL_PCD_SetAddress(p_usbd_instance->pData, dev_addr);
}

/**
  * @brief  Transmits data over an endpoint.
  * @param  p_usbd_instance: Device handle
  * @param  ep_addr: Endpoint number
  * @param  pbuf: Pointer to data to be sent
  * @param  size: Data size
  * @retval None.
  */
void USBD_LL_Transmit(USBD_Handle_t *p_usbd_instance, uint8_t ep_addr, uint8_t *pbuf, uint32_t size)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  hal_status = HAL_PCD_EP_Transmit(p_usbd_instance->pData, ep_addr, pbuf, size);
  USBD_Get_USB_Status(hal_status);
}

/**
  * @brief  Prepares an endpoint for reception.
  * @param  p_usbd_instance: Device handle
  * @param  ep_addr: Endpoint number
  * @param  pbuf: Pointer to data to be received
  * @param  size: Data size
  * @retval void
  */
void USBD_LL_PrepareReceive(USBD_Handle_t *p_usbd_instance, uint8_t ep_addr, uint8_t *pbuf, uint32_t size)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  hal_status = HAL_PCD_EP_Receive(p_usbd_instance->pData, ep_addr, pbuf, size);
  USBD_Get_USB_Status(hal_status);
}

/**
  * @brief  Returns the last transferred packet size.
  * @param  p_usbd_instance: Device handle
  * @param  ep_addr: Endpoint number
  * @retval Received Data Size
  */
uint32_t USBD_LL_GetRxDataSize(USBD_Handle_t *p_usbd_instance, uint8_t ep_addr)
{
  return HAL_PCD_EP_GetRxCount((PCD_HandleTypeDef*) p_usbd_instance->pData, ep_addr);
}

/**
  * @brief  single allocation.
  * @param  size: Size of allocated memory
  * @retval None
  */
void *USBD_static_malloc(uint32_t size)
{
  static uint32_t mem[(sizeof(USBD_CDC_Buffer_t)/4)+1];/* On 32-bit boundary */
  return mem;
}

/**
  * @brief  Dummy memory free
  * @param  p: Pointer to allocated  memory address
  * @retval None
  */
void USBD_static_free(void *p)
{
}

/**
  * @brief  Delays routine for the USB Device Library.
  * @param  Delay: Delay in ms
  * @retval None
  */
void USBD_LL_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}

/**
  * @brief  Returns the USB status depending on the HAL status:
  * @param  hal_status: HAL status
  * @retval USB status
  */
USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef hal_status)
{
  USBD_StatusTypeDef usb_status = USBD_OK;

  switch (hal_status)
  {
    case HAL_OK :
      usb_status = USBD_OK;
    break;
    case HAL_ERROR :
      usb_status = USBD_FAIL;
    break;
    case HAL_BUSY :
      usb_status = USBD_BUSY;
    break;
    case HAL_TIMEOUT :
      usb_status = USBD_FAIL;
    break;
    default :
      usb_status = USBD_FAIL;
    break;
  }
  return usb_status;
}

USBD_DescriptorsTypeDef FS_Desc =
{
  USBD_FS_DeviceDescriptor
, USBD_FS_LangIDStrDescriptor
, USBD_FS_ManufacturerStrDescriptor
, USBD_FS_ProductStrDescriptor
, USBD_FS_SerialStrDescriptor
, USBD_FS_ConfigStrDescriptor
, USBD_FS_InterfaceStrDescriptor
};

/** USB standard device descriptor. */
__ALIGN_BEGIN uint8_t USBD_FS_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END =
{
  0x12,                       /*bLength */
  USB_DESC_TYPE_DEVICE,       /*bDescriptorType*/
  0x00,                       /*bcdUSB */
  0x02,
  0x02,                       /*bDeviceClass*/
  0x02,                       /*bDeviceSubClass*/
  0x00,                       /*bDeviceProtocol*/
  USB_MAX_EP0_SIZE,           /*bMaxPacketSize*/
  LOBYTE(USBD_VID),           /*idVendor*/
  HIBYTE(USBD_VID),           /*idVendor*/
  LOBYTE(USBD_PID_FS),        /*idProduct*/
  HIBYTE(USBD_PID_FS),        /*idProduct*/
  0x00,                       /*bcdDevice rel. 2.00*/
  0x02,
  USBD_IDX_MFC_STR,           /*Index of manufacturer  string*/
  USBD_IDX_PRODUCT_STR,       /*Index of product string*/
  USBD_IDX_SERIAL_STR,        /*Index of serial number string*/
  USBD_MAX_NUM_CONFIGURATION  /*bNumConfigurations*/
};

/** USB lang identifier descriptor. */
__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END =
{
     USB_LEN_LANGID_STR_DESC,
     USB_DESC_TYPE_STRING,
     LOBYTE(USBD_LANGID_STRING),
     HIBYTE(USBD_LANGID_STRING)
};

/* Internal string descriptor. */
__ALIGN_BEGIN uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;


__ALIGN_BEGIN uint8_t USBD_StringSerial[USB_SIZ_STRING_SERIAL] __ALIGN_END = {
  USB_SIZ_STRING_SERIAL,
  USB_DESC_TYPE_STRING,
};

/**
  * @brief  Return the device descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(USBD_FS_DeviceDesc);
  return USBD_FS_DeviceDesc;
}

/**
  * @brief  Return the LangID string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(USBD_LangIDDesc);
  return USBD_LangIDDesc;
}

/**
  * @brief  Return the product string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  if(speed == 0)
  {
    USBD_GetString((uint8_t *)USBD_PRODUCT_STRING_FS, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString((uint8_t *)USBD_PRODUCT_STRING_FS, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}

/**
  * @brief  Return the manufacturer string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

/**
  * @brief  Return the serial number string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = USB_SIZ_STRING_SERIAL;

  /* Update the serial number string descriptor with the data from the unique
   * ID */
  Get_SerialNum();
  /* USER CODE BEGIN USBD_FS_SerialStrDescriptor */

  /* USER CODE END USBD_FS_SerialStrDescriptor */
  return (uint8_t *) USBD_StringSerial;
}

/**
  * @brief  Return the configuration string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  if(speed == USBD_SPEED_HIGH)
  {
    USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING_FS, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING_FS, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}

/**
  * @brief  Return the interface string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  if(speed == 0)
  {
    USBD_GetString((uint8_t *)USBD_INTERFACE_STRING_FS, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString((uint8_t *)USBD_INTERFACE_STRING_FS, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}

/**
  * @brief  Create the serial number string descriptor
  * @param  None
  * @retval None
  */
void Get_SerialNum(void)
{
  uint32_t deviceserial0, deviceserial1, deviceserial2;

  deviceserial0 = *(uint32_t *) DEVICE_ID1;
  deviceserial1 = *(uint32_t *) DEVICE_ID2;
  deviceserial2 = *(uint32_t *) DEVICE_ID3;

  deviceserial0 += deviceserial2;

  if (deviceserial0 != 0)
  {
    IntToUnicode(deviceserial0, &USBD_StringSerial[2], 8);
    IntToUnicode(deviceserial1, &USBD_StringSerial[18], 4);
  }
}

/**
  * @brief  Convert Hex 32Bits value into char
  * @param  value: value to convert
  * @param  pbuf: pointer to the buffer
  * @param  len: buffer length
  * @retval None
  */
void IntToUnicode(uint32_t value, uint8_t * pbuf, uint8_t len)
{
  uint8_t idx = 0;

  for (idx = 0; idx < len; idx++)
  {
    if (((value >> 28)) < 0xA)
    {
      pbuf[2 * idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2 * idx] = (value >> 28) + 'A' - 10;
    }

    value = value << 4;

    pbuf[2 * idx + 1] = 0;
  }
}