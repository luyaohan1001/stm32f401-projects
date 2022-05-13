/**
 * @brief usb-cdc-device.c merged three classes: 
 *          1. usbd_cdc_if
 *          2. usb_device
 *          3. usbd_cdc
 */

/* usbd_cdc_if.c BEGIN---- ---- ---- ---- ---- ---- ---- ---- */

/* Includes ------------------------------------------------------------------*/
#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
// #include "usbd_cdc.h"
// #include "usbd_cdc_if.h"
#include "usb-cdc-device.h"
#include "usbd_def.h" /* defines USBD_StatusTypeDef */


/* usb_device_if layer ---- ---- ---- ---- ---- ---- ---- ---- */
/* Originally usbd_cdc_if.c */

/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];


extern USBD_HandleTypeDef hUsbDeviceFS;

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);


USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
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
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
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
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  /* USER CODE END 13 */
  return result;
}

/* usbd_cdc_if.c END ---- ---- ---- ---- ---- ---- ---- ---- */












/* usb_device.c BEGIN ---- ---- ---- ---- ---- ---- ---- ---- */
/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceFS;
// extern USBD_HandleTypeDef hUsbDeviceFS; /* from usbd_cdc_if.c */
USBD_HandleTypeDef* p_usbd_handle = &hUsbDeviceFS;

/**
  * Init USB device Library, add supported class and start the library
  * @retval None
  */
void usb_device_init(void)
{

  // USBD_Init(p_usbd_handle, &FS_Desc, DEVICE_FS);
  // USBD_RegisterClass(p_usbd_handle, &USBD_CDC);
  // USBD_CDC_RegisterInterface(p_usbd_handle, &USBD_Interface_fops_FS);
  // USBD_Start(p_usbd_handle);
  // return;

  /* Init Device Library, add supported class and start the library. */
  // 1. USBD_Init(&p_usbd_handle, &FS_Desc, DEVICE_FS);
  /* Unlink previous class*/
  p_usbd_handle->pClass[0] = NULL;
  p_usbd_handle->pUserData[0] = NULL;
  p_usbd_handle->pConfDesc = NULL;

  /* Assign USBD Descriptors */
  p_usbd_handle->pDesc = &FS_Desc;

  /* Set Device initial State */
  p_usbd_handle->dev_state = USBD_STATE_DEFAULT;  // DEFAULT - ADDRESSED - CONFIGURED - SUSPEND
  p_usbd_handle->id = DEVICE_FS; // DEVICE_FS (0)

  /* Initialize low level driver */
  // USBD_LL_Init(p_usbd_handle); // configure the TX or the RX FIFO of the new defined endport
                                  // From usbd_conf.c expanded below. 

  /* Init USB Ip. */
  extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
  if (p_usbd_handle->id == DEVICE_FS) {
  /* Link the driver to the stack. */
  hpcd_USB_OTG_FS.pData = p_usbd_handle;
                                           /* pData is the pointer to upper stack handler. */
  p_usbd_handle->pData = &hpcd_USB_OTG_FS; /* connect usbd_handle with the lower-level pcd_handle*/

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

  
  HAL_PCD_Init(&hpcd_USB_OTG_FS);

  /* in stm32f4xx_hal_pcd_ex.h */ 
  HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 0x80);    /* set RX FIFO size 0x80*/
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 0x40); /* set TX FIFO 0 with size 0x40 */ 
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1, 0x80); /* set TX FIFO 1 with size 0x80 */
  }
  // return USBD_OK;



  /*---------------------------------------------------------------------------*/
  // 2. USBD_RegisterClass(&p_usbd_handle, &USBD_CDC); // usbd_core.c
  
  /* link the class to the USB Device handle */
  /* This will populate the functions, the actual implementation is in usbd_cdc. */
  p_usbd_handle->pClass[0] = &USBD_CDC;

  /* Get Device Configuration Descriptor */
  /*
  uint16_t len = 0U;
  if (p_usbd_handle->pClass[p_usbd_handle->classId]->GetFSConfigDescriptor != NULL)
  {
    // pClass is of type USBD_ClassTypeDef.
    // p_usbd_handle is of type USBD_HandleTypeDef
    // The actual GetFSConfigDescriptor is implemented as  USBD_CDC_GetFSCfgDesc(uint16_t *length) in usbd_cdc.c
    p_usbd_handle->pConfDesc = (void *)p_usbd_handle->pClass[p_usbd_handle->classId]->GetFSConfigDescriptor(&len);
  }
  */
  // p_usbd_handle->pConfDesc = (void *)p_usbd_handle->pClass[p_usbd_handle->classId]->GetFSConfigDescriptor(0);
  // p_usbd_handle->pConfDesc = (void *)p_usbd_handle->pClass[0]->GetFSConfigDescriptor(0); 
  /* p_usbd_handle->pClass[0] = (USBD_CDC)*/
  p_usbd_handle->pConfDesc = (&USBD_CDC)->GetFSConfigDescriptor(0) ;

  /* Increment the NumClasses */
  // p_usbd_handle->NumClasses ++;

  /*---------------------------------------------------------------------------*/
  /*  Commented Out, USB still discoverable but not able to be used as VCP. */
  // 3. USBD_CDC_RegisterInterface(&p_usbd_handle, &USBD_Interface_fops_FS);
  /* USBD_Interface_fops_FS defined in usbd_cdc_if.c */
  /* Valid operations: CDC_Init_FS, CDC_DeInit_FS, CDC_Control_FS, CDC_Receive_FS, CDC*/
  p_usbd_handle->pUserData[p_usbd_handle->classId] = &USBD_Interface_fops_FS;

  /*---------------------------------------------------------------------------*/
  // 4. USBD_Startp_usbd_handle;
  /* Enable USB Transciver HAL_PCD_Start takes a PCD_HandleTypeDef *hpcd argument and initialize. */
  // HAL_PCD_Start(p_usbd_handle->pData);  /* p_usbd_handle->pData has type void */
                                           /* pData = hpcd_USB_OTG_FS was assigned in step 1. USBD_LL_Init */
  HAL_PCD_Start(&hpcd_USB_OTG_FS);
  /*---------------------------------------------------------------------------*/
}
/* usb_device.c END ---- ---- ---- ---- ---- ---- ---- ---- */



/* usbd_cdc.c START ---- ---- ---- ---- ---- ---- ---- ---- */
/* Includes ------------------------------------------------------------------*/
// #include "usbd_cdc.h"
#include "usbd_ctlreq.h"


static uint8_t USBD_CDC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_CDC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_CDC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_CDC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_CDC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_CDC_EP0_RxReady(USBD_HandleTypeDef *pdev);
#ifndef USE_USBD_COMPOSITE
static uint8_t *USBD_CDC_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_CDC_GetHSCfgDesc(uint16_t *length);
static uint8_t *USBD_CDC_GetOtherSpeedCfgDesc(uint16_t *length);
static uint8_t *USBD_CDC_GetOtherSpeedCfgDesc(uint16_t *length);
uint8_t *USBD_CDC_GetDeviceQualifierDescriptor(uint16_t *length);
#endif /* USE_USBD_COMPOSITE  */

#ifndef USE_USBD_COMPOSITE
/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CDC_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};
#endif /* USE_USBD_COMPOSITE  */
/**
  * @}
  */

/** @defgroup USBD_CDC_Private_Variables
  * @{
  */


/* CDC interface class callbacks structure */
USBD_ClassTypeDef  USBD_CDC =
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
#ifdef USE_USBD_COMPOSITE
  NULL,
  NULL,
  NULL,
  NULL,
#else
  USBD_CDC_GetHSCfgDesc,
  USBD_CDC_GetFSCfgDesc,
  USBD_CDC_GetOtherSpeedCfgDesc,
  USBD_CDC_GetDeviceQualifierDescriptor,
#endif /* USE_USBD_COMPOSITE  */
};

#ifndef USE_USBD_COMPOSITE
/* USB CDC device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CDC_CfgDesc[USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END =
{
  /* Configuration Descriptor */
  0x09,                                       /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,                /* bDescriptorType: Configuration */
  USB_CDC_CONFIG_DESC_SIZ,                    /* wTotalLength */
  0x00,
  0x02,                                       /* bNumInterfaces: 2 interfaces */
  0x01,                                       /* bConfigurationValue: Configuration value */
  0x00,                                       /* iConfiguration: Index of string descriptor
                                                 describing the configuration */
#if (USBD_SELF_POWERED == 1U)
  0xC0,                                       /* bmAttributes: Bus Powered according to user configuration */
#else
  0x80,                                       /* bmAttributes: Bus Powered according to user configuration */
#endif /* USBD_SELF_POWERED */
  USBD_MAX_POWER,                             /* MaxPower (mA) */

  /*---------------------------------------------------------------------------*/

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
#endif /* USE_USBD_COMPOSITE  */

static uint8_t CDCInEpAdd = CDC_IN_EP;
static uint8_t CDCOutEpAdd = CDC_OUT_EP;
static uint8_t CDCCmdEpAdd = CDC_CMD_EP;

/**
  * @}
  */

/** @defgroup USBD_CDC_Private_Functions
  * @{
  */

/**
  * @brief  USBD_CDC_Init
  *         Initialize the CDC interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_CDC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);
  USBD_CDC_HandleTypeDef *hcdc;

  hcdc = (USBD_CDC_HandleTypeDef *)USBD_malloc(sizeof(USBD_CDC_HandleTypeDef));

  if (hcdc == NULL)
  {
    pdev->pClassDataCmsit[pdev->classId] = NULL;
    return (uint8_t)USBD_EMEM;
  }

  (void)USBD_memset(hcdc, 0, sizeof(USBD_CDC_HandleTypeDef));

  pdev->pClassDataCmsit[pdev->classId] = (void *)hcdc;
  pdev->pClassData = pdev->pClassDataCmsit[pdev->classId];

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  CDCInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_BULK);
  CDCOutEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_OUT, USBD_EP_TYPE_BULK);
  CDCCmdEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_INTR);
#endif /* USE_USBD_COMPOSITE */

  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    /* Open EP IN */
    (void)USBD_LL_OpenEP(pdev, CDCInEpAdd, USBD_EP_TYPE_BULK,
                         CDC_DATA_HS_IN_PACKET_SIZE);

    pdev->ep_in[CDCInEpAdd & 0xFU].is_used = 1U;

    /* Open EP OUT */
    (void)USBD_LL_OpenEP(pdev, CDCOutEpAdd, USBD_EP_TYPE_BULK,
                         CDC_DATA_HS_OUT_PACKET_SIZE);

    pdev->ep_out[CDCOutEpAdd & 0xFU].is_used = 1U;

    /* Set bInterval for CDC CMD Endpoint */
    pdev->ep_in[CDCCmdEpAdd & 0xFU].bInterval = CDC_HS_BINTERVAL;
  }
  else
  {
    /* Open EP IN */
    (void)USBD_LL_OpenEP(pdev, CDCInEpAdd, USBD_EP_TYPE_BULK,
                         CDC_DATA_FS_IN_PACKET_SIZE);

    pdev->ep_in[CDCInEpAdd & 0xFU].is_used = 1U;

    /* Open EP OUT */
    (void)USBD_LL_OpenEP(pdev, CDCOutEpAdd, USBD_EP_TYPE_BULK,
                         CDC_DATA_FS_OUT_PACKET_SIZE);

    pdev->ep_out[CDCOutEpAdd & 0xFU].is_used = 1U;

    /* Set bInterval for CMD Endpoint */
    pdev->ep_in[CDCCmdEpAdd & 0xFU].bInterval = CDC_FS_BINTERVAL;
  }

  /* Open Command IN EP */
  (void)USBD_LL_OpenEP(pdev, CDCCmdEpAdd, USBD_EP_TYPE_INTR, CDC_CMD_PACKET_SIZE);
  pdev->ep_in[CDCCmdEpAdd & 0xFU].is_used = 1U;

  hcdc->RxBuffer = NULL;

  /* Init  physical Interface components */
  ((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Init();

  /* Init Xfer states */
  hcdc->TxState = 0U;
  hcdc->RxState = 0U;

  if (hcdc->RxBuffer == NULL)
  {
    return (uint8_t)USBD_EMEM;
  }

  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    /* Prepare Out endpoint to receive next packet */
    (void)USBD_LL_PrepareReceive(pdev, CDCOutEpAdd, hcdc->RxBuffer,
                                 CDC_DATA_HS_OUT_PACKET_SIZE);
  }
  else
  {
    /* Prepare Out endpoint to receive next packet */
    (void)USBD_LL_PrepareReceive(pdev, CDCOutEpAdd, hcdc->RxBuffer,
                                 CDC_DATA_FS_OUT_PACKET_SIZE);
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_Init
  *         DeInitialize the CDC layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_CDC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);


#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this CDC class instance */
  CDCInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_BULK);
  CDCOutEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_OUT, USBD_EP_TYPE_BULK);
  CDCCmdEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_INTR);
#endif /* USE_USBD_COMPOSITE */

  /* Close EP IN */
  (void)USBD_LL_CloseEP(pdev, CDCInEpAdd);
  pdev->ep_in[CDCInEpAdd & 0xFU].is_used = 0U;

  /* Close EP OUT */
  (void)USBD_LL_CloseEP(pdev, CDCOutEpAdd);
  pdev->ep_out[CDCOutEpAdd & 0xFU].is_used = 0U;

  /* Close Command IN EP */
  (void)USBD_LL_CloseEP(pdev, CDCCmdEpAdd);
  pdev->ep_in[CDCCmdEpAdd & 0xFU].is_used = 0U;
  pdev->ep_in[CDCCmdEpAdd & 0xFU].bInterval = 0U;

  /* DeInit  physical Interface components */
  if (pdev->pClassDataCmsit[pdev->classId] != NULL)
  {
    ((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->DeInit();
    (void)USBD_free(pdev->pClassDataCmsit[pdev->classId]);
    pdev->pClassDataCmsit[pdev->classId] = NULL;
    pdev->pClassData = NULL;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_Setup
  *         Handle the CDC specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_CDC_Setup(USBD_HandleTypeDef *pdev,
                              USBD_SetupReqTypedef *req)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
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
          ((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Control(req->bRequest,
                                                                           (uint8_t *)hcdc->data,
                                                                           req->wLength);

          len = MIN(CDC_REQ_MAX_DATA_SIZE, req->wLength);
          (void)USBD_CtlSendData(pdev, (uint8_t *)hcdc->data, len);
        }
        else
        {
          hcdc->CmdOpCode = req->bRequest;
          hcdc->CmdLength = (uint8_t)MIN(req->wLength, USB_MAX_EP0_SIZE);

          (void)USBD_CtlPrepareRx(pdev, (uint8_t *)hcdc->data, hcdc->CmdLength);
        }
      }
      else
      {
        ((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Control(req->bRequest,
                                                                         (uint8_t *)req, 0U);
      }
      break;

    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        case USB_REQ_GET_STATUS:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            (void)USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_GET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            (void)USBD_CtlSendData(pdev, &ifalt, 1U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_SET_INTERFACE:
          if (pdev->dev_state != USBD_STATE_CONFIGURED)
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_CLEAR_FEATURE:
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }

  return (uint8_t)ret;
}

/**
  * @brief  USBD_CDC_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t USBD_CDC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_CDC_HandleTypeDef *hcdc;
  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef *)pdev->pData;

  if (pdev->pClassDataCmsit[pdev->classId] == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if ((pdev->ep_in[epnum & 0xFU].total_length > 0U) &&
      ((pdev->ep_in[epnum & 0xFU].total_length % hpcd->IN_ep[epnum & 0xFU].maxpacket) == 0U))
  {
    /* Update the packet total length */
    pdev->ep_in[epnum & 0xFU].total_length = 0U;

    /* Send ZLP */
    (void)USBD_LL_Transmit(pdev, epnum, NULL, 0U);
  }
  else
  {
    hcdc->TxState = 0U;

    if (((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->TransmitCplt != NULL)
    {
      ((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->TransmitCplt(hcdc->TxBuffer, &hcdc->TxLength, epnum);
    }
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t USBD_CDC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (pdev->pClassDataCmsit[pdev->classId] == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  /* Get the received data length */
  hcdc->RxLength = USBD_LL_GetRxDataSize(pdev, epnum);

  /* USB data will be immediately processed, this allow next USB traffic being
  NAKed till the end of the application Xfer */

  ((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Receive(hcdc->RxBuffer, &hcdc->RxLength);

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_EP0_RxReady
  *         Handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_CDC_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (hcdc == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if ((pdev->pUserData[pdev->classId] != NULL) && (hcdc->CmdOpCode != 0xFFU))
  {
    ((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Control(hcdc->CmdOpCode,
                                                                     (uint8_t *)hcdc->data,
                                                                     (uint16_t)hcdc->CmdLength);
    hcdc->CmdOpCode = 0xFFU;
  }

  return (uint8_t)USBD_OK;
}
#ifndef USE_USBD_COMPOSITE
/**
  * @brief  USBD_CDC_GetFSCfgDesc
  *         Return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_CDC_GetFSCfgDesc(uint16_t *length)
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
  * @brief  USBD_CDC_GetHSCfgDesc
  *         Return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_CDC_GetHSCfgDesc(uint16_t *length)
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
static uint8_t *USBD_CDC_GetOtherSpeedCfgDesc(uint16_t *length)
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
  * @brief  USBD_CDC_GetDeviceQualifierDescriptor
  *         return Device Qualifier descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
uint8_t *USBD_CDC_GetDeviceQualifierDescriptor(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_CDC_DeviceQualifierDesc);

  return USBD_CDC_DeviceQualifierDesc;
}
#endif /* USE_USBD_COMPOSITE  */
/**
  * @brief  USBD_CDC_RegisterInterface
  * @param  pdev: device instance
  * @param  fops: CD  Interface callback
  * @retval status
  */
uint8_t USBD_CDC_RegisterInterface(USBD_HandleTypeDef *pdev,
                                   USBD_CDC_ItfTypeDef *fops)
{
  if (fops == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  pdev->pUserData[pdev->classId] = fops;

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_SetTxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Tx Buffer
  * @param  length: Tx Buffer length
  * @retval status
  */
uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *pdev,
                             uint8_t *pbuff, uint32_t length)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

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
  * @param  pdev: device instance
  * @param  pbuff: Rx Buffer
  * @retval status
  */
uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (hcdc == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  hcdc->RxBuffer = pbuff;

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_TransmitPacket
  *         Transmit packet on IN endpoint
  * @param  pdev: device instance
  * @retval status
  */
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *pdev)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  USBD_StatusTypeDef ret = USBD_BUSY;

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  CDCInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_BULK);
#endif /* USE_USBD_COMPOSITE */
  if (pdev->pClassDataCmsit[pdev->classId] == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (hcdc->TxState == 0U)
  {
    /* Tx Transfer in progress */
    hcdc->TxState = 1U;

    /* Update the packet total length */
    pdev->ep_in[CDCInEpAdd & 0xFU].total_length = hcdc->TxLength;

    /* Transmit next packet */
    (void)USBD_LL_Transmit(pdev, CDCInEpAdd, hcdc->TxBuffer, hcdc->TxLength);

    ret = USBD_OK;
  }

  return (uint8_t)ret;
}

/**
  * @brief  USBD_CDC_ReceivePacket
  *         prepare OUT Endpoint for reception
  * @param  pdev: device instance
  * @retval status
  */
uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *pdev)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  CDCOutEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_OUT, USBD_EP_TYPE_BULK);
#endif /* USE_USBD_COMPOSITE */

  if (pdev->pClassDataCmsit[pdev->classId] == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    /* Prepare Out endpoint to receive next packet */
    (void)USBD_LL_PrepareReceive(pdev, CDCOutEpAdd, hcdc->RxBuffer,
                                 CDC_DATA_HS_OUT_PACKET_SIZE);
  }
  else
  {
    /* Prepare Out endpoint to receive next packet */
    (void)USBD_LL_PrepareReceive(pdev, CDCOutEpAdd, hcdc->RxBuffer,
                                 CDC_DATA_FS_OUT_PACKET_SIZE);
  }

  return (uint8_t)USBD_OK;
}

/* usbd_cdc.c END ---- ---- ---- ---- ---- ---- ---- ---- */