
#ifndef __USB_CDC_DEVICE_H
#define __USB_CDC_DEVICE_H

/* Includes ------------------------------------------------------------------*/
/* usb_device.h ---- ---- ---- ---- */
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#ifndef __USBD_DEF_H
#define __USBD_DEF_H


/* usbd_def.h BEGIN ---- ---- ---- ---- */
#ifndef NULL
#define NULL                                            0U
#endif /* NULL */

#ifndef USBD_MAX_NUM_INTERFACES
#define USBD_MAX_NUM_INTERFACES                         1U
#endif /* USBD_MAX_NUM_CONFIGURATION */

#ifndef USBD_MAX_NUM_CONFIGURATION
#define USBD_MAX_NUM_CONFIGURATION                      1U
#endif /* USBD_MAX_NUM_CONFIGURATION */

#ifndef USBD_MAX_CLASS_ENDPOINTS
#define USBD_MAX_CLASS_ENDPOINTS                       5U
#endif /* USBD_MAX_CLASS_ENDPOINTS */

#ifndef USBD_MAX_CLASS_INTERFACES
#define USBD_MAX_CLASS_INTERFACES                      5U
#endif /* USBD_MAX_CLASS_INTERFACES */

#ifndef USBD_LPM_ENABLED
#define USBD_LPM_ENABLED                                0U
#endif /* USBD_LPM_ENABLED */

#ifndef USBD_SELF_POWERED
#define USBD_SELF_POWERED                               1U
#endif /*USBD_SELF_POWERED */

#ifndef USBD_MAX_POWER
#define USBD_MAX_POWER                                  0x32U /* 100 mA */
#endif /* USBD_MAX_POWER */

#ifndef USBD_SUPPORT_USER_STRING_DESC
#define USBD_SUPPORT_USER_STRING_DESC                   0U
#endif /* USBD_SUPPORT_USER_STRING_DESC */

#ifndef USBD_CLASS_USER_STRING_DESC
#define USBD_CLASS_USER_STRING_DESC                     0U
#endif /* USBD_CLASS_USER_STRING_DESC */

#define  USB_LEN_DEV_QUALIFIER_DESC                     0x0AU
#define  USB_LEN_DEV_DESC                               0x12U
#define  USB_LEN_CFG_DESC                               0x09U
#define  USB_LEN_IF_DESC                                0x09U
#define  USB_LEN_EP_DESC                                0x07U
#define  USB_LEN_OTG_DESC                               0x03U
#define  USB_LEN_LANGID_STR_DESC                        0x04U
#define  USB_LEN_OTHER_SPEED_DESC_SIZ                   0x09U

#define  USBD_IDX_LANGID_STR                            0x00U
#define  USBD_IDX_MFC_STR                               0x01U
#define  USBD_IDX_PRODUCT_STR                           0x02U
#define  USBD_IDX_SERIAL_STR                            0x03U
#define  USBD_IDX_CONFIG_STR                            0x04U
#define  USBD_IDX_INTERFACE_STR                         0x05U

#define  USB_REQ_TYPE_STANDARD                          0x00U
#define  USB_REQ_TYPE_CLASS                             0x20U
#define  USB_REQ_TYPE_VENDOR                            0x40U
#define  USB_REQ_TYPE_MASK                              0x60U

#define  USB_REQ_RECIPIENT_DEVICE                       0x00U
#define  USB_REQ_RECIPIENT_INTERFACE                    0x01U
#define  USB_REQ_RECIPIENT_ENDPOINT                     0x02U
#define  USB_REQ_RECIPIENT_MASK                         0x03U

#define  USB_REQ_GET_STATUS                             0x00U
#define  USB_REQ_CLEAR_FEATURE                          0x01U
#define  USB_REQ_SET_FEATURE                            0x03U
#define  USB_REQ_SET_ADDRESS                            0x05U
#define  USB_REQ_GET_DESCRIPTOR                         0x06U
#define  USB_REQ_SET_DESCRIPTOR                         0x07U
#define  USB_REQ_GET_CONFIGURATION                      0x08U
#define  USB_REQ_SET_CONFIGURATION                      0x09U
#define  USB_REQ_GET_INTERFACE                          0x0AU
#define  USB_REQ_SET_INTERFACE                          0x0BU
#define  USB_REQ_SYNCH_FRAME                            0x0CU

#define  USB_DESC_TYPE_DEVICE                           0x01U
#define  USB_DESC_TYPE_CONFIGURATION                    0x02U
#define  USB_DESC_TYPE_STRING                           0x03U
#define  USB_DESC_TYPE_INTERFACE                        0x04U
#define  USB_DESC_TYPE_ENDPOINT                         0x05U
#define  USB_DESC_TYPE_DEVICE_QUALIFIER                 0x06U
#define  USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION        0x07U
#define  USB_DESC_TYPE_IAD                              0x0BU
#define  USB_DESC_TYPE_BOS                              0x0FU

#define USB_CONFIG_REMOTE_WAKEUP                        0x02U
#define USB_CONFIG_SELF_POWERED                         0x01U

#define USB_FEATURE_EP_HALT                             0x00U
#define USB_FEATURE_REMOTE_WAKEUP                       0x01U
#define USB_FEATURE_TEST_MODE                           0x02U

#define USB_DEVICE_CAPABITY_TYPE                        0x10U

#define USB_CONF_DESC_SIZE                              0x09U
#define USB_IF_DESC_SIZE                                0x09U
#define USB_EP_DESC_SIZE                                0x07U
#define USB_IAD_DESC_SIZE                               0x08U

#define USB_HS_MAX_PACKET_SIZE                          512U
#define USB_FS_MAX_PACKET_SIZE                          64U
#define USB_MAX_EP0_SIZE                                64U

/*  Device Status */
#define USBD_STATE_DEFAULT                              0x01U
#define USBD_STATE_ADDRESSED                            0x02U
#define USBD_STATE_CONFIGURED                           0x03U
#define USBD_STATE_SUSPENDED                            0x04U


/*  EP0 State */
#define USBD_EP0_IDLE                                   0x00U
#define USBD_EP0_SETUP                                  0x01U
#define USBD_EP0_DATA_IN                                0x02U
#define USBD_EP0_DATA_OUT                               0x03U
#define USBD_EP0_STATUS_IN                              0x04U
#define USBD_EP0_STATUS_OUT                             0x05U
#define USBD_EP0_STALL                                  0x06U

#define USBD_EP_TYPE_CTRL                               0x00U
#define USBD_EP_TYPE_ISOC                               0x01U
#define USBD_EP_TYPE_BULK                               0x02U
#define USBD_EP_TYPE_INTR                               0x03U

#ifdef USE_USBD_COMPOSITE
#define USBD_EP_IN                                      0x80U
#define USBD_EP_OUT                                     0x00U
#define USBD_FUNC_DESCRIPTOR_TYPE                       0x24U
#define USBD_DESC_SUBTYPE_ACM                           0x0FU
#define USBD_DESC_ECM_BCD_LOW                           0x00U
#define USBD_DESC_ECM_BCD_HIGH                          0x10U
#endif /* USE_USBD_COMPOSITE */

typedef  struct  usb_setup_req
{
  uint8_t   bmRequest;
  uint8_t   bRequest;
  uint16_t  wValue;
  uint16_t  wIndex;
  uint16_t  wLength;
} USBD_SetupReqTypedef;

typedef struct
{
  uint8_t   bLength;
  uint8_t   bDescriptorType;
  uint16_t  wTotalLength;
  uint8_t   bNumInterfaces;
  uint8_t   bConfigurationValue;
  uint8_t   iConfiguration;
  uint8_t   bmAttributes;
  uint8_t   bMaxPower;
} __PACKED USBD_ConfigDescTypeDef;

typedef struct
{
  uint8_t   bLength;
  uint8_t   bDescriptorType;
  uint16_t  wTotalLength;
  uint8_t   bNumDeviceCaps;
} USBD_BosDescTypeDef;

typedef struct
{
  uint8_t   bLength;
  uint8_t   bDescriptorType;
  uint8_t   bEndpointAddress;
  uint8_t   bmAttributes;
  uint16_t  wMaxPacketSize;
  uint8_t   bInterval;
} __PACKED USBD_EpDescTypeDef;

typedef  struct
{
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint8_t  bDescriptorSubType;
} USBD_DescHeaderTypeDef;

struct _USBD_Handle_t;

typedef struct _Device_cb
{
  uint8_t (*Init)(struct _USBD_Handle_t *pdev, uint8_t cfgidx);
  uint8_t (*DeInit)(struct _USBD_Handle_t *pdev, uint8_t cfgidx);
  /* Control Endpoints*/
  uint8_t (*Setup)(struct _USBD_Handle_t *pdev, USBD_SetupReqTypedef  *req);
  uint8_t (*EP0_TxSent)(struct _USBD_Handle_t *pdev);
  uint8_t (*EP0_RxReady)(struct _USBD_Handle_t *pdev);
  /* Class Specific Endpoints*/
  uint8_t (*DataIn)(struct _USBD_Handle_t *pdev, uint8_t epnum);
  uint8_t (*DataOut)(struct _USBD_Handle_t *pdev, uint8_t epnum);
  uint8_t (*SOF)(struct _USBD_Handle_t *pdev);
  uint8_t (*IsoINIncomplete)(struct _USBD_Handle_t *pdev, uint8_t epnum);
  uint8_t (*IsoOUTIncomplete)(struct _USBD_Handle_t *pdev, uint8_t epnum);

  uint8_t  *(*GetHSConfigDescriptor)(uint16_t *length);
  uint8_t  *(*GetFSConfigDescriptor)(uint16_t *length);
  uint8_t  *(*GetOtherSpeedConfigDescriptor)(uint16_t *length);
  uint8_t  *(*GetDeviceQualifierDescriptor)(uint16_t *length);
#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
  uint8_t  *(*GetUsrStrDescriptor)(struct _USBD_Handle_t *pdev, uint8_t index,  uint16_t *length);
#endif /* USBD_SUPPORT_USER_STRING_DESC  */

} USBD_ClassTypeDef;

/* USB Device Speed */
typedef enum
{
  USBD_SPEED_HIGH  = 0U,
  USBD_SPEED_FULL  = 1U,
  USBD_SPEED_LOW   = 2U,
} USBD_SpeedTypeDef;

/* USB Device status */
typedef enum
{
  USBD_OK = 0U,
  USBD_BUSY,
  USBD_EMEM,
  USBD_FAIL,
} USBD_StatusTypeDef;

/* USB Device descriptors structure */
typedef struct
{
  uint8_t *(*GetDeviceDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetLangIDStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetManufacturerStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetProductStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetSerialStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetConfigurationStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetInterfaceStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
#if (USBD_CLASS_USER_STRING_DESC == 1)
  uint8_t *(*GetUserStrDescriptor)(USBD_SpeedTypeDef speed, uint8_t idx, uint16_t *length);
#endif /* USBD_CLASS_USER_STRING_DESC */
#if ((USBD_LPM_ENABLED == 1U) || (USBD_CLASS_BOS_ENABLED == 1))
  uint8_t *(*GetBOSDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
#endif /* (USBD_LPM_ENABLED == 1U) || (USBD_CLASS_BOS_ENABLED == 1) */
} USBD_DescriptorsTypeDef;

/* USB Device handle structure */
typedef struct
{
  uint32_t status;
  uint32_t total_length;
  uint32_t rem_length;
  uint32_t maxpacket;
  uint16_t is_used;
  uint16_t bInterval;
} USBD_EndpointTypeDef;

#ifdef USE_USBD_COMPOSITE
typedef enum
{
  CLASS_TYPE_NONE    = 0,
  CLASS_TYPE_HID     = 1,
  CLASS_TYPE_CDC     = 2,
  CLASS_TYPE_MSC     = 3,
  CLASS_TYPE_DFU     = 4,
  CLASS_TYPE_CHID    = 5,
  CLASS_TYPE_AUDIO   = 6,
  CLASS_TYPE_ECM     = 7,
  CLASS_TYPE_RNDIS   = 8,
  CLASS_TYPE_MTP     = 9,
  CLASS_TYPE_VIDEO   = 10,
  CLASS_TYPE_PRINTER = 11,
  CLASS_TYPE_CCID    = 12,
} USBD_CompositeClassTypeDef;


/* USB Device handle structure */
typedef struct
{
  uint8_t                     add;
  uint8_t                     type;
  uint8_t                     size;
  uint8_t                     is_used;
} USBD_EPTypeDef;

/* USB Device handle structure */
typedef struct
{
  USBD_CompositeClassTypeDef   ClassType;
  uint32_t                     ClassId;
  uint32_t                     Active;
  uint32_t                     NumEps;
  USBD_EPTypeDef               Eps[USBD_MAX_CLASS_ENDPOINTS];
  uint8_t                      *EpAdd;
  uint32_t                     NumIf;
  uint8_t                      Ifs[USBD_MAX_CLASS_INTERFACES];
  uint32_t                     CurrPcktSze;
} USBD_CompositeElementTypeDef;
#endif /* USE_USBD_COMPOSITE */



/* USB Device handle structure */
typedef struct _USBD_Handle_t
{
  uint8_t                 id;                   /* DEVICE_FS or DEVICE_HS */
  uint32_t                dev_config;
  uint32_t                dev_default_config;
  uint32_t                dev_config_status;
  USBD_SpeedTypeDef       dev_speed;
  USBD_EndpointTypeDef    ep_in[16];
  USBD_EndpointTypeDef    ep_out[16];
  __IO uint32_t           ep0_state;
  uint32_t                ep0_data_len;
  __IO uint8_t            dev_state;
  __IO uint8_t            dev_old_state;
  uint8_t                 dev_address;
  uint8_t                 dev_connection_status;
  uint32_t                dev_remote_wakeup;
  uint8_t                 ConfIdx;

  USBD_SetupReqTypedef    request;
  USBD_DescriptorsTypeDef *pDesc;
  USBD_ClassTypeDef       *pClass;          /* &USBD_CDC */
  void                    *pClassData;
  void                    *pClassDataCmsit;
  void                    *pUserData;
  void                    *pData;
  void                    *pBosDesc;
  void                    *pConfDesc;
  uint32_t                NumClasses;
} USBD_Handle_t;

/* USB Device endpoint direction */
typedef enum
{
  OUT   = 0x00,
  IN    = 0x80,
} USBD_EPDirectionTypeDef;

typedef enum
{
  NETWORK_CONNECTION = 0x00,
  RESPONSE_AVAILABLE = 0x01,
  CONNECTION_SPEED_CHANGE = 0x2A
} USBD_CDC_NotifCodeTypeDef;


__STATIC_INLINE uint16_t SWAPBYTE(uint8_t *addr)
{
  uint16_t _SwapVal, _Byte1, _Byte2;
  uint8_t *_pbuff = addr;

  _Byte1 = *(uint8_t *)_pbuff;
  _pbuff++;
  _Byte2 = *(uint8_t *)_pbuff;

  _SwapVal = (_Byte2 << 8) | _Byte1;

  return _SwapVal;
}


#ifndef LOBYTE
#define LOBYTE(x)  ((uint8_t)((x) & 0x00FFU))
#endif /* LOBYTE */

#ifndef HIBYTE
#define HIBYTE(x)  ((uint8_t)(((x) & 0xFF00U) >> 8U))
#endif /* HIBYTE */

#ifndef MIN
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#endif /* MIN */

#ifndef MAX
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))
#endif /* MAX */

#if  defined ( __GNUC__ )
#ifndef __weak
#define __weak   __attribute__((weak))
#endif /* __weak */
#ifndef __packed
#define __packed __attribute__((__packed__))
#endif /* __packed */
#endif /* __GNUC__ */


/* In HS mode and when the DMA is used, all variables and data structures dealing
   with the DMA during the transaction process should be 4-bytes aligned */

#if defined ( __GNUC__ ) && !defined (__CC_ARM) /* GNU Compiler */
#ifndef __ALIGN_END
#define __ALIGN_END    __attribute__ ((aligned (4U)))
#endif /* __ALIGN_END */
#ifndef __ALIGN_BEGIN
#define __ALIGN_BEGIN
#endif /* __ALIGN_BEGIN */
#else
#ifndef __ALIGN_END
#define __ALIGN_END
#endif /* __ALIGN_END */
#ifndef __ALIGN_BEGIN
#if defined   (__CC_ARM)      /* ARM Compiler */
#define __ALIGN_BEGIN    __align(4U)
#elif defined (__ICCARM__)    /* IAR Compiler */
#define __ALIGN_BEGIN
#endif /* __CC_ARM */
#endif /* __ALIGN_BEGIN */
#endif /* __GNUC__ */


#endif /* __USBD_DEF_H */


/* Macro define ------------------------------------------------------------------*/
#define USBD_SOF          USBD_LL_SOF
/* Define size for the receive and transmit buffer over CDC */
#define APP_RX_DATA_SIZE  2048
#define APP_TX_DATA_SIZE  2048

#ifndef CDC_IN_EP
#define CDC_IN_EP                                   0x81U  /* EP1 for data IN */
#endif /* CDC_IN_EP */
#ifndef CDC_OUT_EP
#define CDC_OUT_EP                                  0x01U  /* EP1 for data OUT */
#endif /* CDC_OUT_EP */
#ifndef CDC_CMD_EP
#define CDC_CMD_EP                                  0x82U  /* EP2 for CDC commands */
#endif /* CDC_CMD_EP  */

#ifndef CDC_HS_BINTERVAL
#define CDC_HS_BINTERVAL                            0x10U
#endif /* CDC_HS_BINTERVAL */

#ifndef CDC_FS_BINTERVAL
#define CDC_FS_BINTERVAL                            0x10U
#endif /* CDC_FS_BINTERVAL */

/* CDC Endpoints parameters: you can fine tune these values depending on the needed baudrates and performance. */
#define CDC_DATA_HS_MAX_PACKET_SIZE                 512U  /* Endpoint IN & OUT Packet size */
#define CDC_DATA_FS_MAX_PACKET_SIZE                 64U  /* Endpoint IN & OUT Packet size */
#define CDC_CMD_PACKET_SIZE                         8U  /* Control Endpoint Packet size */

#define USB_CDC_CONFIG_DESC_SIZ                     67U
#define CDC_DATA_HS_IN_PACKET_SIZE                  CDC_DATA_HS_MAX_PACKET_SIZE
#define CDC_DATA_HS_OUT_PACKET_SIZE                 CDC_DATA_HS_MAX_PACKET_SIZE

#define CDC_DATA_FS_IN_PACKET_SIZE                  CDC_DATA_FS_MAX_PACKET_SIZE
#define CDC_DATA_FS_OUT_PACKET_SIZE                 CDC_DATA_FS_MAX_PACKET_SIZE

#define CDC_REQ_MAX_DATA_SIZE                       0x7U
/*---------------------------------------------------------------------*/
/*  CDC definitions                                                    */
/*---------------------------------------------------------------------*/
#define CDC_SEND_ENCAPSULATED_COMMAND               0x00U
#define CDC_GET_ENCAPSULATED_RESPONSE               0x01U
#define CDC_SET_COMM_FEATURE                        0x02U
#define CDC_GET_COMM_FEATURE                        0x03U
#define CDC_CLEAR_COMM_FEATURE                      0x04U
#define CDC_SET_LINE_CODING                         0x20U
#define CDC_GET_LINE_CODING                         0x21U
#define CDC_SET_CONTROL_LINE_STATE                  0x22U
#define CDC_SEND_BREAK                              0x23U

#define USBD_CDC_CLASS &USBD_CDC



/* USB Device handle structure */
struct USB_Device_Handle
{
  uint8_t                 id;
  uint32_t                dev_config;
  uint32_t                dev_default_config;
  uint32_t                dev_config_status;
  USBD_SpeedTypeDef       dev_speed;
  USBD_EndpointTypeDef    ep_in[16];
  USBD_EndpointTypeDef    ep_out[16];
  __IO uint32_t           ep0_state;
  uint32_t                ep0_data_len;
  __IO uint8_t            dev_state;
  __IO uint8_t            dev_old_state;
  uint8_t                 dev_address;
  uint8_t                 dev_connection_status;
  uint8_t                 dev_test_mode;
  uint32_t                dev_remote_wakeup;
  uint8_t                 ConfIdx;

  USBD_SetupReqTypedef    request;
  USBD_DescriptorsTypeDef *pDesc;
  USBD_ClassTypeDef       *pClass[1];
  void                    *pClassData;
  void                    *pClassDataCmsit[1];
  void                    *pUserData[1];
  void                    *pData;
  void                    *pBosDesc;
  void                    *pConfDesc;
  uint32_t                classId;
  uint32_t                NumClasses;
};

/** USB Device initialization function. */
void usb_device_init(void);   // equvilent to  void MX_USB_DEVICE_Init(void)
/* usb_device.h END ---- ---- ---- ---- ---- ---- ---- ---- */


/* usbd_cdc.h END ---- ---- ---- ---- ---- ---- ---- ---- */
typedef struct
{
  uint32_t bitrate;
  uint8_t  format;
  uint8_t  paritytype;
  uint8_t  datatype;
} USBD_CDC_LineCodingTypeDef;

typedef struct _USBD_CDC_Itf
{
  int8_t (* Init)(void);
  int8_t (* DeInit)(void);
  int8_t (* Control)(uint8_t cmd, uint8_t *pbuf, uint16_t length);
  int8_t (* Receive)(uint8_t *Buf, uint32_t *Len);
  int8_t (* TransmitCplt)(uint8_t *Buf, uint32_t *Len, uint8_t epnum);
} USBD_CDC_ItfTypeDef;


typedef struct
{
  uint32_t data[CDC_DATA_HS_MAX_PACKET_SIZE / 4U];      /* Force 32-bit alignment */
  uint8_t  CmdOpCode;
  uint8_t  CmdLength;
  uint8_t  *RxBuffer;
  uint8_t  *TxBuffer;
  uint32_t RxLength;
  uint32_t TxLength;

  __IO uint32_t TxState;
  __IO uint32_t RxState;
} USBD_CDC_Buffer_t;

extern USBD_ClassTypeDef USBD_CDC;


uint8_t USBD_CDC_RegisterInterface(USBD_Handle_t *pdev,
                                   USBD_CDC_ItfTypeDef *fops);

uint8_t USBD_CDC_SetTxBuffer(USBD_Handle_t *pdev, uint8_t *pbuff,
                             uint32_t length);

uint8_t USBD_CDC_SetRxBuffer(USBD_Handle_t *pdev, uint8_t *pbuff);
uint8_t USBD_CDC_ReceivePacket(USBD_Handle_t *pdev);
uint8_t USBD_CDC_TransmitPacket(USBD_Handle_t *pdev);
/* usbd_cdc.h END ---- ---- ---- ---- ---- ---- ---- ---- */



/* usbd_core.h BEGIN ---- ---- ---- ---- ---- ---- ---- ---- */
USBD_StatusTypeDef USBD_Init(USBD_Handle_t *pdev, USBD_DescriptorsTypeDef *pdesc, uint8_t id);
USBD_StatusTypeDef USBD_DeInit(USBD_Handle_t *pdev);
USBD_StatusTypeDef USBD_Start(USBD_Handle_t *pdev);
USBD_StatusTypeDef USBD_Stop(USBD_Handle_t *pdev);
USBD_StatusTypeDef USBD_RegisterClass(USBD_Handle_t *pdev, USBD_ClassTypeDef *pclass);

uint8_t USBD_CoreFindIF(USBD_Handle_t *pdev, uint8_t index);
uint8_t USBD_CoreFindEP(USBD_Handle_t *pdev, uint8_t index);

void USBD_SetClassConfig(USBD_Handle_t *pdev, uint8_t cfgidx);
USBD_StatusTypeDef USBD_ClrClassConfig(USBD_Handle_t *pdev, uint8_t cfgidx);

void USBD_LL_SetupStage(USBD_Handle_t *pdev, uint8_t *psetup);
USBD_StatusTypeDef USBD_LL_DataOutStage(USBD_Handle_t *pdev, uint8_t epnum, uint8_t *pdata);

USBD_StatusTypeDef USBD_LL_Reset(USBD_Handle_t *pdev);
USBD_StatusTypeDef USBD_LL_Suspend(USBD_Handle_t *pdev);
USBD_StatusTypeDef USBD_LL_Resume(USBD_Handle_t *pdev);

USBD_StatusTypeDef USBD_LL_SOF(USBD_Handle_t  *pdev);
USBD_StatusTypeDef USBD_LL_IsoINIncomplete(USBD_Handle_t *pdev, uint8_t epnum);

USBD_StatusTypeDef USBD_LL_DevConnected(USBD_Handle_t *pdev);
USBD_StatusTypeDef USBD_LL_DevDisconnected(USBD_Handle_t *pdev);

/* USBD Low Level Driver */
USBD_StatusTypeDef USBD_LL_Init(USBD_Handle_t *pdev);
USBD_StatusTypeDef USBD_LL_DeInit(USBD_Handle_t *pdev);
USBD_StatusTypeDef USBD_LL_Start(USBD_Handle_t *pdev);
void USBD_LL_Stop(USBD_Handle_t *pdev);

void USBD_LL_OpenEP(USBD_Handle_t *pdev, uint8_t ep_addr,
                                  uint8_t ep_type, uint16_t ep_mps);

void USBD_LL_CloseEP(USBD_Handle_t *pdev, uint8_t ep_addr);
void USBD_LL_FlushEP(USBD_Handle_t *pdev, uint8_t ep_addr);
void USBD_LL_StallEP(USBD_Handle_t *pdev, uint8_t ep_addr);
void USBD_LL_ClearStallEP(USBD_Handle_t *pdev, uint8_t ep_addr);
void USBD_LL_SetUSBAddress(USBD_Handle_t *pdev, uint8_t dev_addr);

void USBD_LL_Transmit(USBD_Handle_t *pdev, uint8_t ep_addr,
                                    uint8_t *pbuf, uint32_t size);

void USBD_LL_PrepareReceive(USBD_Handle_t *pdev, uint8_t ep_addr,
                                          uint8_t *pbuf, uint32_t size);

#ifdef USBD_HS_TESTMODE_ENABLE
USBD_StatusTypeDef USBD_LL_SetTestMode(USBD_Handle_t *pdev, uint8_t testmode);
#endif /* USBD_HS_TESTMODE_ENABLE */

uint8_t USBD_LL_IsStallEP(USBD_Handle_t *pdev, uint8_t ep_addr);
uint32_t USBD_LL_GetRxDataSize(USBD_Handle_t *pdev, uint8_t  ep_addr);

void  USBD_LL_Delay(uint32_t Delay);

void *USBD_GetEpDesc(uint8_t *pConfDesc, uint8_t EpAddr);
USBD_DescHeaderTypeDef *USBD_GetNextDesc(uint8_t *pbuf, uint16_t *ptr);

/* usbd_core.h END ---- ---- ---- ---- ---- ---- ---- ---- */

/* ---------------------------------------------------------------------------------------*/
/* Initializes the CDC media low layer over the FS USB IP */
int8_t CDC_Init_FS(void);

/* @brief  DeInitializes the CDC media low layer */
int8_t CDC_DeInit_FS(void);

/* @brief  Manage the CDC class requests */
int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);

/* @brief  Data received over USB OUT endpoint are sent over CDC interface through this function. */
int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len);

/* Data to send over USB IN endpoint are sent over CDC interface through this function. */
void usbd_transmit(uint8_t* Buf, uint16_t Len);

/* CDC_TransmitCplt_FS Data transmitted callback */
int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum);

/* usbd_cdc_if.h END ---- ---- ---- ---- ---- ---- ---- ---- */


/* usb_device.h BEGIN ---- ---- ---- ---- ---- ---- ---- ---- */

/* Init USB device Library, add supported class and start the library */
void usb_device_init(void);
/* usb_device.h END ---- ---- ---- ---- ---- ---- ---- ---- */

/* usbd_cdc.h START ---- ---- ---- ---- ---- ---- ---- ---- */
uint8_t USBD_CDC_Init(USBD_Handle_t *pdev, uint8_t cfgidx);
uint8_t USBD_CDC_DeInit(USBD_Handle_t *pdev, uint8_t cfgidx);
uint8_t USBD_CDC_Setup(USBD_Handle_t *pdev, USBD_SetupReqTypedef *req);
uint8_t USBD_CDC_DataIn(USBD_Handle_t *pdev, uint8_t epnum);
uint8_t USBD_CDC_DataOut(USBD_Handle_t *pdev, uint8_t epnum);
uint8_t USBD_CDC_EP0_RxReady(USBD_Handle_t *pdev);
uint8_t *USBD_CDC_GetFSCfgDesc(uint16_t *length);
uint8_t *USBD_CDC_GetHSCfgDesc(uint16_t *length);
uint8_t *USBD_CDC_GetOtherSpeedCfgDesc(uint16_t *length);
uint8_t *USBD_CDC_GetOtherSpeedCfgDesc(uint16_t *length);
uint8_t *USBD_CDC_GetDeviceQualifierDescriptor(uint16_t *length);

/**
  * @brief  Initialize the CDC interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
uint8_t USBD_CDC_Init(USBD_Handle_t *pdev, uint8_t cfgidx);

/**
  * @brief  USBD_CDC_Init
  *         DeInitialize the CDC layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
uint8_t USBD_CDC_DeInit(USBD_Handle_t *pdev, uint8_t cfgidx);

/**
  * @brief  USBD_CDC_Setup
  *         Handle the CDC specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
uint8_t USBD_CDC_Setup(USBD_Handle_t *pdev,
                              USBD_SetupReqTypedef *req);

/**
  * @brief  USBD_CDC_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
uint8_t USBD_CDC_DataIn(USBD_Handle_t *pdev, uint8_t epnum);

/**
  * @brief  USBD_CDC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
uint8_t USBD_CDC_DataOut(USBD_Handle_t *pdev, uint8_t epnum);

/**
  * @brief  USBD_CDC_EP0_RxReady
  *         Handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
uint8_t USBD_CDC_EP0_RxReady(USBD_Handle_t *pdev);

/**
  * @brief  Get configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
uint8_t *USBD_CDC_GetFSCfgDesc(uint16_t *length);

/**
  * @brief  USBD_CDC_GetHSCfgDesc
  *         Return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
uint8_t *USBD_CDC_GetHSCfgDesc(uint16_t *length);

/**
  * @brief  USBD_CDC_GetOtherSpeedCfgDesc
  *         Return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
uint8_t *USBD_CDC_GetOtherSpeedCfgDesc(uint16_t *length);

/**
  * @brief  USBD_CDC_GetDeviceQualifierDescriptor
  *         return Device Qualifier descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
uint8_t *USBD_CDC_GetDeviceQualifierDescriptor(uint16_t *length);

/**
  * @brief  USBD_CDC_RegisterInterface
  * @param  pdev: device instance
  * @param  fops: CD  Interface callback
  * @retval status
  */
uint8_t USBD_CDC_RegisterInterface(USBD_Handle_t *pdev,
                                   USBD_CDC_ItfTypeDef *fops);

/**
  * @brief  USBD_CDC_SetTxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Tx Buffer
  * @param  length: Tx Buffer length
  * @retval status
  */
uint8_t USBD_CDC_SetTxBuffer(USBD_Handle_t *pdev,
                             uint8_t *pbuff, uint32_t length);

/**
  * @brief  USBD_CDC_SetRxBuffer
  */
uint8_t USBD_CDC_SetRxBuffer(USBD_Handle_t *pdev, uint8_t *pbuff);

/**
  */
uint8_t USBD_CDC_TransmitPacket(USBD_Handle_t *pdev);

/**
  * @brief  USBD_CDC_ReceivePacket
  *         prepare OUT Endpoint for reception
  * @param  pdev: device instance
  * @retval status
  */
uint8_t USBD_CDC_ReceivePacket(USBD_Handle_t *pdev);

/* usbd_cdc.h END ---- ---- ---- ---- ---- ---- ---- ---- */


/* usbd_core.h BEGIN ---- ---- ---- ---- ---- ---- ---- ---- */

/**
  * @brief  Initializes the device stack and load the class driver
  * @param  pdev: device instance
  * @param  pdesc: Descriptor structure address
  * @param  id: Low level core index
  * @retval None
  */
USBD_StatusTypeDef USBD_Init(USBD_Handle_t *pdev,
                             USBD_DescriptorsTypeDef *pdesc, uint8_t id);

/**
  * @brief  USBD_DeInit
  *         Re-Initialize the device library
  * @param  pdev: device instance
  * @retval status: status
  */
USBD_StatusTypeDef USBD_DeInit(USBD_Handle_t *pdev);

/**
  * @brief  USBD_RegisterClass
  *         Link class driver to Device Core.
  * @param  pDevice : Device Handle
  * @param  pclass: Class handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_RegisterClass(USBD_Handle_t *pdev, USBD_ClassTypeDef *pclass);


/**
  * @brief  USBD_Start
  *         Start the USB Device Core.
  * @param  pdev: Device Handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_Start(USBD_Handle_t *pdev);


/**
  * @brief  USBD_Stop
  *         Stop the USB Device Core.
  * @param  pdev: Device Handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_Stop(USBD_Handle_t *pdev);



/**
  * @brief  USBD_ClrClassConfig
  *         Clear current configuration
  * @param  pdev: device instance
  * @param  cfgidx: configuration index
  * @retval status: USBD_StatusTypeDef
  */
USBD_StatusTypeDef USBD_ClrClassConfig(USBD_Handle_t *pdev, uint8_t cfgidx);



/**
  * @brief  USBD_LL_DataOutStage
  *         Handle data OUT stage
  */
USBD_StatusTypeDef USBD_LL_DataOutStage(USBD_Handle_t *pdev,
                                        uint8_t epnum, uint8_t *pdata);

/**
  * @brief  USBD_LL_DataInStage
  *         Handle data in stage
  */
void USBD_LL_DataInStage(USBD_Handle_t *pdev,
                                       uint8_t epnum, uint8_t *pdata);

/**
  * @brief  USBD_LL_Reset
  *         Handle Reset event
  */

USBD_StatusTypeDef USBD_LL_Reset(USBD_Handle_t *pdev);

/**
  * @brief  USBD_LL_SetSpeed
  *         Handle Reset event
  * @param  pdev: device instance
  * @retval None.
  */
void USBD_LL_SetSpeed(USBD_Handle_t *pdev,
                                    USBD_SpeedTypeDef speed);

/**
  * @brief  Handle Suspend event
  * @param  pdev: device instance
  * @retval status
  */

USBD_StatusTypeDef USBD_LL_Suspend(USBD_Handle_t *pdev);

/**
  * @brief  USBD_LL_Resume
  *         Handle Resume event
  * @param  pdev: device instance
  * @retval status
  */

USBD_StatusTypeDef USBD_LL_Resume(USBD_Handle_t *pdev);

/**
  * @brief  USBD_LL_SOF
  *         Handle SOF event
  * @param  pdev: device instance
  * @retval status
  */

USBD_StatusTypeDef USBD_LL_SOF(USBD_Handle_t *pdev);

/**
  * @brief  USBD_LL_IsoINIncomplete
  *         Handle iso in incomplete event
  * @param  pdev: device instance
  * @retval status
  */
USBD_StatusTypeDef USBD_LL_IsoINIncomplete(USBD_Handle_t *pdev,
                                           uint8_t epnum);

/**
  * @brief  Handle iso out incomplete event.
  * @param  pdev: device instance
  * @retval None.
  */
void USBD_LL_IsoOUTIncomplete(USBD_Handle_t *pdev,
                                            uint8_t epnum);

/**
  * @brief  USBD_LL_DevConnected
  *         Handle device connection event
  * @param  pdev: device instance
  * @retval status
  */
USBD_StatusTypeDef USBD_LL_DevConnected(USBD_Handle_t *pdev);

/**
  * @brief  USBD_LL_DevDisconnected
  *         Handle device disconnection event
  * @param  pdev: device instance
  * @retval status
  */
USBD_StatusTypeDef USBD_LL_DevDisconnected(USBD_Handle_t *pdev);

/**
  * @brief  USBD_CoreFindIF
  *         return the class index relative to the selected interface
  * @param  pdev: device instance
  * @param  index : selected interface number
  * @retval index of the class using the selected interface number. OxFF if no class found.
  */
uint8_t USBD_CoreFindIF(USBD_Handle_t *pdev, uint8_t index);

/**
  * @brief  Returns the class index relative to the selected endpoint
  * @param  pdev: device instance
  * @param  index : selected endpoint number
  * @retval index of the class using the selected endpoint number. 0xFF if no class found.
  */
uint8_t USBD_CoreFindEP(USBD_Handle_t *pdev, uint8_t index);

/**
  * @brief  Returns the Endpoint descriptor
  * @param  pdev: device instance
  * @param  pConfDesc:  pointer to Bos descriptor
  * @param  EpAddr:  endpoint address
  * @retval pointer to video endpoint descriptor
  */
void *USBD_GetEpDesc(uint8_t *pConfDesc, uint8_t EpAddr);

/**
  * @brief  Returns the next descriptor header
  * @param  buf: Buffer where the descriptor is available
  * @param  ptr: data pointer inside the descriptor
  * @retval next header
  */
USBD_DescHeaderTypeDef *USBD_GetNextDesc(uint8_t *pbuf, uint16_t *ptr);

/* usbd_core.h END ---- ---- ---- ---- ---- ---- ---- ---- */


/* usbd_ctlreq.h BEGIN ---- ---- ---- ---- ---- ---- ---- ---- */
/* Includes ------------------------------------------------------------------*/

USBD_StatusTypeDef USBD_StdDevReq(USBD_Handle_t *pdev, USBD_SetupReqTypedef *req);
USBD_StatusTypeDef USBD_StdItfReq(USBD_Handle_t *pdev, USBD_SetupReqTypedef *req);
USBD_StatusTypeDef USBD_StdEPReq(USBD_Handle_t *pdev, USBD_SetupReqTypedef *req);

void USBD_CtlError(USBD_Handle_t *pdev, USBD_SetupReqTypedef *req);
void USBD_ParseSetupRequest(USBD_SetupReqTypedef *req, uint8_t *pdata);
void USBD_GetString(uint8_t *desc, uint8_t *unicode, uint16_t *len);


void USBD_GetDescriptor(USBD_Handle_t *pdev, USBD_SetupReqTypedef *req);
void USBD_SetAddress(USBD_Handle_t *pdev, USBD_SetupReqTypedef *req);
USBD_StatusTypeDef USBD_SetConfig(USBD_Handle_t *pdev, USBD_SetupReqTypedef *req);
void USBD_GetConfig(USBD_Handle_t *pdev, USBD_SetupReqTypedef *req);
void USBD_GetStatus(USBD_Handle_t *pdev, USBD_SetupReqTypedef *req);
void USBD_SetFeature(USBD_Handle_t *pdev, USBD_SetupReqTypedef *req);
void USBD_ClrFeature(USBD_Handle_t *pdev, USBD_SetupReqTypedef *req);
uint8_t USBD_GetLen(uint8_t *buf);
/* usbd_ctlreq.h END ---- ---- ---- ---- ---- ---- ---- ---- */


/* usbd_ioreq.h BEGIN ---- ---- ---- ---- ---- ---- ---- ---- */
void USBD_CtlSendData(USBD_Handle_t *pdev,
                                    uint8_t *pbuf, uint32_t len);

void USBD_CtlContinueSendData(USBD_Handle_t *pdev,
                                            uint8_t *pbuf, uint32_t len);

void USBD_CtlPrepareRx(USBD_Handle_t *pdev,
                                     uint8_t *pbuf, uint32_t len);

USBD_StatusTypeDef USBD_CtlContinueRx(USBD_Handle_t *pdev,
                                      uint8_t *pbuf, uint32_t len);

void USBD_CtlSendStatus(USBD_Handle_t *pdev);
USBD_StatusTypeDef USBD_CtlReceiveStatus(USBD_Handle_t *pdev);

uint32_t USBD_GetRxCount(USBD_Handle_t *pdev, uint8_t ep_addr);
/* usbd_ioreq.h END ---- ---- ---- ---- ---- ---- ---- ---- */



/* usbd_conf.h BEGIN ---- ---- ---- ---- ---- ---- ---- ---- */
#define USBD_MAX_NUM_INTERFACES     1U
#define USBD_MAX_NUM_CONFIGURATION     1U
#define USBD_MAX_STR_DESC_SIZ     512U
#define USBD_DEBUG_LEVEL     0U
#define USBD_LPM_ENABLED     0U
#define USBD_SELF_POWERED     1U

/****************************************/
/* #define for FS and HS identification */
#define DEVICE_FS 		0
#define DEVICE_HS 		1

/* Memory management macros make sure to use static memory allocation */
/** Alias for memory allocation. */

#define USBD_malloc         (void *)USBD_static_malloc

/** Alias for memory copy. */
#define USBD_memcpy         memcpy

/** Alias for delay. */
#define USBD_Delay          HAL_Delay

/* DEBUG macros */

#if (USBD_DEBUG_LEVEL > 0)
#define USBD_UsrLog(...)    printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBD_UsrLog(...)
#endif /* (USBD_DEBUG_LEVEL > 0U) */

#if (USBD_DEBUG_LEVEL > 1)

#define USBD_ErrLog(...)    printf("ERROR: ") ;\
                            printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBD_ErrLog(...)
#endif /* (USBD_DEBUG_LEVEL > 1U) */

#if (USBD_DEBUG_LEVEL > 2)
#define USBD_DbgLog(...)    printf("DEBUG : ") ;\
                            printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBD_DbgLog(...)
#endif /* (USBD_DEBUG_LEVEL > 2U) */


void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *p);

/* usbd_conf.h END ---- ---- ---- ---- ---- ---- ---- ---- */



/* usbd_desc.h BEGIN ---- ---- ---- ---- ---- ---- ---- ---- */
#define         DEVICE_ID1          (UID_BASE)
#define         DEVICE_ID2          (UID_BASE + 0x4)
#define         DEVICE_ID3          (UID_BASE + 0x8)

#define  USB_SIZ_STRING_SERIAL       0x1A

/** Descriptor for the Usb device. */
extern USBD_DescriptorsTypeDef FS_Desc;


/* usbd_desc.h END ---- ---- ---- ---- ---- ---- ---- ---- */


void Get_SerialNum(void);
void IntToUnicode(uint32_t value, uint8_t * pbuf, uint8_t len);

uint8_t * USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);


/* Start of usbd_desc.h */
#define         DEVICE_ID1          (UID_BASE)
#define         DEVICE_ID2          (UID_BASE + 0x4)
#define         DEVICE_ID3          (UID_BASE + 0x8)

#define  USB_SIZ_STRING_SERIAL       0x1A

extern USBD_DescriptorsTypeDef FS_Desc;

/* End of usbd_desc.h */
#endif /* __USB_CDC_DEVICE_H */








