
#ifndef __USB_CDC_DEVICE_H
#define __USB_CDC_DEVICE_H


/* usb_device.h BEGIN ---- ---- ---- ---- ---- ---- ---- ---- */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "usbd_def.h"

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
  USBD_ClassTypeDef       *pClass[USBD_MAX_SUPPORTED_CLASS];
  void                    *pClassData;
  void                    *pClassDataCmsit[USBD_MAX_SUPPORTED_CLASS];
  void                    *pUserData[USBD_MAX_SUPPORTED_CLASS];
  void                    *pData;
  void                    *pBosDesc;
  void                    *pConfDesc;
  uint32_t                classId;
  uint32_t                NumClasses;
};

/** USB Device initialization function. */
void usb_device_init(void); // equvilent to  void MX_USB_DEVICE_Init(void)
/* usb_device.h END ---- ---- ---- ---- ---- ---- ---- ---- */

/* usbd_cdc_if.h BEGIN---- ---- ---- ---- ---- ---- ---- ---- */
/* Includes ------------------------------------------------------------------*/
// #include "usbd_cdc.h"
/* Define size for the receive and transmit buffer over CDC */
#define APP_RX_DATA_SIZE  2048
#define APP_TX_DATA_SIZE  2048

/** CDC Interface callback. */
// extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
/* usbd_cdc_if.h END---- ---- ---- ---- ---- ---- ---- ---- */


/* usbd_cdc.h END ---- ---- ---- ---- ---- ---- ---- ---- */
/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

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
} USBD_CDC_HandleTypeDef;


extern USBD_ClassTypeDef USBD_CDC;
#define USBD_CDC_CLASS &USBD_CDC

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t USBD_CDC_RegisterInterface(USBD_HandleTypeDef *pdev,
                                   USBD_CDC_ItfTypeDef *fops);

uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff,
                             uint32_t length);

uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff);
uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *pdev);
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *pdev);

#endif /* __USB_CDC_DEVICE_H */
/* usbd_cdc.h END ---- ---- ---- ---- ---- ---- ---- ---- */



