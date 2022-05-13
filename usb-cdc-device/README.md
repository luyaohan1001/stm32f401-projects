Originally usbd_cdc_if.c (HAL middle ware) was merged into usb-cdc-device.c

usb_device.c/.h carries only 
    the MX_USB_DEVICE_Init(void) 
    USBD_HandleTypeDef hUsbDeviceFS;
	that's the one earliest merged into usb_cdc_device.c/.h


The study can begin by merging:
	usbd_cdc_if.c
  usb_device.c
	usb_cdc.c





