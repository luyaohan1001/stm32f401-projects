Originally usbd_cdc_if.c (HAL middle ware) was merged into usb-cdc-device.c

usb_device.c/.h carries only 
    the MX_USB_DEVICE_Init(void) 
    USBD_HandleTypeDef hUsbDeviceFS;
	that's the one earliest merged into usb_cdc_device.c/.h


The study can begin by merging:
	usbd_cdc_if.c
  usb_device.c
	usb_cdc.c

# USB Classes
| Base | Class      | Descriptor Usage Description                       |
|------|------------|----------------------------------------------------|
| 00h  | Device     | Use class information in the Interface Descriptors |
| 01h  | Interface  | Audio                                              |
| 02h  | Both       | Communications and CDC Control                     |
| 03h  | Interface  | HID (Human Interface Device)                       |
| 05h  | Interface  | Physical                                           |
| 06h  | Interface  | Image                                              |
| 07h  | Interface  | Printer                                            |
| 08h  | Interface  | Mass Storage                                       |
| 09h  | Device     | Hub                                                |
| 0Ah  | Interface  | CDC-Data                                           |
| 0Bh  | Interface  | Smart Card                                         |
| 0Dh  | Interface  | Content Security                                   |
| 0Eh  | Interface  | Video                                              |
| 0Fh  | Interface  | Personal Healthcare                                |
| 10h  | Interface  | Audio/Video Devices                                |
| 11h  | Device     | Billboard Device Class                             |
| 12h  | Interface  | USB Type-C Bridge Class                            |
| 3Ch	 | Interface	| I3C Device Class                                   |
| DCh  | Both       | Diagnostic Device                                  |
| E0h  | Interface  | Wireless Controller                                |
| EFh  | Both       | Miscellaneous                                      |
| FEh  | Interface  | Application Specific                               |
| FFh  | Both       | Vendor Specific                                    |





