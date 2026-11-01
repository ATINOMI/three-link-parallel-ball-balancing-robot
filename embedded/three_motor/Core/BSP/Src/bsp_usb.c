#include "bsp_usb.h"

#include "usbd_cdc_if.h"
extern USBD_HandleTypeDef hUsbDeviceFS;

uint8_t BSP_USB_CanSend(void) {
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED ||
      hUsbDeviceFS.pClassData == NULL) {
    return 0U;
  }
  USBD_CDC_HandleTypeDef *cdc = hUsbDeviceFS.pClassData;
  return cdc->TxState == 0U;
}
uint8_t BSP_USB_Send(const uint8_t *data, uint16_t length) {
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED ||
      hUsbDeviceFS.pClassData == NULL) {
    return USBD_FAIL;
  }
  return CDC_Transmit_FS((uint8_t *)data, length);
}

__weak void BSP_USB_ReceiveCallback(const uint8_t *data, uint32_t length) {
  (void)data;
  (void)length;
}
