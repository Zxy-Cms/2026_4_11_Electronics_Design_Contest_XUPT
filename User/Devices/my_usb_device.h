/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : my_usb_device.h
  * @brief          : USB 设备收发接口头文件
  ******************************************************************************
  * @attention
  *
  * 本文件用于声明 Devices 层 USB CDC 收发接口。
  * 底层 USB 收发由 Bsp 层封装完成。
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MY_USB_DEVICE_H__
#define __MY_USB_DEVICE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bsp_common.h"

/* Exported functions --------------------------------------------------------*/
BSP_StatusTypeDef Usb_Device_Send(const uint8_t *p_data, uint16_t length);
BSP_StatusTypeDef Usb_Device_SendString(const char *p_string);
uint16_t Usb_Device_Read(uint8_t *p_data, uint16_t length);
uint16_t Usb_Device_GetRxCount(void);
uint8_t Usb_Device_HasRxOverflow(void);
void Usb_Device_ClearRxBuffer(void);
void Usb_Device_ClearRxOverflow(void);

#ifdef __cplusplus
}
#endif

#endif /* __MY_USB_DEVICE_H__ */
