/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_usb.h
  * @brief          : USB CDC 底层驱动头文件
  ******************************************************************************
  * @attention
  *
  * 本模块用于封装 USB CDC 底层收发与接收缓存。
  * Devices 层通过本模块访问 USB，不直接调用 HAL 或 USB 中间件接口。
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __BSP_USB_H__
#define __BSP_USB_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bsp_common.h"

/* Exported functions --------------------------------------------------------*/
BSP_StatusTypeDef BSP_USB_Send(const uint8_t *p_data, uint16_t length);
uint16_t BSP_USB_Read(uint8_t *p_data, uint16_t length);
uint16_t BSP_USB_GetRxCount(void);
uint8_t BSP_USB_HasRxOverflow(void);
void BSP_USB_ClearRxBuffer(void);
void BSP_USB_ClearRxOverflow(void);
void BSP_USB_RxCallback(uint8_t *p_data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_USB_H__ */
