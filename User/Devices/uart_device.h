/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : uart_device.h
  * @brief          : UART 璁惧灞傚ご鏂囦欢
  ******************************************************************************
  * @attention
  *
  * 璁惧灞傛妸鍙鎬ф洿寮虹殑绔彛缂栧彿鏄犲皠鍒?Bsp 灞?UART 璧勬簮銆?  * 涓氬姟浠ｇ爜搴旈€氳繃鏈眰璁块棶涓插彛锛岃€屼笉鏄洿鎺ユ搷浣?HAL 鍙ユ焺銆?  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __UART_DEVICE_H__
#define __UART_DEVICE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bsp_common.h"

/* Exported constants --------------------------------------------------------*/
#define UART_DEVICE_WAIT_FOREVER  0xFFFFFFFFU

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  UART_DEVICE_USART1 = 0U,
  UART_DEVICE_USART6 = 1U,
  UART_DEVICE_UART4 = 2U,
  UART_DEVICE_MAX
} Uart_DevicePortTypeDef;

/* Exported functions --------------------------------------------------------*/
BSP_StatusTypeDef Uart_Device_Init(Uart_DevicePortTypeDef uart_port);
BSP_StatusTypeDef Uart_Device_SetTimeout(Uart_DevicePortTypeDef uart_port, uint32_t timeout_ms);
BSP_StatusTypeDef Uart_Device_Send(Uart_DevicePortTypeDef uart_port,
                                   const uint8_t *p_data,
                                   uint16_t length);
BSP_StatusTypeDef Uart_Device_SendString(Uart_DevicePortTypeDef uart_port, const char *p_string);
BSP_StatusTypeDef Uart_Device_Read(Uart_DevicePortTypeDef uart_port,
                                   uint8_t *p_data,
                                   uint16_t length);
BSP_StatusTypeDef Uart_Device_GetTxStatus(Uart_DevicePortTypeDef uart_port);
BSP_StatusTypeDef Uart_Device_GetRxStatus(Uart_DevicePortTypeDef uart_port);

/**
  * @brief  为指定串口启动“单字节中断接收 + 内部环形缓冲”功能。
  * @param  uart_port: 需要开启字节流接收的串口端口。
  *                   当前版本主要用于 UART4 视觉输入。
  * @retval BSP_OK 表示启动成功，BSP_ERROR 表示端口非法或底层中断接收启动失败。
  */
BSP_StatusTypeDef Uart_Device_StartByteReceiveIT(Uart_DevicePortTypeDef uart_port);

/**
  * @brief  从指定串口的内部字节队列中取出 1 个接收字节。
  * @param  uart_port: 需要读取的串口端口。
  * @param  p_data: 输出参数，返回取出的单字节数据。
  * @retval 1 表示成功取到 1 个字节，0 表示当前队列为空或参数无效。
  */
uint8_t Uart_Device_PopByte(Uart_DevicePortTypeDef uart_port, uint8_t *p_data);

/**
  * @brief  清空指定串口的内部字节接收队列。
  * @param  uart_port: 需要清空队列的串口端口。
  * @retval None
  */
void Uart_Device_ClearByteQueue(Uart_DevicePortTypeDef uart_port);

#ifdef __cplusplus
}
#endif

#endif /* __UART_DEVICE_H__ */
