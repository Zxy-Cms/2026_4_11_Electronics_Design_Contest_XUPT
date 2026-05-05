/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_uart.h
  * @brief          : UART 底层驱动头文件
  ******************************************************************************
  * @attention
  *
  * 本模块只负责管理 UART 硬件资源绑定、初始化以及阻塞式收发。
  * 上层模块不应直接访问 HAL 的 UART 句柄。
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bsp_common.h"
#include "usart.h"

/* Exported types ------------------------------------------------------------*/
typedef void (*BSP_UART_InitFunc)(void);

typedef struct
{
  /* HAL 层 UART 句柄，例如 &huart1、&huart2。 */
  UART_HandleTypeDef *huart;

  /* 对应的底层初始化函数，例如 MX_USART1_UART_Init。 */
  BSP_UART_InitFunc init;
} BSP_UART_ResourceTypeDef;

typedef struct
{
  /* 指向具体 UART 硬件资源描述结构体。 */
  const BSP_UART_ResourceTypeDef *p_resource;

  /* 阻塞式收发默认超时时间，单位 ms。 */
  uint32_t timeout_ms;
} BSP_UART_TypeDef;

/* Exported functions --------------------------------------------------------*/
BSP_StatusTypeDef BSP_UART_Init(const BSP_UART_TypeDef *p_uart);
BSP_StatusTypeDef BSP_UART_Transmit(const BSP_UART_TypeDef *p_uart,
                                    const uint8_t *p_data,
                                    uint16_t length);
BSP_StatusTypeDef BSP_UART_Receive(const BSP_UART_TypeDef *p_uart,
                                   uint8_t *p_data,
                                   uint16_t length);
BSP_StatusTypeDef BSP_UART_GetTxStatus(const BSP_UART_TypeDef *p_uart);
BSP_StatusTypeDef BSP_UART_GetRxStatus(const BSP_UART_TypeDef *p_uart);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_UART_H__ */
