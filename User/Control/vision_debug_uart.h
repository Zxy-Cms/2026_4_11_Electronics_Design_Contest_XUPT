#ifndef __VISION_DEBUG_UART_H__
#define __VISION_DEBUG_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_common.h"

/**
  * @brief  视觉调试串口输出配置结构体。
  * @param  enable: 调试任务总开关，1 表示允许输出，0 表示关闭输出。
  * @param  period_ms: 调试串口发送周期，单位为毫秒。
  * @retval None
  */
typedef struct
{
  uint8_t enable;
  uint32_t period_ms;
} VisionDebugUart_ConfigTypeDef;

/**
  * @brief  初始化视觉调试串口任务。
  * @param  None
  * @retval BSP_OK 表示初始化成功，BSP_ERROR 表示 USART6 设备层初始化失败。
  */
BSP_StatusTypeDef VisionDebugUart_Init(void);

/**
  * @brief  获取视觉调试串口任务配置结构体，便于在线调整发送周期或总开关。
  * @param  None
  * @retval 返回全局配置结构体指针。
  */
VisionDebugUart_ConfigTypeDef *VisionDebugUart_GetConfig(void);

/**
  * @brief  视觉调试串口周期任务。
  * @param  None
  * @retval None
  */
void VisionDebugUart_Task(void);

/**
  * @brief  速度环调试串口周期任务。
  * @param  None
  * @retval None
  */
void VisionDebugUart_VelocityTask(void);

#ifdef __cplusplus
}
#endif

#endif /* __VISION_DEBUG_UART_H__ */


