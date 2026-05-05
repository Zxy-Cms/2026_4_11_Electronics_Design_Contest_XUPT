/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_pwm.h
  * @brief          : PWM 底层驱动头文件
  ******************************************************************************
  * @attention
  *
  * 本模块只负责执行 PWM 初始化、启动和比较值设置。
  * 具体使用哪个定时器、哪个通道、调用哪个初始化函数，由上层 Bsp
  * 通过资源结构体传入，这样后续增加新定时器时无需修改 bsp_pwm.c。
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __BSP_PWM_H__
#define __BSP_PWM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bsp_common.h"
#include "tim.h"

/* Exported types ------------------------------------------------------------*/
typedef void (*BSP_PWM_InitFunc)(void);

typedef struct
{
  /* HAL 定时器句柄，例如 &htim14、&htim3。 */
  TIM_HandleTypeDef *htim;

  /* 对应的底层初始化函数，例如 MX_TIM14_Init。 */
  BSP_PWM_InitFunc init;

  /* PWM 输出通道，例如 TIM_CHANNEL_1。 */
  uint32_t channel;
} BSP_PWM_ResourceTypeDef;

typedef struct
{
  /* 指向具体 PWM 硬件资源的描述结构体。 */
  const BSP_PWM_ResourceTypeDef *p_resource;

  /* 比较值，会直接写入 CCR 寄存器。 */
  uint16_t compare;
} BSP_PWM_TypeDef;

/* Exported functions --------------------------------------------------------*/
BSP_StatusTypeDef BSP_PWM_Init(const BSP_PWM_TypeDef *p_pwm);
BSP_StatusTypeDef BSP_PWM_Set(const BSP_PWM_TypeDef *p_pwm);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_PWM_H__ */
