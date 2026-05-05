/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_common.h
  * @brief          : Bsp 公共类型头文件
  ******************************************************************************
  * @attention
  *
  * 本文件用于定义 Bsp 层公共状态类型。
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __BSP_COMMON_H__
#define __BSP_COMMON_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  BSP_OK = 0U,
  BSP_ERROR = 1U,
  BSP_BUSY = 2U
} BSP_StatusTypeDef;

#ifdef __cplusplus
}
#endif

#endif /* __BSP_COMMON_H__ */
