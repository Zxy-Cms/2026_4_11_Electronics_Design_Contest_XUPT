/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_servo.h
  * @brief          : 舵机底层驱动头文件
  ******************************************************************************
  * @attention
  *
  * 本模块用于封装舵机到底层 PWM 输出的映射关系。
  * 当前版本先只处理 TIM14 对应的一路舵机输出。
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __BSP_SERVO_H__
#define __BSP_SERVO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bsp_common.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  /* 当前这一路舵机使用 TIM14。 */
  SERVO_TIM14 = 0U,
  /* 当前这一路舵机使用 TIM13。 */
  SERVO_TIM13 = 1U,
  /* 当前这两路舵机使用 TIM2_CH1 / TIM2_CH2。 */
  SERVO_TIM2_1 = 2U,
  SERVO_TIM2_2 = 3U,
  /* 调试舵机单独使用 TIM12_CH1 / TIM12_CH2，与 TIM2 主控彻底解耦。 */
  SERVO_TIM12_1 = 4U,
  SERVO_TIM12_2 = 5U,
  /* 后续如果增加 TIM3、TIM4 等舵机，在这里继续往下加。 */
  SERVO_TIM_MAX
} Servo_TimTypeDef;

/* Exported functions --------------------------------------------------------*/
BSP_StatusTypeDef BSP_Servo_Init(Servo_TimTypeDef servo_tim, uint16_t compare);
BSP_StatusTypeDef BSP_Servo_SetCompare(Servo_TimTypeDef servo_tim, uint16_t compare);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SERVO_H__ */


