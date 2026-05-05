/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Servo_device.h
  * @brief          : 舵机设备层头文件
  ******************************************************************************
  * @attention
  *
  * Devices 层对外使用可读性更强的定时器名字，
  * 例如 SERVO_TIM14，不暴露底层 HAL 句柄。
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __SERVO_DEVICE_H__
#define __SERVO_DEVICE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bsp_servo.h"

/* Exported constants --------------------------------------------------------*/
/* 舵机测试范围按 0.5ms~2.5ms 使用。 */
#define SERVO_MIN_PULSE_US       500U
#define SERVO_MID_PULSE_US      1500U
#define SERVO_MAX_PULSE_US      2500U

/* Exported functions --------------------------------------------------------*/
BSP_StatusTypeDef Servo_Init(Servo_TimTypeDef servo_tim);
BSP_StatusTypeDef Servo_SetPulseUs(Servo_TimTypeDef servo_tim, uint16_t pulse_us);
BSP_StatusTypeDef Servo_SetAngle(Servo_TimTypeDef servo_tim, uint16_t angle);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_DEVICE_H__ */
