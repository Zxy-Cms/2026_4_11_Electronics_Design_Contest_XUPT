#ifndef __GYRO_DEBUG_SERVO_H__
#define __GYRO_DEBUG_SERVO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_common.h"
#include "bsp_servo.h"

typedef struct
{
  Servo_TimTypeDef roll_servo_tim;
  Servo_TimTypeDef pitch_servo_tim;
  float input_min_deg;
  float input_max_deg;
  uint16_t servo_min_deg;
  uint16_t servo_mid_deg;
  uint16_t servo_max_deg;
} GyroDebugServo_ConfigTypeDef;

typedef struct
{
  float roll_input_deg;
  float pitch_input_deg;
  uint16_t roll_servo_angle_deg;
  uint16_t pitch_servo_angle_deg;
} GyroDebugServo_StateTypeDef;

/**
  * @brief  获取陀螺仪调试舵机配置结构体。
  * @param  None
  * @retval 返回全局配置结构体指针，可用于在线修改映射范围和输出通道。
  */
GyroDebugServo_ConfigTypeDef *GyroDebugServo_GetConfig(void);

/**
  * @brief  初始化陀螺仪调试用的双舵机输出。
  * @param  None
  * @retval BSP_OK 表示初始化成功，BSP_ERROR 表示任意一路舵机初始化失败。
  */
BSP_StatusTypeDef GyroDebugServo_Init(void);

/**
  * @brief  根据陀螺仪横滚角和俯仰角更新两路调试舵机角度。
  * @param  roll_input_deg: 横滚角输入，单位为度。
  * @param  pitch_input_deg: 俯仰角输入，单位为度。
  * @retval None
  */
void GyroDebugServo_Update(float roll_input_deg, float pitch_input_deg);

/**
  * @brief  获取当前双路调试舵机状态。
  * @param  None
  * @retval 返回状态结构体常量指针，便于上层查看输入角和输出角。
  */
const GyroDebugServo_StateTypeDef *GyroDebugServo_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* __GYRO_DEBUG_SERVO_H__ */
