#ifndef __VISION_AXIS_H__
#define __VISION_AXIS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_common.h"
#include "bsp_servo.h"
#include "pid.h"

/* 单轴控制配置.
   外环位置 PID 输出目标速度.
   内环速度 PID 输出舵机脉宽增量. */
typedef struct
{
  Servo_TimTypeDef servo_tim;
  float servo_center_us;
  float servo_min_us;
  float servo_max_us;
  float servo_direction;
  float velocity_filter_alpha;
  PID_ParamsTypeDef position_pid;
  PID_ParamsTypeDef velocity_pid;
} VisionAxis_ConfigTypeDef;

/* 单轴运行状态, 便于调试和后续扩展. */
typedef struct
{
  float target_position;
  float current_position;
  float measured_velocity;
  float target_velocity;
  float servo_output_us;
  float servo_pulse_us;
} VisionAxis_StateTypeDef;

/* 单轴控制对象. */
typedef struct
{
  VisionAxis_ConfigTypeDef config;
  PID_TypeDef position_pid;
  PID_TypeDef velocity_pid;
  VisionAxis_StateTypeDef state;
  float last_position;
  uint8_t measurement_ready;
} VisionAxis_TypeDef;

/* 初始化单轴控制对象和舵机输出. */
BSP_StatusTypeDef VisionAxis_Init(VisionAxis_TypeDef *p_axis,
                                  const VisionAxis_ConfigTypeDef *p_config);

/* 控制异常或回中时重置单轴状态. */
void VisionAxis_Reset(VisionAxis_TypeDef *p_axis);

/* 用位置差分估算当前速度. */
void VisionAxis_UpdateMeasurement(VisionAxis_TypeDef *p_axis,
                                  float current_position,
                                  float dt_s);

/* 执行一轮双环控制计算. */
BSP_StatusTypeDef VisionAxis_Run(VisionAxis_TypeDef *p_axis,
                                 float target_position,
                                 float current_position,
                                 float dt_s);

/* 读取当前单轴状态. */
const VisionAxis_StateTypeDef *VisionAxis_GetState(const VisionAxis_TypeDef *p_axis);

#ifdef __cplusplus
}
#endif

#endif /* __VISION_AXIS_H__ */
