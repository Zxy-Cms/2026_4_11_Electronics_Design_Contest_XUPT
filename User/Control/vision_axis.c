#include "vision_axis.h"

#include "Servo_device.h"

/* 通用限幅, 用于舵机脉宽保护. */
static float VisionAxis_Clamp(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }

  if (value > max_value)
  {
    return max_value;
  }

  return value;
}

/* 把浮点脉宽转换成舵机接口需要的整数脉宽. */
static uint16_t VisionAxis_FloatToPulse(float pulse_us)
{
  if (pulse_us < 0.0f)
  {
    pulse_us = 0.0f;
  }

  return (uint16_t)(pulse_us + 0.5f);
}

/* 统一完成单轴舵机输出和限位保护. */
static BSP_StatusTypeDef VisionAxis_WriteServo(VisionAxis_TypeDef *p_axis, float pulse_us)
{
  pulse_us = VisionAxis_Clamp(pulse_us,
                              p_axis->config.servo_min_us,
                              p_axis->config.servo_max_us);
  p_axis->state.servo_pulse_us = pulse_us;

  return Servo_SetPulseUs(p_axis->config.servo_tim, VisionAxis_FloatToPulse(pulse_us));
}

BSP_StatusTypeDef VisionAxis_Init(VisionAxis_TypeDef *p_axis,
                                  const VisionAxis_ConfigTypeDef *p_config)
{
  if ((p_axis == (VisionAxis_TypeDef *)0) ||
      (p_config == (const VisionAxis_ConfigTypeDef *)0))
  {
    return BSP_ERROR;
  }

  if ((p_config->servo_center_us < p_config->servo_min_us) ||
      (p_config->servo_center_us > p_config->servo_max_us))
  {
    return BSP_ERROR;
  }

  p_axis->config = *p_config;

  /* 滤波系数限制在 0 到 1 之间, 防止配置错误. */
  if (p_axis->config.velocity_filter_alpha < 0.0f)
  {
    p_axis->config.velocity_filter_alpha = 0.0f;
  }

  if (p_axis->config.velocity_filter_alpha > 1.0f)
  {
    p_axis->config.velocity_filter_alpha = 1.0f;
  }

  PID_Init(&p_axis->position_pid, &p_axis->config.position_pid);
  PID_Init(&p_axis->velocity_pid, &p_axis->config.velocity_pid);

  p_axis->last_position = 0.0f;
  p_axis->measurement_ready = 0U;
  p_axis->state.target_position = 0.0f;
  p_axis->state.current_position = 0.0f;
  p_axis->state.measured_velocity = 0.0f;
  p_axis->state.target_velocity = 0.0f;
  p_axis->state.servo_output_us = 0.0f;
  p_axis->state.servo_pulse_us = p_axis->config.servo_center_us;

  if (Servo_Init(p_axis->config.servo_tim) != BSP_OK)
  {
    return BSP_ERROR;
  }

  return VisionAxis_WriteServo(p_axis, p_axis->config.servo_center_us);
}

void VisionAxis_Reset(VisionAxis_TypeDef *p_axis)
{
  if (p_axis == (VisionAxis_TypeDef *)0)
  {
    return;
  }

  PID_Reset(&p_axis->position_pid);
  PID_Reset(&p_axis->velocity_pid);

  p_axis->measurement_ready = 0U;
  p_axis->last_position = 0.0f;
  p_axis->state.measured_velocity = 0.0f;
  p_axis->state.target_velocity = 0.0f;
  p_axis->state.servo_output_us = 0.0f;
  (void)VisionAxis_WriteServo(p_axis, p_axis->config.servo_center_us);
}

void VisionAxis_UpdateMeasurement(VisionAxis_TypeDef *p_axis,
                                  float current_position,
                                  float dt_s)
{
  float raw_velocity;

  if (p_axis == (VisionAxis_TypeDef *)0)
  {
    return;
  }

  p_axis->state.current_position = current_position;

  /* 第一帧只建立位置基准, 不直接算速度. */
  if (p_axis->measurement_ready == 0U)
  {
    p_axis->last_position = current_position;
    p_axis->state.measured_velocity = 0.0f;
    p_axis->measurement_ready = 1U;
    return;
  }

  if (dt_s <= 0.0f)
  {
    p_axis->last_position = current_position;
    return;
  }

  raw_velocity = (current_position - p_axis->last_position) / dt_s;
  p_axis->last_position = current_position;

  if (p_axis->config.velocity_filter_alpha <= 0.0f)
  {
    p_axis->state.measured_velocity = raw_velocity;
    return;
  }

  /* 简单一阶滤波, 抑制视觉测量抖动. */
  p_axis->state.measured_velocity +=
    p_axis->config.velocity_filter_alpha * (raw_velocity - p_axis->state.measured_velocity);
}

BSP_StatusTypeDef VisionAxis_Run(VisionAxis_TypeDef *p_axis,
                                 float target_position,
                                 float current_position,
                                 float dt_s)
{
  float servo_delta_us;
  float servo_pulse_us;

  if ((p_axis == (VisionAxis_TypeDef *)0) || (dt_s <= 0.0f))
  {
    return BSP_ERROR;
  }

  p_axis->state.target_position = target_position;
  p_axis->state.current_position = current_position;

  /* 外环根据位置误差给出目标速度. */
  p_axis->state.target_velocity =
    PID_Update(&p_axis->position_pid, target_position, current_position, dt_s);

  /* 内环根据速度误差给出最终舵机脉宽增量. */
  servo_delta_us =
    PID_Update(&p_axis->velocity_pid,
               p_axis->state.target_velocity,
               p_axis->state.measured_velocity,
               dt_s);

  p_axis->state.servo_output_us = servo_delta_us;

  servo_pulse_us = p_axis->config.servo_center_us +
                   (p_axis->config.servo_direction * servo_delta_us);

  return VisionAxis_WriteServo(p_axis, servo_pulse_us);
}

const VisionAxis_StateTypeDef *VisionAxis_GetState(const VisionAxis_TypeDef *p_axis)
{
  if (p_axis == (const VisionAxis_TypeDef *)0)
  {
    return (const VisionAxis_StateTypeDef *)0;
  }

  return &p_axis->state;
}
