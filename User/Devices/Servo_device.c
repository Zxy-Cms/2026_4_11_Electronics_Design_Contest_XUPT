/* Includes ------------------------------------------------------------------*/
#include "Servo_device.h"
#include "bsp_servo.h"

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  检查舵机定时器编号是否有效。
  * @param  servo_tim: 舵机定时器编号。
  * @retval 有效返回 1，无效返回 0。
  */
static uint8_t Servo_IsValidTim(Servo_TimTypeDef servo_tim)
{
  if ((uint32_t)servo_tim >= (uint32_t)SERVO_TIM_MAX)
  {
    return 0U;
  }

  return 1U;
}

/**
  * @brief  初始化指定舵机定时器对应的舵机。
  * @param  servo_tim: 舵机定时器编号，例如 SERVO_TIM14。
  * @retval BSP_OK 表示成功，其余表示失败。
  */
BSP_StatusTypeDef Servo_Init(Servo_TimTypeDef servo_tim)
{
  if (Servo_IsValidTim(servo_tim) == 0U)
  {
    return BSP_ERROR;
  }

  return BSP_Servo_Init(servo_tim, SERVO_MID_PULSE_US);
}

/**
  * @brief  按脉宽设置指定舵机定时器对应的舵机。
  * @param  servo_tim: 舵机定时器编号，例如 SERVO_TIM14。
  * @param  pulse_us: 目标脉宽，单位为 us。
  * @retval BSP_OK 表示成功，其余表示失败。
  */
BSP_StatusTypeDef Servo_SetPulseUs(Servo_TimTypeDef servo_tim, uint16_t pulse_us)
{
  if (Servo_IsValidTim(servo_tim) == 0U)
  {
    return BSP_ERROR;
  }

  if (pulse_us < SERVO_MIN_PULSE_US)
  {
    pulse_us = SERVO_MIN_PULSE_US;
  }

  if (pulse_us > SERVO_MAX_PULSE_US)
  {
    pulse_us = SERVO_MAX_PULSE_US;
  }

  /* compare 会由 Bsp 层写入指定舵机定时器的底层 PWM 输出。 */
  return BSP_Servo_SetCompare(servo_tim, pulse_us);
}

/**
  * @brief  按角度设置指定舵机定时器对应的舵机。
  * @param  servo_tim: 舵机定时器编号，例如 SERVO_TIM14。
  * @param  angle: 目标角度，范围 0~180。
  * @retval BSP_OK 表示成功，其余表示失败。
  */
BSP_StatusTypeDef Servo_SetAngle(Servo_TimTypeDef servo_tim, uint16_t angle)
{
  uint32_t pulse_us;

  if (Servo_IsValidTim(servo_tim) == 0U)
  {
    return BSP_ERROR;
  }

  if (angle > 180U)
  {
    angle = 180U;
  }

  /* 按 0 度=500us，180 度=2500us 线性换算。 */
  pulse_us = SERVO_MIN_PULSE_US +
             (((uint32_t)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * angle) / 180U);

  return Servo_SetPulseUs(servo_tim, (uint16_t)pulse_us);
}
