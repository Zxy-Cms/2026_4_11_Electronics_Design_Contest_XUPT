#include "gyro_debug_servo.h"

#include "Servo_device.h"

/* 该模块只用于陀螺仪调试任务。
   默认单独使用 TIM12 的两路调试舵机输出:
   1. TIM12_CH1 对应 roll
   2. TIM12_CH2 对应 pitch
   这样可以确保调试任务不会覆盖 TIM2 的正式双轴闭环输出。
   输入角度默认按 -60°~60° 线性映射到 90°~130°。 */
static GyroDebugServo_ConfigTypeDef g_gyro_debug_servo_config =
{
  SERVO_TIM12_1,
  SERVO_TIM12_2,
  -60.0f,
  60.0f,
  90U,
  110U,
  130U
};

/* 调试状态同时保存原始输入角和两路舵机输出角，
   便于后续串口打印、屏幕显示或在线观察。 */
static GyroDebugServo_StateTypeDef g_gyro_debug_servo_state =
{
  0.0f,
  0.0f,
  110U,
  110U
};

/**
  * @brief  对浮点输入做上下限约束，避免超出配置范围。
  * @param  value: 待限幅的输入值。
  * @param  min_value: 允许的最小值。
  * @param  max_value: 允许的最大值。
  * @retval 返回限幅后的结果。
  */
static float GyroDebugServo_ClampFloat(float value, float min_value, float max_value)
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

/**
  * @brief  把单个姿态角按配置线性映射成舵机角度。
  * @param  input_angle_deg: 输入姿态角，单位为度。
  * @retval 返回映射后的舵机角度，结果会被限制在 servo_min_deg~servo_max_deg。
  */
static uint16_t GyroDebugServo_MapInputToServo(float input_angle_deg)
{
  float input_range;
  float output_range;
  float ratio;
  uint16_t servo_angle;

  input_angle_deg = GyroDebugServo_ClampFloat(input_angle_deg,
                                              g_gyro_debug_servo_config.input_min_deg,
                                              g_gyro_debug_servo_config.input_max_deg);

  input_range = g_gyro_debug_servo_config.input_max_deg -
                g_gyro_debug_servo_config.input_min_deg;

  if (input_range <= 0.0f)
  {
    servo_angle = g_gyro_debug_servo_config.servo_mid_deg;
  }
  else
  {
    output_range = (float)((int32_t)g_gyro_debug_servo_config.servo_max_deg -
                           (int32_t)g_gyro_debug_servo_config.servo_min_deg);
    ratio = (input_angle_deg - g_gyro_debug_servo_config.input_min_deg) / input_range;
    servo_angle = (uint16_t)((float)g_gyro_debug_servo_config.servo_min_deg +
                             (ratio * output_range) + 0.5f);
  }

  if (servo_angle < g_gyro_debug_servo_config.servo_min_deg)
  {
    servo_angle = g_gyro_debug_servo_config.servo_min_deg;
  }

  if (servo_angle > g_gyro_debug_servo_config.servo_max_deg)
  {
    servo_angle = g_gyro_debug_servo_config.servo_max_deg;
  }

  return servo_angle;
}

/**
  * @brief  获取陀螺仪调试舵机配置结构体。
  * @param  None
  * @retval 返回全局配置结构体指针，可用于在线修改输入范围和输出通道。
  */
GyroDebugServo_ConfigTypeDef *GyroDebugServo_GetConfig(void)
{
  return &g_gyro_debug_servo_config;
}

/**
  * @brief  初始化陀螺仪调试用的双舵机输出，并将两路舵机同时回到中位。
  * @param  None
  * @retval BSP_OK 表示初始化成功，BSP_ERROR 表示任意一路舵机初始化失败。
  */
BSP_StatusTypeDef GyroDebugServo_Init(void)
{
  g_gyro_debug_servo_state.roll_input_deg = 0.0f;
  g_gyro_debug_servo_state.pitch_input_deg = 0.0f;
  g_gyro_debug_servo_state.roll_servo_angle_deg = g_gyro_debug_servo_config.servo_mid_deg;
  g_gyro_debug_servo_state.pitch_servo_angle_deg = g_gyro_debug_servo_config.servo_mid_deg;

  if (Servo_Init(g_gyro_debug_servo_config.roll_servo_tim) != BSP_OK)
  {
    return BSP_ERROR;
  }

  if (Servo_Init(g_gyro_debug_servo_config.pitch_servo_tim) != BSP_OK)
  {
    return BSP_ERROR;
  }

  if (Servo_SetAngle(g_gyro_debug_servo_config.roll_servo_tim,
                     g_gyro_debug_servo_config.servo_mid_deg) != BSP_OK)
  {
    return BSP_ERROR;
  }

  return Servo_SetAngle(g_gyro_debug_servo_config.pitch_servo_tim,
                        g_gyro_debug_servo_config.servo_mid_deg);
}

/**
  * @brief  根据陀螺仪 roll/pitch 角更新 TIM12 的两路调试舵机。
  * @param  roll_input_deg: 横滚角输入，单位为度。
  * @param  pitch_input_deg: 俯仰角输入，单位为度。
  * @retval None
  */
void GyroDebugServo_Update(float roll_input_deg, float pitch_input_deg)
{
  uint16_t roll_servo_angle;
  uint16_t pitch_servo_angle;

  /* 这里只做线性映射，不做 PID。
     用途是单独验证：
     1. 陀螺仪数据方向是否正确
     2. TIM12 两路调试舵机方向是否正确
     3. 角度映射幅度是否符合机械预期 */
  g_gyro_debug_servo_state.roll_input_deg = roll_input_deg;
  g_gyro_debug_servo_state.pitch_input_deg = pitch_input_deg;

  roll_servo_angle = GyroDebugServo_MapInputToServo(roll_input_deg);
  pitch_servo_angle = GyroDebugServo_MapInputToServo(pitch_input_deg);

  g_gyro_debug_servo_state.roll_servo_angle_deg = roll_servo_angle;
  g_gyro_debug_servo_state.pitch_servo_angle_deg = pitch_servo_angle;

  (void)Servo_SetAngle(g_gyro_debug_servo_config.roll_servo_tim, roll_servo_angle);
  (void)Servo_SetAngle(g_gyro_debug_servo_config.pitch_servo_tim, pitch_servo_angle);
}

/**
  * @brief  获取当前双路调试舵机状态。
  * @param  None
  * @retval 返回状态结构体常量指针，便于查看 roll/pitch 输入和舵机输出角。
  */
const GyroDebugServo_StateTypeDef *GyroDebugServo_GetState(void)
{
  return &g_gyro_debug_servo_state;
}


