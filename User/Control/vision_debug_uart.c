#include "vision_debug_uart.h"

#include <stdio.h>

#include "system_time_device.h"
#include "uart_device.h"
#include "vision_control.h"

/* 串口六调试任务只负责输出控制器状态，不参与任何控制决策。 */
typedef struct
{
  VisionDebugUart_ConfigTypeDef config;
  uint32_t last_position_send_tick_ms;
  uint32_t last_velocity_send_tick_ms;
} VisionDebugUart_ContextTypeDef;

static VisionDebugUart_ContextTypeDef g_vision_debug_uart =
{
  {1U, 60U},
  0U,
  0U
};

/**
  * @brief  把浮点值按四舍五入规则转换为 long 整数。
  * @param  value: 待转换的浮点数。
  * @retval 返回四舍五入后的 long 整数结果。
  */
static long VisionDebugUart_RoundToLong(float value)
{
  if (value >= 0.0f)
  {
    return (long)(value + 0.5f);
  }

  return (long)(value - 0.5f);
}

/**
  * @brief  计算最近一帧视觉数据相对当前时刻的帧龄。
  * @param  p_state: 顶层控制器状态指针，用于读取最近视觉帧时间戳。
  * @param  now_ms: 当前系统毫秒节拍。
  * @retval 返回最近一帧视觉数据的帧龄，单位毫秒。
  *         如果当前没有有效视觉帧，则返回 0xFFFFFFFF。
  */
static uint32_t VisionDebugUart_GetFrameAgeMs(const VisionControl_StateTypeDef *p_state, uint32_t now_ms)
{
  if ((p_state == (const VisionControl_StateTypeDef *)0) ||
      (p_state->frame.frame_valid == 0U) ||
      (now_ms < p_state->frame.recv_tick_ms))
  {
    return 0xFFFFFFFFU;
  }

  return now_ms - p_state->frame.recv_tick_ms;
}

/**
  * @brief  初始化视觉调试串口任务，并配置 USART6 为较短的阻塞超时。
  * @param  None
  * @retval BSP_OK 表示初始化成功，BSP_ERROR 表示 USART6 初始化失败。
  */
BSP_StatusTypeDef VisionDebugUart_Init(void)
{
  g_vision_debug_uart.last_position_send_tick_ms = 0U;
  g_vision_debug_uart.last_velocity_send_tick_ms = 0U;

  if (Uart_Device_Init(UART_DEVICE_USART6) != BSP_OK)
  {
    return BSP_ERROR;
  }

  if (Uart_Device_SetTimeout(UART_DEVICE_USART6, 20U) != BSP_OK)
  {
    return BSP_ERROR;
  }

  return BSP_OK;
}

/**
  * @brief  获取视觉调试串口任务配置结构体。
  * @param  None
  * @retval 返回全局配置结构体指针，便于在线修改发送开关和发送周期。
  */
VisionDebugUart_ConfigTypeDef *VisionDebugUart_GetConfig(void)
{
  return &g_vision_debug_uart.config;
}

/**
  * @brief  视觉调试串口周期任务。
  * @param  None
  * @retval None
  */
void VisionDebugUart_Task(void)
{
  const VisionControl_StateTypeDef *p_state;
  uint32_t now_ms;
  uint32_t frame_age_ms;
  long current_vel_x;
  long current_vel_y;
  long target_vel_x;
  long target_vel_y;
  long pos_out_x;
  long pos_out_y;
  long servo_out_x;
  long servo_out_y;
  int length;
  char tx_buffer[256];

  if (g_vision_debug_uart.config.enable == 0U)
  {
    return;
  }

  now_ms = SystemTime_Device_GetTickMs();
  if ((g_vision_debug_uart.config.period_ms > 0U) &&
      ((now_ms - g_vision_debug_uart.last_position_send_tick_ms) < g_vision_debug_uart.config.period_ms))
  {
    return;
  }

  if (Uart_Device_GetTxStatus(UART_DEVICE_USART6) != BSP_OK)
  {
    return;
  }

  p_state = VisionControl_GetState();
  if (p_state == (const VisionControl_StateTypeDef *)0)
  {
    return;
  }

  g_vision_debug_uart.last_position_send_tick_ms = now_ms;
  frame_age_ms = VisionDebugUart_GetFrameAgeMs(p_state, now_ms);

  /* 位置环输出对应外环 PID 的输出 target_velocity。
     这里把位置、速度、舵机输出一次性打出来，方便逐题调参时同时判断：
     1. 当前点是不是视觉噪声大
     2. 目标速度是不是给得太猛
     3. 速度环输出是不是已经顶到限幅 */
  current_vel_x = VisionDebugUart_RoundToLong(p_state->x_axis.measured_velocity);
  current_vel_y = VisionDebugUart_RoundToLong(p_state->y_axis.measured_velocity);
  target_vel_x = VisionDebugUart_RoundToLong(p_state->x_axis.target_velocity);
  target_vel_y = VisionDebugUart_RoundToLong(p_state->y_axis.target_velocity);
  pos_out_x = target_vel_x;
  pos_out_y = target_vel_y;
  servo_out_x = VisionDebugUart_RoundToLong(p_state->x_axis.servo_output_us);
  servo_out_y = VisionDebugUart_RoundToLong(p_state->y_axis.servo_output_us);

  /* 文本字段说明：
     MODE / TASK / RUN : 当前模式、题号、题目运行状态
     PT                : 当前目标点编号
     VREG              : 视觉帧里最近一次收到的目标区域号，主要给 0xB5 模式观察
     VO / CE           : 视觉在线标志、控制使能标志
     CUR_X / CUR_Y     : 当前视觉坐标
     TGT_X / TGT_Y     : 当前状态机输出的目标像素坐标
     POS_OUT_X / Y     : 位置环输出，也就是给速度环的目标速度
     CUR_VX / CUR_VY   : 当前视觉速度估计
     VEL_OUT_X / Y     : 速度环输出，也就是最终舵机脉宽增量
     AGE               : 最近一帧视觉数据的帧龄 */
  length = snprintf(tx_buffer,
                    sizeof(tx_buffer),
                    "MODE:%u,TASK:%u,RUN:%u,PT:%u,VREG:%u,VO:%u,CE:%u,CUR_X:%u,CUR_Y:%u,TGT_X:%u,TGT_Y:%u,POS_OUT_X:%ld,POS_OUT_Y:%ld,CUR_VX:%ld,CUR_VY:%ld,VEL_OUT_X:%ld,VEL_OUT_Y:%ld,AGE:%lu\r\n",
                    (unsigned int)p_state->mode,
                    (unsigned int)p_state->task_id,
                    (unsigned int)p_state->task_state,
                    (unsigned int)p_state->target_point_id,
                    (unsigned int)p_state->frame.target_region_id,
                    (unsigned int)p_state->vision_online,
                    (unsigned int)p_state->control_enabled,
                    (unsigned int)p_state->frame.current_x,
                    (unsigned int)p_state->frame.current_y,
                    (unsigned int)p_state->active_target_x,
                    (unsigned int)p_state->active_target_y,
                    pos_out_x,
                    pos_out_y,
                    current_vel_x,
                    current_vel_y,
                    servo_out_x,
                    servo_out_y,
                    (unsigned long)frame_age_ms);

  if ((length <= 0) || ((uint32_t)length >= sizeof(tx_buffer)))
  {
    return;
  }

  (void)Uart_Device_Send(UART_DEVICE_USART6, (const uint8_t *)tx_buffer, (uint16_t)length);
}

/**
  * @brief  速度环调试串口周期任务。
  * @param  None
  * @retval None
  */
void VisionDebugUart_VelocityTask(void)
{
  const VisionControl_StateTypeDef *p_state;
  uint32_t now_ms;
  long current_vel_x;
  long current_vel_y;
  long target_vel_x;
  long target_vel_y;
  long vel_out_x;
  long vel_out_y;
  int length;
  char tx_buffer[192];

  if (g_vision_debug_uart.config.enable == 0U)
  {
    return;
  }

  now_ms = SystemTime_Device_GetTickMs();
  if ((g_vision_debug_uart.config.period_ms > 0U) &&
      ((now_ms - g_vision_debug_uart.last_velocity_send_tick_ms) < g_vision_debug_uart.config.period_ms))
  {
    return;
  }

  if (Uart_Device_GetTxStatus(UART_DEVICE_USART6) != BSP_OK)
  {
    return;
  }

  p_state = VisionControl_GetState();
  if (p_state == (const VisionControl_StateTypeDef *)0)
  {
    return;
  }

  g_vision_debug_uart.last_velocity_send_tick_ms = now_ms;

  current_vel_x = VisionDebugUart_RoundToLong(p_state->x_axis.measured_velocity);
  current_vel_y = VisionDebugUart_RoundToLong(p_state->y_axis.measured_velocity);
  target_vel_x = VisionDebugUart_RoundToLong(p_state->x_axis.target_velocity);
  target_vel_y = VisionDebugUart_RoundToLong(p_state->y_axis.target_velocity);
  vel_out_x = VisionDebugUart_RoundToLong(p_state->x_axis.servo_output_us);
  vel_out_y = VisionDebugUart_RoundToLong(p_state->y_axis.servo_output_us);

  /* 文本字段说明：
     CUR_VX / CUR_VY  : 当前速度估计值
     TGT_VX / TGT_VY  : 目标速度，也就是位置环输出
     VEL_OUT_X / Y    : 速度环输出，也就是最终舵机脉宽增量 */
  length = snprintf(tx_buffer,
                    sizeof(tx_buffer),
                    "%ld,%ld,%ld,%ld,%ld,%ld\r\n",
                    current_vel_x,
                    current_vel_y,
                    target_vel_x,
                    target_vel_y,
                    vel_out_x,
                    vel_out_y);

  if ((length <= 0) || ((uint32_t)length >= sizeof(tx_buffer)))
  {
    return;
  }

  (void)Uart_Device_Send(UART_DEVICE_USART6, (const uint8_t *)tx_buffer, (uint16_t)length);
}





