#include "vision_control.h"

#include <string.h>

#include "control_tick_device.h"
#include "serial_screen_device.h"
#include "system_time_device.h"
#include "uart_device.h"

/* 默认调平参考点与正式控制参数宏定义。
   其中 X/Y 两路舵机的 balance_us 就是平台默认平衡角度对应的中值脉宽，
   后续现场调平时你优先改这两个宏即可。 */
#define VISION_CONTROL_DEFAULT_LEVEL_TARGET_X   PLATE_TASK_LEVEL_DEFAULT_TARGET_X
#define VISION_CONTROL_DEFAULT_LEVEL_TARGET_Y   PLATE_TASK_LEVEL_DEFAULT_TARGET_Y
#define VISION_CONTROL_FRAME_TIMEOUT_MS         200U
#define VISION_CONTROL_FRAME_HOLD_MS            400U
#define VISION_CONTROL_FRAME_HOLD_ERROR_PX      20.0f
#define VISION_CONTROL_FRAME_HOLD_OUTPUT_US     40.0f
#define VISION_CONTROL_FIXED_DT_S               0.005f
/* 视觉分辨率当前为 640x640。
   这里把位置环和速度环死区单独提成宏，便于后续现场快速微调。 */
#define VISION_CONTROL_POSITION_DEADBAND_PX         3.0f
#define VISION_CONTROL_VELOCITY_DEADBAND_PXPS       3.0f
/* 基础题保持原来的输出上限，避免影响已经调顺的定点保持效果。
   发挥题则把 X 轴单独再收一点，优先减小长距离横向运动时的冲板和贴边风险。 */
#define VISION_CONTROL_BASIC_X_POSITION_OUTPUT_LIMIT_PXPS       160.0f
#define VISION_CONTROL_BASIC_Y_POSITION_OUTPUT_LIMIT_PXPS       160.0f
#define VISION_CONTROL_BASIC2_X_POSITION_OUTPUT_LIMIT_PXPS      195.0f
#define VISION_CONTROL_BASIC2_Y_POSITION_OUTPUT_LIMIT_PXPS      200.0f
#define VISION_CONTROL_ADVANCED_X_POSITION_OUTPUT_LIMIT_PXPS     95.0f
#define VISION_CONTROL_ADVANCED_Y_POSITION_OUTPUT_LIMIT_PXPS    120.0f
#define VISION_CONTROL_X_POSITION_KP                1.35f
#define VISION_CONTROL_Y_POSITION_KP                1.20f
#define VISION_CONTROL_X_POSITION_KI                0.05f
#define VISION_CONTROL_Y_POSITION_KI                0.03f
/* 当前机构下，只要追踪 5 号点，最终稳定点都会略偏左。
   因此把“去 5 号点时的 X 目标补偿”统一放到控制层做，
   基础二和其他所有经过 5 的题目都会共用这份修正，
   同时又不会直接改动九点表原始坐标。 */
#define VISION_CONTROL_POINT5_TARGET_X_COMPENSATION_PX          18U
/* 基础二“快速回中心”单独使用一套更保守的 PID。
   这套参数的思路不是继续猛推，而是：
   1. 保留比基础一更大的输出上限，保证回中心不拖沓；
   2. 去掉位置环积分，避免到中心附近以后持续顶着不放；
   3. 适当降低位置环 / 速度环比例，减少大幅往返振荡。 */
#define VISION_CONTROL_BASIC2_X_POSITION_KP         1.08f
#define VISION_CONTROL_BASIC2_Y_POSITION_KP         1.00f
#define VISION_CONTROL_BASIC2_X_POSITION_KI         0.03f
#define VISION_CONTROL_BASIC2_Y_POSITION_KI         0.00f
#define VISION_CONTROL_POSITION_INTEGRAL_LIMIT      250.0f
/* 舵机角度与脉宽采用 0°=500us、180°=2500us 线性映射。
   40° 对应的脉宽偏差约为 444.4us。
   这里把正式控制的最大偏转统一限制为“相对中值 ±40°”。 */
#define VISION_CONTROL_SERVO_DELTA_LIMIT_US     444.4f
/* 速度环先用更小的软限幅调试，避免速度误差较大时持续打到机械硬限幅。 */
#define VISION_CONTROL_BASIC_X_VELOCITY_OUTPUT_LIMIT_US         320.0f
#define VISION_CONTROL_BASIC_Y_VELOCITY_OUTPUT_LIMIT_US         320.0f
#define VISION_CONTROL_BASIC2_X_VELOCITY_OUTPUT_LIMIT_US        320.0f
#define VISION_CONTROL_BASIC2_Y_VELOCITY_OUTPUT_LIMIT_US        340.0f
#define VISION_CONTROL_BASIC2_X_VELOCITY_KP         1.50f
#define VISION_CONTROL_BASIC2_X_VELOCITY_KI         0.35f
#define VISION_CONTROL_BASIC2_X_VELOCITY_INTEGRAL_LIMIT         120.0f
#define VISION_CONTROL_BASIC2_Y_VELOCITY_KP         1.70f
#define VISION_CONTROL_ADVANCED_X_VELOCITY_OUTPUT_LIMIT_US      220.0f
#define VISION_CONTROL_ADVANCED_Y_VELOCITY_OUTPUT_LIMIT_US      260.0f
/* 发挥四完成时间通过串口屏的 X0 虚拟浮点控件显示。
   当前串口屏配置为“3 位整数、2 位小数”，因此这里按 10ms 为 1 个显示单位发送。
   例如 12345ms 会先换算成 1234，再由串口屏显示为 12.34 s。 */
#define VISION_CONTROL_ADVANCED4_TIME_COMPONENT        "x0"
#define VISION_CONTROL_ADVANCED4_TIME_SCALE_MS         10U
#define VISION_CONTROL_ADVANCED4_TIME_UPDATE_PERIOD_MS 100U

#define VISION_CONTROL_X_SERVO_BALANCE_US      1362.8f    /* X 轴零偏按轻微偏左现象做收尾微调 */
#define VISION_CONTROL_X_SERVO_MIN_US           944.4f    /* 约 40° = 80° - 40° */
#define VISION_CONTROL_X_SERVO_MAX_US          1833.3f    /* 约 120° = 80° + 40° */

#define VISION_CONTROL_Y_SERVO_BALANCE_US      1500.0f    /* 90° */
#define VISION_CONTROL_Y_SERVO_MIN_US          1055.6f    /* 约 50° = 90° - 40° */
#define VISION_CONTROL_Y_SERVO_MAX_US          1944.4f    /* 约 130° = 90° + 40° */

/* 顶层控制器负责把各模块串起来:
   1. UART4 视觉收帧，接收当前坐标与可选目标区域号
   2. USART1 串口屏命令与页面显示
   3. TIM11 固定 5ms 控制节拍
   4. TIM2 双轴舵机输出 */
typedef struct
{
  VisionControl_ConfigTypeDef config;
  VisionControl_StateTypeDef state;
  VisionFrameParser_TypeDef parser;
  VisionAxis_TypeDef x_axis;
  VisionAxis_TypeDef y_axis;
  uint32_t last_frame_tick_ms;
  volatile uint8_t control_tick_pending;
  uint8_t last_screen_target_id;
  uint32_t last_screen_time_value;
  uint32_t last_screen_time_send_tick_ms;
  uint32_t advanced4_start_tick_ms;
  uint32_t advanced4_latched_elapsed_ms;
  uint8_t advanced4_time_latched;
} VisionControl_ContextTypeDef;

static VisionControl_ConfigTypeDef g_vision_control_config =
{
  .center_x = VISION_CONTROL_DEFAULT_LEVEL_TARGET_X,
  .center_y = VISION_CONTROL_DEFAULT_LEVEL_TARGET_Y,
  .frame_timeout_ms = VISION_CONTROL_FRAME_TIMEOUT_MS,
  .x_axis =
  {
    .servo_tim = SERVO_TIM2_1,
    .servo_center_us = VISION_CONTROL_X_SERVO_BALANCE_US,
    .servo_min_us = VISION_CONTROL_X_SERVO_MIN_US,
    .servo_max_us = VISION_CONTROL_X_SERVO_MAX_US,
    .servo_direction = -1.0f,
    .velocity_filter_alpha = 0.2f,
    .position_pid =
    {
      .kp = VISION_CONTROL_X_POSITION_KP,
      .ki = VISION_CONTROL_X_POSITION_KI,
      .kd = 0.00f,
      .integral_limit = VISION_CONTROL_POSITION_INTEGRAL_LIMIT,
      .output_limit = VISION_CONTROL_BASIC_X_POSITION_OUTPUT_LIMIT_PXPS,
      .deadband = VISION_CONTROL_POSITION_DEADBAND_PX
    },
    .velocity_pid =
    {
      .kp = 1.8f,
      .ki = 0.00f,
      .kd = 0.00f,
      .integral_limit = 0.0f,
      .output_limit = VISION_CONTROL_BASIC_X_VELOCITY_OUTPUT_LIMIT_US,
      .deadband = VISION_CONTROL_VELOCITY_DEADBAND_PXPS
    }
  },
  .y_axis =
  {
    .servo_tim = SERVO_TIM2_2,
    .servo_center_us = VISION_CONTROL_Y_SERVO_BALANCE_US,
    .servo_min_us = VISION_CONTROL_Y_SERVO_MIN_US,
    .servo_max_us = VISION_CONTROL_Y_SERVO_MAX_US,
    .servo_direction = 1.0f,
    .velocity_filter_alpha = 0.2f,
    .position_pid =
    {
      .kp = VISION_CONTROL_Y_POSITION_KP,
      .ki = VISION_CONTROL_Y_POSITION_KI,
      .kd = 0.00f,
      .integral_limit = VISION_CONTROL_POSITION_INTEGRAL_LIMIT,
      .output_limit = VISION_CONTROL_BASIC_Y_POSITION_OUTPUT_LIMIT_PXPS,
      .deadband = VISION_CONTROL_POSITION_DEADBAND_PX
    },
    .velocity_pid =
    {
      .kp = 2.0f,
      .ki = 0.00f,
      .kd = 0.00f,
      .integral_limit = 0.0f,
      .output_limit = VISION_CONTROL_BASIC_Y_VELOCITY_OUTPUT_LIMIT_US,
      .deadband = VISION_CONTROL_VELOCITY_DEADBAND_PXPS
    }
  }
};

static VisionControl_ContextTypeDef g_vision_control;

/**
  * @brief  简单绝对值函数，避免为了几处比较额外引入标准库浮点接口。
  * @param  value: 输入浮点值。
  * @retval 返回 value 的绝对值。
  */
static float VisionControl_Abs(float value)
{
  if (value < 0.0f)
  {
    return -value;
  }

  return value;
}

/**
  * @brief  比较两套 PID 参数是否完全一致。
  * @param  p_lhs: 左侧参数指针。
  * @param  p_rhs: 右侧参数指针。
  * @retval 1 表示所有字段都一致，0 表示存在差异。
  */
static uint8_t VisionControl_IsPidParamsEqual(const PID_ParamsTypeDef *p_lhs,
                                              const PID_ParamsTypeDef *p_rhs)
{
  if ((p_lhs == (const PID_ParamsTypeDef *)0) ||
      (p_rhs == (const PID_ParamsTypeDef *)0))
  {
    return 0U;
  }

  if ((p_lhs->kp != p_rhs->kp) ||
      (p_lhs->ki != p_rhs->ki) ||
      (p_lhs->kd != p_rhs->kd) ||
      (p_lhs->integral_limit != p_rhs->integral_limit) ||
      (p_lhs->output_limit != p_rhs->output_limit) ||
      (p_lhs->deadband != p_rhs->deadband))
  {
    return 0U;
  }

  return 1U;
}

/**
  * @brief  动态更新单轴位置环和速度环的输出限幅。
  * @param  p_axis: 需要更新限幅的单轴控制对象。
  * @param  position_limit: 位置环输出上限，单位 px/s。
  * @param  velocity_limit: 速度环输出上限，单位 us。
  * @retval None
  */
static void VisionControl_UpdateAxisOutputLimit(VisionAxis_TypeDef *p_axis,
                                                float position_limit,
                                                float velocity_limit)
{
  PID_ParamsTypeDef pid_params;

  if (p_axis == (VisionAxis_TypeDef *)0)
  {
    return;
  }

  if (p_axis->position_pid.params.output_limit != position_limit)
  {
    pid_params = p_axis->position_pid.params;
    pid_params.output_limit = position_limit;
    PID_SetParams(&p_axis->position_pid, &pid_params);
  }

  if (p_axis->velocity_pid.params.output_limit != velocity_limit)
  {
    pid_params = p_axis->velocity_pid.params;
    pid_params.output_limit = velocity_limit;
    PID_SetParams(&p_axis->velocity_pid, &pid_params);
  }
}

/**
  * @brief  动态更新单轴位置环和速度环的 PID 参数。
  * @param  p_axis: 需要更新 PID 的单轴控制对象。
  * @param  position_kp: 位置环比例系数。
  * @param  position_ki: 位置环积分系数。
  * @param  velocity_kp: 速度环比例系数。
  * @param  velocity_ki: 速度环积分系数。
  * @param  velocity_integral_limit: 速度环积分限幅。
  * @retval None
  *
  * @note   这里只改与“控制力度”相关的参数，当前任务对应的输出限幅仍由
  *         VisionControl_ApplyOutputLimitByTask() 统一管理。
  *         一旦检测到 PID 参数确实发生切换，就顺手清掉该环的历史误差和积分项，
  *         避免基础一切到基础二时把旧任务的积分残留带进来。
  */
static void VisionControl_UpdateAxisPidProfile(VisionAxis_TypeDef *p_axis,
                                               float position_kp,
                                               float position_ki,
                                               float velocity_kp,
                                               float velocity_ki,
                                               float velocity_integral_limit)
{
  PID_ParamsTypeDef pid_params;

  if (p_axis == (VisionAxis_TypeDef *)0)
  {
    return;
  }

  pid_params = p_axis->position_pid.params;
  pid_params.kp = position_kp;
  pid_params.ki = position_ki;
  pid_params.kd = 0.0f;
  pid_params.integral_limit = VISION_CONTROL_POSITION_INTEGRAL_LIMIT;
  pid_params.deadband = VISION_CONTROL_POSITION_DEADBAND_PX;
  if (VisionControl_IsPidParamsEqual(&p_axis->position_pid.params, &pid_params) == 0U)
  {
    PID_SetParams(&p_axis->position_pid, &pid_params);
    PID_Reset(&p_axis->position_pid);
  }

  pid_params = p_axis->velocity_pid.params;
  pid_params.kp = velocity_kp;
  pid_params.ki = velocity_ki;
  pid_params.kd = 0.0f;
  pid_params.integral_limit = velocity_integral_limit;
  pid_params.deadband = VISION_CONTROL_VELOCITY_DEADBAND_PXPS;
  if (VisionControl_IsPidParamsEqual(&p_axis->velocity_pid.params, &pid_params) == 0U)
  {
    PID_SetParams(&p_axis->velocity_pid, &pid_params);
    PID_Reset(&p_axis->velocity_pid);
  }
}

/**
  * @brief  按当前任务切换基础题 / 基础二 / 发挥题对应的 PID 配置。
  * @param  task_id: 当前题目编号。
  * @retval None
  */
static void VisionControl_ApplyPidByTask(PlateTask_TaskIdTypeDef task_id)
{
  if (task_id == PLATE_TASK_ID_BASIC2)
  {
    VisionControl_UpdateAxisPidProfile(&g_vision_control.x_axis,
                                       VISION_CONTROL_BASIC2_X_POSITION_KP,
                                       VISION_CONTROL_BASIC2_X_POSITION_KI,
                                       VISION_CONTROL_BASIC2_X_VELOCITY_KP,
                                       VISION_CONTROL_BASIC2_X_VELOCITY_KI,
                                       VISION_CONTROL_BASIC2_X_VELOCITY_INTEGRAL_LIMIT);
    VisionControl_UpdateAxisPidProfile(&g_vision_control.y_axis,
                                       VISION_CONTROL_BASIC2_Y_POSITION_KP,
                                       VISION_CONTROL_BASIC2_Y_POSITION_KI,
                                       VISION_CONTROL_BASIC2_Y_VELOCITY_KP,
                                       0.0f,
                                       0.0f);
    return;
  }

  VisionControl_UpdateAxisPidProfile(&g_vision_control.x_axis,
                                     VISION_CONTROL_X_POSITION_KP,
                                     VISION_CONTROL_X_POSITION_KI,
                                     1.8f,
                                     0.0f,
                                     0.0f);
  VisionControl_UpdateAxisPidProfile(&g_vision_control.y_axis,
                                     VISION_CONTROL_Y_POSITION_KP,
                                     VISION_CONTROL_Y_POSITION_KI,
                                     2.0f,
                                     0.0f,
                                     0.0f);
}

/**
  * @brief  按当前题目模式切换基础题和发挥题的输出限幅。
  * @param  mode: 当前题目模式。
  * @retval None
  */
static void VisionControl_ApplyOutputLimitByTask(PlateTask_ModeTypeDef mode,
                                                 PlateTask_TaskIdTypeDef task_id)
{
  float x_position_limit;
  float y_position_limit;
  float x_velocity_limit;
  float y_velocity_limit;

  if (task_id == PLATE_TASK_ID_BASIC2)
  {
    /* 基础二只有一个“尽快回中心点并稳定保持”的目标。
       这里单独把基础二的赶路限幅放宽一档，
       让它回中心更利索，同时不影响基础一已经调好的保持手感。 */
    x_position_limit = VISION_CONTROL_BASIC2_X_POSITION_OUTPUT_LIMIT_PXPS;
    y_position_limit = VISION_CONTROL_BASIC2_Y_POSITION_OUTPUT_LIMIT_PXPS;
    x_velocity_limit = VISION_CONTROL_BASIC2_X_VELOCITY_OUTPUT_LIMIT_US;
    y_velocity_limit = VISION_CONTROL_BASIC2_Y_VELOCITY_OUTPUT_LIMIT_US;
  }
  else if (mode == PLATE_TASK_MODE_ADVANCED)
  {
    x_position_limit = VISION_CONTROL_ADVANCED_X_POSITION_OUTPUT_LIMIT_PXPS;
    y_position_limit = VISION_CONTROL_ADVANCED_Y_POSITION_OUTPUT_LIMIT_PXPS;
    x_velocity_limit = VISION_CONTROL_ADVANCED_X_VELOCITY_OUTPUT_LIMIT_US;
    y_velocity_limit = VISION_CONTROL_ADVANCED_Y_VELOCITY_OUTPUT_LIMIT_US;
  }
  else
  {
    x_position_limit = VISION_CONTROL_BASIC_X_POSITION_OUTPUT_LIMIT_PXPS;
    y_position_limit = VISION_CONTROL_BASIC_Y_POSITION_OUTPUT_LIMIT_PXPS;
    x_velocity_limit = VISION_CONTROL_BASIC_X_VELOCITY_OUTPUT_LIMIT_US;
    y_velocity_limit = VISION_CONTROL_BASIC_Y_VELOCITY_OUTPUT_LIMIT_US;
  }

  VisionControl_UpdateAxisOutputLimit(&g_vision_control.x_axis, x_position_limit, x_velocity_limit);
  VisionControl_UpdateAxisOutputLimit(&g_vision_control.y_axis, y_position_limit, y_velocity_limit);
}

/**
  * @brief  将 X/Y 双轴内部状态同步到对外状态结构体。
  * @param  None
  * @retval None
  */
static void VisionControl_SyncAxisState(void)
{
  const VisionAxis_StateTypeDef *p_x_state;
  const VisionAxis_StateTypeDef *p_y_state;

  p_x_state = VisionAxis_GetState(&g_vision_control.x_axis);
  p_y_state = VisionAxis_GetState(&g_vision_control.y_axis);

  if (p_x_state != (const VisionAxis_StateTypeDef *)0)
  {
    g_vision_control.state.x_axis = *p_x_state;
  }

  if (p_y_state != (const VisionAxis_StateTypeDef *)0)
  {
    g_vision_control.state.y_axis = *p_y_state;
  }
}

/**
  * @brief  向串口屏实时刷新当前目标点编号 n2。
  * @param  target_id: 当前正在追踪的目标点编号，范围 0~9。
  * @retval None
  */
static void VisionControl_UpdateScreenTargetPoint(uint8_t target_id)
{
  if (g_vision_control.last_screen_target_id == target_id)
  {
    return;
  }

  g_vision_control.last_screen_target_id = target_id;
  (void)SerialScreen_Device_SendNumberValue("n2", (uint32_t)target_id);
}

/**
  * @brief  向串口屏的 X0 虚拟浮点控件发送 1 次时间数值。
  * @param  elapsed_ms: 需要显示的原始毫秒时间。
  * @retval None
  */
static void VisionControl_SendAdvanced4TimeValue(uint32_t elapsed_ms)
{
  uint32_t screen_value;

  screen_value = elapsed_ms / VISION_CONTROL_ADVANCED4_TIME_SCALE_MS;
  g_vision_control.last_screen_time_value = screen_value;
  (void)SerialScreen_Device_SendNumberValue(VISION_CONTROL_ADVANCED4_TIME_COMPONENT,
                                            screen_value);
}

/**
  * @brief  清除发挥四计时显示的内部锁存状态，并按需将 X0 清零。
  * @param  send_zero: 1 表示立即向串口屏发送 X0=0，0 表示只清本地状态不发串口。
  * @retval None
  */
static void VisionControl_ResetAdvanced4TimeDisplay(uint8_t send_zero)
{
  g_vision_control.last_screen_time_value = 0U;
  g_vision_control.last_screen_time_send_tick_ms = 0U;
  g_vision_control.advanced4_start_tick_ms = 0U;
  g_vision_control.advanced4_latched_elapsed_ms = 0U;
  g_vision_control.advanced4_time_latched = 0U;

  if (send_zero != 0U)
  {
    (void)SerialScreen_Device_SendNumberValue(VISION_CONTROL_ADVANCED4_TIME_COMPONENT, 0U);
  }
}

/**
  * @brief  根据发挥四任务运行状态，实时刷新串口屏 X0 上的总耗时。
  * @param  p_task_state: 当前题目状态机只读状态。
  * @param  now_ms: 当前系统毫秒节拍。
  * @retval None
  */
static void VisionControl_UpdateAdvanced4TimeDisplay(const PlateTask_StateTypeDef *p_task_state,
                                                     uint32_t now_ms)
{
  uint32_t elapsed_ms;
  uint32_t screen_value;

  if (p_task_state == (const PlateTask_StateTypeDef *)0)
  {
    return;
  }

  if (p_task_state->task_id != PLATE_TASK_ID_ADVANCED4)
  {
    if ((g_vision_control.advanced4_start_tick_ms != 0U) ||
        (g_vision_control.last_screen_time_value != 0U) ||
        (g_vision_control.advanced4_time_latched != 0U))
    {
      VisionControl_ResetAdvanced4TimeDisplay(1U);
    }
    return;
  }

  if (p_task_state->task_start_tick_ms == 0U)
  {
    if ((g_vision_control.advanced4_start_tick_ms != 0U) ||
        (g_vision_control.last_screen_time_value != 0U) ||
        (g_vision_control.advanced4_time_latched != 0U))
    {
      VisionControl_ResetAdvanced4TimeDisplay(1U);
    }
    return;
  }

  if (g_vision_control.advanced4_start_tick_ms != p_task_state->task_start_tick_ms)
  {
    g_vision_control.advanced4_start_tick_ms = p_task_state->task_start_tick_ms;
    g_vision_control.last_screen_time_value = 0U;
    g_vision_control.last_screen_time_send_tick_ms = 0U;
    g_vision_control.advanced4_latched_elapsed_ms = 0U;
    g_vision_control.advanced4_time_latched = 0U;
  }

  if (g_vision_control.advanced4_time_latched != 0U)
  {
    elapsed_ms = g_vision_control.advanced4_latched_elapsed_ms;
  }
  else
  {
    elapsed_ms = p_task_state->task_elapsed_ms;
    if ((elapsed_ms == 0U) && (now_ms >= p_task_state->task_start_tick_ms))
    {
      elapsed_ms = now_ms - p_task_state->task_start_tick_ms;
    }

    if (p_task_state->task_finished != 0U)
    {
      g_vision_control.advanced4_latched_elapsed_ms = elapsed_ms;
      g_vision_control.advanced4_time_latched = 1U;
    }
  }

  screen_value = elapsed_ms / VISION_CONTROL_ADVANCED4_TIME_SCALE_MS;
  if ((g_vision_control.advanced4_time_latched != 0U) ||
      ((now_ms - g_vision_control.last_screen_time_send_tick_ms) >=
       VISION_CONTROL_ADVANCED4_TIME_UPDATE_PERIOD_MS))
  {
    if ((screen_value != g_vision_control.last_screen_time_value) ||
        (g_vision_control.last_screen_time_send_tick_ms == 0U))
    {
      VisionControl_SendAdvanced4TimeValue(elapsed_ms);
      g_vision_control.last_screen_time_send_tick_ms = now_ms;
    }
  }
}

/**
  * @brief  统一执???双轴回??并清空控制器内部状态??
  * @param  None
  * @retval None
  */
static void VisionControl_ResetOutput(void)
{
  VisionAxis_Reset(&g_vision_control.x_axis);
  VisionAxis_Reset(&g_vision_control.y_axis);
  VisionControl_SyncAxisState();
  g_vision_control.state.control_enabled = 0U;
}

/**
  * @brief  清空视觉输入缓冲、解析器状态和最近一次视觉帧。
  * @param  None
  * @retval None
  */
static void VisionControl_ResetVisionInput(void)
{
  Uart_Device_ClearByteQueue(UART_DEVICE_UART4);
  VisionFrameParser_Clear(&g_vision_control.parser);
  g_vision_control.last_frame_tick_ms = 0U;
  memset(&g_vision_control.state.frame, 0, sizeof(g_vision_control.state.frame));
  g_vision_control.state.vision_online = 0U;
}

/**
  * @brief  把最新视觉帧中的当前坐标实时刷新到串口屏的 n0/n1 数值控件。
  * @param  p_frame: 最近一次解析成功的视觉帧指针。
  * @retval None
  */
static void VisionControl_UpdateScreenCurrentCoordinate(const VisionFrame_TypeDef *p_frame)
{
  if ((p_frame == (const VisionFrame_TypeDef *)0) || (p_frame->frame_valid == 0U))
  {
    return;
  }

  (void)SerialScreen_Device_SendNumberValue("n0", (uint32_t)p_frame->current_x);
  (void)SerialScreen_Device_SendNumberValue("n1", (uint32_t)p_frame->current_y);
}

/**
  * @brief  用最新视觉帧更新双轴的当前位置和速度估计。
  * @param  None
  * @retval None
  */
static void VisionControl_UpdateFrameMeasurement(void)
{
  const VisionFrame_TypeDef *p_frame;
  float frame_dt_s = 0.0f;

  p_frame = VisionFrameParser_GetLatest(&g_vision_control.parser);
  if ((p_frame == (const VisionFrame_TypeDef *)0) || (p_frame->frame_valid == 0U))
  {
    return;
  }

  g_vision_control.state.frame = *p_frame;
  VisionControl_UpdateScreenCurrentCoordinate(p_frame);

  /* 视觉帧到达时间不固定，因此速度估计必须使用真实帧间隔。 */
  if ((g_vision_control.last_frame_tick_ms != 0U) &&
      (p_frame->recv_tick_ms > g_vision_control.last_frame_tick_ms))
  {
    frame_dt_s =
      ((float)(p_frame->recv_tick_ms - g_vision_control.last_frame_tick_ms)) * 0.001f;
  }

  VisionAxis_UpdateMeasurement(&g_vision_control.x_axis, (float)p_frame->current_x, frame_dt_s);
  VisionAxis_UpdateMeasurement(&g_vision_control.y_axis, (float)p_frame->current_y, frame_dt_s);

  g_vision_control.last_frame_tick_ms = p_frame->recv_tick_ms;
  VisionControl_SyncAxisState();
}

/**
  * @brief  消费串口屏输入字节，并交给题目状态机处理。
  * @param  None
  * @retval None
  */
static void VisionControl_ServiceScreenCommand(void)
{
  uint8_t command;
  uint8_t handled = 0U;

  while (SerialScreen_Device_PopCommand(&command) != 0U)
  {
    /* 0xCC 为最高优先级复位调平命令。
       串口屏一旦发出该字节，立即退出当前所有题目流程，
       清空已缓存的输入序列，并让 TIM2 双轴回到默认平衡状态。 */
    if (command == SERIAL_SCREEN_CMD_RESET_LEVEL)
    {
      VisionControl_RequestLevelReset();
      handled = 1U;
      continue;
    }

    if (PlateTaskFsm_HandleCommand(command) != 0U)
    {
      handled = 1U;
    }
  }

  if (handled != 0U)
  {
    /* 只要任务协议有变化，就清一次积分与输出，避免切题瞬间冲击平台。 */
    VisionControl_ResetOutput();
  }
}

/**
  * @brief  取出 1 个待执行的 TIM11 控制节拍。
  * @param  None
  * @retval 1 表示成功取到 1 个待执行节拍，0 表示当前没有待执行节拍。
  */
static uint8_t VisionControl_TakeTick(void)
{
  uint8_t has_tick = 0U;

  if (g_vision_control.control_tick_pending > 0U)
  {
    g_vision_control.control_tick_pending--;
    g_vision_control.state.control_tick_pending = g_vision_control.control_tick_pending;
    has_tick = 1U;
  }

  return has_tick;
}

/**
  * @brief  按固定 5ms 周期执行 1 次完整控制拍。
  * @param  now_ms: 当前系统毫秒节拍。
  * @retval None
  */
static void VisionControl_RunFixedTick(uint32_t now_ms)
{
  const PlateTask_StateTypeDef *p_task_state;
  uint8_t frame_is_fresh = 0U;
  uint32_t frame_age_ms = 0xFFFFFFFFU;
  float position_error_x = 0.0f;
  float position_error_y = 0.0f;

  if ((g_vision_control.state.frame.frame_valid != 0U) &&
      (now_ms >= g_vision_control.state.frame.recv_tick_ms))
  {
    frame_age_ms = now_ms - g_vision_control.state.frame.recv_tick_ms;

    if (frame_age_ms <= g_vision_control.config.frame_timeout_ms)
    {
      frame_is_fresh = 1U;
    }
  }

  g_vision_control.state.vision_online = frame_is_fresh;

  PlateTaskFsm_Update(now_ms,
                      g_vision_control.state.frame.current_x,
                      g_vision_control.state.frame.current_y,
                      g_vision_control.state.frame.target_region_id,
                      g_vision_control.state.frame.target_region_valid,
                      frame_is_fresh);

  p_task_state = PlateTaskFsm_GetState();
  g_vision_control.state.mode = p_task_state->mode;
  g_vision_control.state.task_id = (uint8_t)p_task_state->task_id;
  g_vision_control.state.task_state = p_task_state->run_state;
  g_vision_control.state.task_finished = p_task_state->task_finished;
  g_vision_control.state.task_timeout = p_task_state->task_timeout;
  g_vision_control.state.active_target_x = p_task_state->active_target_x;
  g_vision_control.state.active_target_y = p_task_state->active_target_y;
  if (p_task_state->current_target_id == 5U)
  {
    /* 只要当前目标点编号是 5，就统一附加同一份 X 补偿。
       这样基础二、基础三/四、发挥题中经过 5 的路径点都会更容易落到 5 的中心。 */
    g_vision_control.state.active_target_x += VISION_CONTROL_POINT5_TARGET_X_COMPENSATION_PX;
  }
  g_vision_control.state.target_point_id = p_task_state->current_target_id;
  g_vision_control.state.task_elapsed_ms = p_task_state->task_elapsed_ms;
  VisionControl_UpdateAdvanced4TimeDisplay(p_task_state, now_ms);

  /* 基础题保持原来的输出力度。
     发挥题单独把位置环和速度环的输出限幅压低一档，
     重点降低长距离切换时的冲击，不改基础题已经调好的终点稳定性。 */
  VisionControl_ApplyPidByTask(p_task_state->task_id);
  VisionControl_ApplyOutputLimitByTask(p_task_state->mode,
                                       p_task_state->task_id);

  position_error_x =
    (float)((int32_t)g_vision_control.state.active_target_x -
            (int32_t)g_vision_control.state.frame.current_x);
  position_error_y =
    (float)((int32_t)g_vision_control.state.active_target_y -
            (int32_t)g_vision_control.state.frame.current_y);

  VisionControl_UpdateScreenTargetPoint(p_task_state->current_target_id);

  if (PlateTaskFsm_IsControlEnabled() == 0U)
  {
    VisionControl_ResetOutput();
    return;
  }

  if (g_vision_control.state.frame.frame_valid == 0U)
  {
    VisionControl_ResetOutput();
    return;
  }

  if (frame_is_fresh == 0U)
  {
    /* 视觉偶发丢 1~2 帧时，保持上一拍舵机输出，
       避免平台因为瞬时回中把小球又放跑。
       但只有“已经接近目标，且当前控制输出不大”时才允许保持；
       如果此时离目标还远、舵机还在大幅动作，就不能盲目沿用旧输出，
       否则小球可能会被继续推离目标。 */
    if ((frame_age_ms <= VISION_CONTROL_FRAME_HOLD_MS) &&
        (VisionControl_Abs(position_error_x) <= VISION_CONTROL_FRAME_HOLD_ERROR_PX) &&
        (VisionControl_Abs(position_error_y) <= VISION_CONTROL_FRAME_HOLD_ERROR_PX) &&
        (VisionControl_Abs(g_vision_control.state.x_axis.servo_output_us) <=
         VISION_CONTROL_FRAME_HOLD_OUTPUT_US) &&
        (VisionControl_Abs(g_vision_control.state.y_axis.servo_output_us) <=
         VISION_CONTROL_FRAME_HOLD_OUTPUT_US))
    {
      g_vision_control.state.control_enabled = 1U;
      return;
    }

    VisionControl_ResetOutput();
    return;
  }

  if ((VisionAxis_Run(&g_vision_control.x_axis,
                      (float)g_vision_control.state.active_target_x,
                      (float)g_vision_control.state.frame.current_x,
                      VISION_CONTROL_FIXED_DT_S) != BSP_OK) ||
      (VisionAxis_Run(&g_vision_control.y_axis,
                      (float)g_vision_control.state.active_target_y,
                      (float)g_vision_control.state.frame.current_y,
                      VISION_CONTROL_FIXED_DT_S) != BSP_OK))
  {
    VisionControl_ResetOutput();
    return;
  }

  g_vision_control.state.control_enabled = 1U;
  VisionControl_SyncAxisState();
}

/**
  * @brief  获取顶层控制器配置结构体，便于外部调参。
  * @param  None
  * @retval 指向全局控制配置的指针。
  */
VisionControl_ConfigTypeDef *VisionControl_GetConfig(void)
{
  return &g_vision_control_config;
}

/**
  * @brief  初始化顶层控制器及其依赖模块。
  * @param  None
  * @retval BSP_OK 表示初始化成功，BSP_ERROR 表示其中某个子模块初始化失败。
  */
BSP_StatusTypeDef VisionControl_Init(void)
{
  memset(&g_vision_control, 0, sizeof(g_vision_control));

  g_vision_control.config = g_vision_control_config;
  g_vision_control.state.active_target_x = g_vision_control.config.center_x;
  g_vision_control.state.active_target_y = g_vision_control.config.center_y;
  g_vision_control.last_screen_target_id = 0xFFU;

  VisionFrameParser_Init(&g_vision_control.parser);
  VisionControl_ResetVisionInput();

  if (VisionAxis_Init(&g_vision_control.x_axis, &g_vision_control.config.x_axis) != BSP_OK)
  {
    return BSP_ERROR;
  }

  if (VisionAxis_Init(&g_vision_control.y_axis, &g_vision_control.config.y_axis) != BSP_OK)
  {
    return BSP_ERROR;
  }

  if (SerialScreen_Device_Init() != BSP_OK)
  {
    return BSP_ERROR;
  }

  if (Uart_Device_StartByteReceiveIT(UART_DEVICE_UART4) != BSP_OK)
  {
    return BSP_ERROR;
  }

  PlateTaskFsm_Init(g_vision_control.config.center_x, g_vision_control.config.center_y);
  VisionControl_ResetOutput();
  VisionControl_UpdateScreenTargetPoint(0U);
  VisionControl_ResetAdvanced4TimeDisplay(1U);

  if (ControlTick_Device_Init(VisionControl_OnControlTick) != BSP_OK)
  {
    return BSP_ERROR;
  }

  return BSP_OK;
}

/**
  * @brief  顶层主循环任务，负责消费输入并执行待处理的控制拍。
  * @param  None
  * @retval None
  */
void VisionControl_Task(void)
{
  uint32_t now_ms;

  now_ms = SystemTime_Device_GetTickMs();

  if (VisionFrameParser_PollInput(&g_vision_control.parser, now_ms) != 0U)
  {
    VisionControl_UpdateFrameMeasurement();
  }

  //
  VisionControl_ServiceScreenCommand();

  while (VisionControl_TakeTick() != 0U)
  {
    now_ms = SystemTime_Device_GetTickMs();
    VisionControl_RunFixedTick(now_ms);
    g_vision_control.state.control_tick_count++;
  }
}

/**
  * @brief  主动请求进入默认调平模式，并清空串口屏与视觉端已缓存的输入。
  * @param  None
  * @retval None
  */
void VisionControl_RequestLevelReset(void)
{
  SerialScreen_Device_ClearCommandQueue();
  PlateTaskFsm_RequestLevelReset();
  VisionControl_ResetVisionInput();
  VisionControl_ResetOutput();
  VisionControl_UpdateScreenTargetPoint(0U);
  VisionControl_ResetAdvanced4TimeDisplay(1U);
}

/**
  * @brief  更新控制层中心参考点。
  * @param  center_x: 新的中心参考点 X 坐标。
  * @param  center_y: 新的中心参考点 Y 坐标。
  * @retval None
  */
void VisionControl_SetCenter(uint16_t center_x, uint16_t center_y)
{
  PlateTask_ConfigTypeDef *p_task_config;

  g_vision_control_config.center_x = center_x;
  g_vision_control_config.center_y = center_y;

  g_vision_control.config.center_x = center_x;
  g_vision_control.config.center_y = center_y;

  p_task_config = PlateTaskFsm_GetConfig();
  p_task_config->level_target_x = center_x;
  p_task_config->level_target_y = center_y;
}

/**
  * @brief  获取当前控制器运行状态。
  * @param  None
  * @retval 指向控制器状态结构体的常量指针。
  */
const VisionControl_StateTypeDef *VisionControl_GetState(void)
{
  return &g_vision_control.state;
}

/**
  * @brief  由 Devices 层调用，通知控制层新增 1 个待执行控制拍。
  * @param  None
  * @retval None
  */
void VisionControl_OnControlTick(void)
{
  if (g_vision_control.control_tick_pending < 10U)
  {
    g_vision_control.control_tick_pending++;
  }

  g_vision_control.state.control_tick_pending = g_vision_control.control_tick_pending;
}
