#include "plate_task_fsm.h"

#include <string.h>

#include "serial_screen_device.h"
#include "system_time_device.h"

/* 发挥二固定做“两次往返”。
   这里把“往返”定义为：从第 1 个输入点出发，到第 2 个输入点，再回到第 1 个输入点。 */
#define PLATE_TASK_ADVANCED2_ROUND_TRIP_COUNT   2U

/* 题目状态机负责两件事:
   1. 按串口屏协议解析题型选择和目标点输入
   2. 根据当前题型决定“下一步该追哪个点、是否需要停留、是否需要循环” */
typedef struct
{
  PlateTask_ConfigTypeDef config;
  PlateTask_StateTypeDef state;
  uint8_t input_points[PLATE_TASK_MAX_INPUT_POINT_COUNT];
  uint8_t input_count;
  PlateTask_TaskIdTypeDef input_task_id;
  uint8_t run_points[PLATE_TASK_MAX_INPUT_POINT_COUNT];
  uint8_t run_point_count;
  uint8_t advanced2_started;
  uint8_t advanced2_completed_round_trip_count;
} PlateTask_ContextTypeDef;

static PlateTask_ContextTypeDef g_plate_task_context;

/**
  * @brief  清空当前正在接收的目标点输入缓存。
  * @param  None
  * @retval None
  */
static void PlateTaskFsm_ClearInputBuffer(void)
{
  memset(g_plate_task_context.input_points, 0, sizeof(g_plate_task_context.input_points));
  g_plate_task_context.input_count = 0U;
  g_plate_task_context.state.input_active = 0U;
}

/**
  * @brief  清空当前任务运行用的目标点序列缓存。
  * @param  None
  * @retval None
  */
static void PlateTaskFsm_ClearRunBuffer(void)
{
  memset(g_plate_task_context.run_points, 0, sizeof(g_plate_task_context.run_points));
  g_plate_task_context.run_point_count = 0U;
  g_plate_task_context.advanced2_started = 0U;
  g_plate_task_context.advanced2_completed_round_trip_count = 0U;
  g_plate_task_context.state.current_target_index = 0U;
  g_plate_task_context.state.current_target_id = 0U;
  g_plate_task_context.state.target_count = 0U;
}

/**
  * @brief  根据宏定义填充默认的九个目标点像素坐标表。
  * @param  None
  * @retval None
  */
static void PlateTaskFsm_LoadDefaultTargetPoints(void)
{
  PlateTask_ConfigTypeDef *p_config;

  p_config = &g_plate_task_context.config;

  p_config->target_points[1].x = PLATE_TASK_POINT1_X;
  p_config->target_points[1].y = PLATE_TASK_POINT1_Y;
  p_config->target_points[2].x = PLATE_TASK_POINT2_X;
  p_config->target_points[2].y = PLATE_TASK_POINT2_Y;
  p_config->target_points[3].x = PLATE_TASK_POINT3_X;
  p_config->target_points[3].y = PLATE_TASK_POINT3_Y;
  p_config->target_points[4].x = PLATE_TASK_POINT4_X;
  p_config->target_points[4].y = PLATE_TASK_POINT4_Y;
  p_config->target_points[5].x = PLATE_TASK_POINT5_X;
  p_config->target_points[5].y = PLATE_TASK_POINT5_Y;
  p_config->target_points[6].x = PLATE_TASK_POINT6_X;
  p_config->target_points[6].y = PLATE_TASK_POINT6_Y;
  p_config->target_points[7].x = PLATE_TASK_POINT7_X;
  p_config->target_points[7].y = PLATE_TASK_POINT7_Y;
  p_config->target_points[8].x = PLATE_TASK_POINT8_X;
  p_config->target_points[8].y = PLATE_TASK_POINT8_Y;
  p_config->target_points[9].x = PLATE_TASK_POINT9_X;
  p_config->target_points[9].y = PLATE_TASK_POINT9_Y;
}

/**
  * @brief  把状态机输出目标设置为默认调平参考点。
  * @param  None
  * @retval None
  */
static void PlateTaskFsm_SetLevelTarget(void)
{
  g_plate_task_context.state.active_target_x = g_plate_task_context.config.level_target_x;
  g_plate_task_context.state.active_target_y = g_plate_task_context.config.level_target_y;
  g_plate_task_context.state.current_target_id = 0U;
}

/**
  * @brief  按当前 current_target_id 把 active_target_x/y 同步为实际像素坐标。
  * @param  None
  * @retval None
  */
static void PlateTaskFsm_LoadActiveTargetFromCurrentId(void)
{
  uint8_t target_id;

  target_id = g_plate_task_context.state.current_target_id;
  if ((target_id == 0U) || (target_id > PLATE_TASK_TARGET_POINT_COUNT))
  {
    PlateTaskFsm_SetLevelTarget();
    return;
  }

  g_plate_task_context.state.active_target_x =
    g_plate_task_context.config.target_points[target_id].x;
  g_plate_task_context.state.active_target_y =
    g_plate_task_context.config.target_points[target_id].y;
}

/**
  * @brief  清空当前任务运行状态，但不修改默认调平参考点配置。
  * @param  None
  * @retval None
  */
static void PlateTaskFsm_ClearTaskRuntime(void)
{
  PlateTask_ModeTypeDef mode_backup;

  mode_backup = g_plate_task_context.state.mode;
  memset(&g_plate_task_context.state, 0, sizeof(g_plate_task_context.state));
  g_plate_task_context.state.mode = mode_backup;
  g_plate_task_context.state.run_state = PLATE_TASK_RUN_IDLE;
  PlateTaskFsm_ClearInputBuffer();
  g_plate_task_context.input_task_id = PLATE_TASK_ID_NONE;
  PlateTaskFsm_ClearRunBuffer();
}

/**
  * @brief  把串口屏目标点命令字节映射成 1~9 的目标点编号。
  * @param  command: 串口屏发送的单字节命令。
  * @retval 1~9 表示合法目标点编号，0 表示该字节不是目标点输入。
  */
static uint8_t PlateTaskFsm_CommandToTargetId(uint8_t command)
{
  switch (command)
  {
    case SERIAL_SCREEN_CMD_TARGET1: return 1U;
    case SERIAL_SCREEN_CMD_TARGET2: return 2U;
    case SERIAL_SCREEN_CMD_TARGET3: return 3U;
    case SERIAL_SCREEN_CMD_TARGET4: return 4U;
    case SERIAL_SCREEN_CMD_TARGET5: return 5U;
    case SERIAL_SCREEN_CMD_TARGET6: return 6U;
    case SERIAL_SCREEN_CMD_TARGET7: return 7U;
    case SERIAL_SCREEN_CMD_TARGET8: return 8U;
    case SERIAL_SCREEN_CMD_TARGET9: return 9U;
    default: return 0U;
  }
}

/**
  * @brief  根据任务号设置大类模式。
  * @param  task_id: 当前任务号。
  * @retval 返回该任务属于基础题还是发挥题模式。
  */
static PlateTask_ModeTypeDef PlateTaskFsm_TaskIdToMode(PlateTask_TaskIdTypeDef task_id)
{
  if ((task_id == PLATE_TASK_ID_BASIC1) || (task_id == PLATE_TASK_ID_BASIC2))
  {
    return PLATE_TASK_MODE_BASIC;
  }

  return PLATE_TASK_MODE_ADVANCED;
}

/**
  * @brief  用单个目标点立即启动一个任务。
  * @param  task_id: 任务号。
  * @param  target_id: 目标点编号，范围 1~9。
  * @retval None
  */
static void PlateTaskFsm_StartSingleTargetTask(PlateTask_TaskIdTypeDef task_id, uint8_t target_id)
{
  g_plate_task_context.state.mode = PlateTaskFsm_TaskIdToMode(task_id);
  g_plate_task_context.state.task_id = task_id;
  g_plate_task_context.state.run_state = PLATE_TASK_RUN_MOVING;
  g_plate_task_context.state.task_finished = 0U;
  g_plate_task_context.state.task_timeout = 0U;
  g_plate_task_context.state.task_start_tick_ms = SystemTime_Device_GetTickMs();
  g_plate_task_context.state.stay_start_tick_ms = 0U;
  g_plate_task_context.state.task_elapsed_ms = 0U;

  PlateTaskFsm_ClearRunBuffer();
  g_plate_task_context.run_points[0] = target_id;
  g_plate_task_context.run_point_count = 1U;
  g_plate_task_context.state.target_count = 1U;
  g_plate_task_context.state.current_target_index = 0U;
  g_plate_task_context.state.current_target_id = target_id;
  PlateTaskFsm_LoadActiveTargetFromCurrentId();
}

/**
  * @brief  进入等待串口屏输入目标点的状态。
  * @param  task_id: 当前准备配置的任务号。
  * @retval None
  */
static void PlateTaskFsm_StartWaitInputTask(PlateTask_TaskIdTypeDef task_id)
{
  g_plate_task_context.state.mode = PlateTaskFsm_TaskIdToMode(task_id);
  g_plate_task_context.state.task_id = task_id;
  g_plate_task_context.state.run_state = PLATE_TASK_RUN_WAIT_INPUT;
  g_plate_task_context.state.task_finished = 0U;
  g_plate_task_context.state.task_timeout = 0U;
  g_plate_task_context.state.task_start_tick_ms = 0U;
  g_plate_task_context.state.stay_start_tick_ms = 0U;
  g_plate_task_context.state.task_elapsed_ms = 0U;
  g_plate_task_context.state.current_target_index = 0U;
  g_plate_task_context.state.current_target_id = 0U;
  g_plate_task_context.state.target_count = 0U;

  PlateTaskFsm_ClearRunBuffer();
  PlateTaskFsm_ClearInputBuffer();
  g_plate_task_context.input_task_id = task_id;
  PlateTaskFsm_SetLevelTarget();
}

/**
  * @brief  启动“视觉直接给目标区域号”的发挥 0xB5 模式。
  * @param  None
  * @retval None
  */
static void PlateTaskFsm_StartVisionRegionTask(void)
{
  g_plate_task_context.state.mode = PLATE_TASK_MODE_ADVANCED;
  g_plate_task_context.state.task_id = PLATE_TASK_ID_ADVANCED5;
  g_plate_task_context.state.run_state = PLATE_TASK_RUN_WAIT_INPUT;
  g_plate_task_context.state.task_finished = 0U;
  g_plate_task_context.state.task_timeout = 0U;
  g_plate_task_context.state.task_start_tick_ms = 0U;
  g_plate_task_context.state.stay_start_tick_ms = 0U;
  g_plate_task_context.state.task_elapsed_ms = 0U;
  g_plate_task_context.state.current_target_index = 0U;
  g_plate_task_context.state.current_target_id = 0U;
  g_plate_task_context.state.target_count = 0U;

  PlateTaskFsm_ClearRunBuffer();
  PlateTaskFsm_ClearInputBuffer();
  g_plate_task_context.input_task_id = PLATE_TASK_ID_NONE;
  PlateTaskFsm_SetLevelTarget();
}

/**
  * @brief  根据输入缓存生成当前任务真正要执行的目标点序列。
  * @param  None
  * @retval None
  */
static void PlateTaskFsm_ApplyInputSequence(void)
{
  uint8_t index;

  g_plate_task_context.state.input_active = 0U;

  if (g_plate_task_context.input_task_id == PLATE_TASK_ID_NONE)
  {
    return;
  }

  /* 基础一只取第一个输入点，并稳定保持在该点。 */
  if (g_plate_task_context.input_task_id == PLATE_TASK_ID_BASIC1)
  {
    if (g_plate_task_context.input_count > 0U)
    {
      PlateTaskFsm_StartSingleTargetTask(PLATE_TASK_ID_BASIC1,
                                         g_plate_task_context.input_points[0]);
    }
    return;
  }

  /* 基础二默认去中心点 5；如果串口屏额外给了目标点，则使用输入的第一个点覆盖默认值。 */
  if (g_plate_task_context.input_task_id == PLATE_TASK_ID_BASIC2)
  {
    if (g_plate_task_context.input_count > 0U)
    {
      PlateTaskFsm_StartSingleTargetTask(PLATE_TASK_ID_BASIC2,
                                         g_plate_task_context.input_points[0]);
    }
    else
    {
      PlateTaskFsm_StartSingleTargetTask(PLATE_TASK_ID_BASIC2, 5U);
    }
    return;
  }

  /* 发挥二要求两个目标点循环往复，因此少于两个点时不启动任务。 */
  if (g_plate_task_context.input_task_id == PLATE_TASK_ID_ADVANCED2)
  {
    if (g_plate_task_context.input_count < 2U)
    {
      g_plate_task_context.state.run_state = PLATE_TASK_RUN_WAIT_INPUT;
      g_plate_task_context.state.current_target_id = 0U;
      PlateTaskFsm_SetLevelTarget();
      return;
    }

    PlateTaskFsm_ClearRunBuffer();
    g_plate_task_context.run_points[0] = g_plate_task_context.input_points[0];
    g_plate_task_context.run_points[1] = g_plate_task_context.input_points[1];
    g_plate_task_context.run_point_count = 2U;
    g_plate_task_context.state.mode = PLATE_TASK_MODE_ADVANCED;
    g_plate_task_context.state.task_id = PLATE_TASK_ID_ADVANCED2;
    g_plate_task_context.state.run_state = PLATE_TASK_RUN_MOVING;
    g_plate_task_context.state.current_target_index = 0U;
    g_plate_task_context.state.current_target_id = g_plate_task_context.run_points[0];
    g_plate_task_context.state.target_count = 2U;
    g_plate_task_context.state.task_finished = 0U;
    g_plate_task_context.state.task_timeout = 0U;
    g_plate_task_context.state.task_start_tick_ms = SystemTime_Device_GetTickMs();
    g_plate_task_context.state.stay_start_tick_ms = 0U;
    g_plate_task_context.state.task_elapsed_ms = 0U;
    PlateTaskFsm_LoadActiveTargetFromCurrentId();
    return;
  }

  /* 发挥一 / 发挥三 / 发挥四都按“变长点列”处理。
     发挥三当前先按发挥一的顺序点列逻辑实现，但保留独立任务号，后续可单独扩展。 */
  if ((g_plate_task_context.input_task_id == PLATE_TASK_ID_ADVANCED1) ||
      (g_plate_task_context.input_task_id == PLATE_TASK_ID_ADVANCED3) ||
      (g_plate_task_context.input_task_id == PLATE_TASK_ID_ADVANCED4))
  {
    if (g_plate_task_context.input_count == 0U)
    {
      g_plate_task_context.state.run_state = PLATE_TASK_RUN_WAIT_INPUT;
      g_plate_task_context.state.current_target_id = 0U;
      PlateTaskFsm_SetLevelTarget();
      return;
    }

    PlateTaskFsm_ClearRunBuffer();
    for (index = 0U; index < g_plate_task_context.input_count; index++)
    {
      g_plate_task_context.run_points[index] = g_plate_task_context.input_points[index];
    }

    g_plate_task_context.run_point_count = g_plate_task_context.input_count;
    g_plate_task_context.state.mode = PLATE_TASK_MODE_ADVANCED;
    g_plate_task_context.state.task_id = g_plate_task_context.input_task_id;
    g_plate_task_context.state.run_state = PLATE_TASK_RUN_MOVING;
    g_plate_task_context.state.current_target_index = 0U;
    g_plate_task_context.state.current_target_id = g_plate_task_context.run_points[0];
    g_plate_task_context.state.target_count = g_plate_task_context.run_point_count;
    g_plate_task_context.state.task_finished = 0U;
    g_plate_task_context.state.task_timeout = 0U;
    g_plate_task_context.state.task_start_tick_ms = SystemTime_Device_GetTickMs();
    g_plate_task_context.state.stay_start_tick_ms = 0U;
    g_plate_task_context.state.task_elapsed_ms = 0U;
    PlateTaskFsm_LoadActiveTargetFromCurrentId();
  }
}

/**
  * @brief  判断当前点是否进入指定目标半径内。
  * @param  current_x: 当前视觉检测到的 X 坐标。
  * @param  current_y: 当前视觉检测到的 Y 坐标。
  * @param  radius_px: 本次判定使用的像素半径。
  * @retval 1 表示已经进入指定目标半径，0 表示尚未进入。
  */
static uint8_t PlateTaskFsm_IsInsideRadius(uint16_t current_x,
                                           uint16_t current_y,
                                           uint16_t radius_px)
{
  uint32_t dx;
  uint32_t dy;
  uint32_t radius;

  dx = (current_x >= g_plate_task_context.state.active_target_x) ?
       ((uint32_t)current_x - (uint32_t)g_plate_task_context.state.active_target_x) :
       ((uint32_t)g_plate_task_context.state.active_target_x - (uint32_t)current_x);

  dy = (current_y >= g_plate_task_context.state.active_target_y) ?
       ((uint32_t)current_y - (uint32_t)g_plate_task_context.state.active_target_y) :
       ((uint32_t)g_plate_task_context.state.active_target_y - (uint32_t)current_y);

  radius = radius_px;

  return (((dx * dx) + (dy * dy)) <= (radius * radius)) ? 1U : 0U;
}

/**
  * @brief  获取当前任务使用的“进入目标点”判定半径。
  * @param  None
  * @retval 返回当前任务对应的到达判定半径，单位为像素。
  */
static uint16_t PlateTaskFsm_GetCurrentArriveRadiusPx(void)
{
  if (g_plate_task_context.state.task_id == PLATE_TASK_ID_BASIC2)
  {
    /* 基础二只有一个最终中心点目标。
       这里单独把到达半径再收紧一点，避免还停在目标边缘就提前判完成。 */
    return PLATE_TASK_BASIC2_ARRIVE_RADIUS_PX;
  }

  if ((g_plate_task_context.state.task_id == PLATE_TASK_ID_ADVANCED1) ||
      (g_plate_task_context.state.task_id == PLATE_TASK_ID_ADVANCED2) ||
      (g_plate_task_context.state.task_id == PLATE_TASK_ID_ADVANCED5))
  {
    return PLATE_TASK_ADVANCED_ARRIVE_RADIUS_PX;
  }

  if ((g_plate_task_context.state.task_id == PLATE_TASK_ID_ADVANCED3) ||
      (g_plate_task_context.state.task_id == PLATE_TASK_ID_ADVANCED4))
  {
    if ((g_plate_task_context.state.current_target_id == 4U) &&
        ((g_plate_task_context.state.current_target_index + 1U) <
         g_plate_task_context.run_point_count))
    {
      /* 发挥四经过 4 号点时，现场观察容易在附近慢慢停住但始终蹭不进判定圆。
         因此只对“4 号点作为中间经过点”的情况单独放大一点半径，
         让它更容易判定为已经经过；如果 4 号点是最终点，则仍走普通发挥四半径。 */
      return PLATE_TASK_ADVANCED4_POINT4_PASS_RADIUS_PX;
    }

    /* 发挥四中间点只要求“经过”，没有停留时间。
       因此经过判定必须比普通发挥题更严格，否则球只擦到目标圆边缘就会切到下一个点。 */
    return PLATE_TASK_ADVANCED4_PASS_RADIUS_PX;
  }

  return g_plate_task_context.config.arrive_radius_px;
}

/**
  * @brief  获取当前任务使用的“保持停留”判定半径。
  * @param  None
  * @retval 返回当前任务对应的保持半径，单位为像素。
  */
static uint16_t PlateTaskFsm_GetCurrentStayRadiusPx(void)
{
  if ((g_plate_task_context.state.task_id == PLATE_TASK_ID_ADVANCED1) ||
      (g_plate_task_context.state.task_id == PLATE_TASK_ID_ADVANCED2) ||
      (g_plate_task_context.state.task_id == PLATE_TASK_ID_ADVANCED3) ||
      (g_plate_task_context.state.task_id == PLATE_TASK_ID_ADVANCED4) ||
      (g_plate_task_context.state.task_id == PLATE_TASK_ID_ADVANCED5))
  {
    return PLATE_TASK_ADVANCED_STAY_RADIUS_PX;
  }

  return g_plate_task_context.config.stay_radius_px;
}

/**
  * @brief  切换到当前序列中的下一个目标点。
  * @param  None
  * @retval 1 表示成功切换到下一个点，0 表示当前已经没有下一个点。
  */
static uint8_t PlateTaskFsm_MoveToNextPoint(void)
{
  if ((g_plate_task_context.state.current_target_index + 1U) >=
      g_plate_task_context.run_point_count)
  {
    return 0U;
  }

  g_plate_task_context.state.current_target_index++;
  g_plate_task_context.state.current_target_id =
    g_plate_task_context.run_points[g_plate_task_context.state.current_target_index];
  g_plate_task_context.state.stay_start_tick_ms = 0U;
  g_plate_task_context.state.run_state = PLATE_TASK_RUN_MOVING;
  PlateTaskFsm_LoadActiveTargetFromCurrentId();
  return 1U;
}

/**
  * @brief  让发挥二在两个目标点之间循环切换。
  * @param  None
  * @retval None
  */
static void PlateTaskFsm_ToggleAdvanced2Target(void)
{
  if (g_plate_task_context.state.current_target_index == 0U)
  {
    g_plate_task_context.state.current_target_index = 1U;
  }
  else
  {
    g_plate_task_context.state.current_target_index = 0U;
  }

  g_plate_task_context.state.current_target_id =
    g_plate_task_context.run_points[g_plate_task_context.state.current_target_index];
  g_plate_task_context.state.stay_start_tick_ms = 0U;
  g_plate_task_context.state.run_state = PLATE_TASK_RUN_MOVING;
  PlateTaskFsm_LoadActiveTargetFromCurrentId();
}

/**
  * @brief  处理发挥二在单个目标点上完成停留后的状态推进。
  * @param  now_ms: 当前系统毫秒节拍，用于在任务完成时锁存最终耗时。
  * @retval None
  *
  * @note   发挥二的运行规则在这里统一收口：
  *         1. 第 1 个输入点视为“起点/终点”。
  *         2. 首次在起点停留达标后，不计入往返次数，只是正式发车去第 2 个点。
  *         3. 之后每次从第 2 个点回到第 1 个点并停留达标，记作完成 1 次往返。
  *         4. 完成设定次数后，任务直接结束并保持在第 1 个输入点，不再继续循环。
  */
static void PlateTaskFsm_HandleAdvanced2StayComplete(uint32_t now_ms)
{
  if (g_plate_task_context.state.current_target_index == 0U)
  {
    if (g_plate_task_context.advanced2_started == 0U)
    {
      g_plate_task_context.advanced2_started = 1U;
      PlateTaskFsm_ToggleAdvanced2Target();
      return;
    }

    g_plate_task_context.advanced2_completed_round_trip_count++;
    if (g_plate_task_context.advanced2_completed_round_trip_count >=
        PLATE_TASK_ADVANCED2_ROUND_TRIP_COUNT)
    {
      g_plate_task_context.state.task_finished = 1U;
      if (g_plate_task_context.state.task_start_tick_ms != 0U)
      {
        g_plate_task_context.state.task_elapsed_ms =
          now_ms - g_plate_task_context.state.task_start_tick_ms;
      }
      g_plate_task_context.state.run_state = PLATE_TASK_RUN_HOLDING;
      return;
    }

    PlateTaskFsm_ToggleAdvanced2Target();
    return;
  }

  PlateTaskFsm_ToggleAdvanced2Target();
}

/**
  * @brief  获取题目状态机配置表，便于外部直接修改九点表和判定参数。
  * @param  None
  * @retval 返回配置结构体指针。
  */
PlateTask_ConfigTypeDef *PlateTaskFsm_GetConfig(void)
{
  return &g_plate_task_context.config;
}

/**
  * @brief  初始化题目状态机、九点目标表和默认调平参考点。
  * @param  level_target_x: 默认调平参考点 X 坐标。
  * @param  level_target_y: 默认调平参考点 Y 坐标。
  * @retval None
  */
void PlateTaskFsm_Init(uint16_t level_target_x, uint16_t level_target_y)
{
  memset(&g_plate_task_context, 0, sizeof(g_plate_task_context));

  g_plate_task_context.config.arrive_radius_px = PLATE_TASK_DEFAULT_ARRIVE_RADIUS_PX;
  g_plate_task_context.config.stay_radius_px = PLATE_TASK_DEFAULT_STAY_RADIUS_PX;
  g_plate_task_context.config.level_target_x = level_target_x;
  g_plate_task_context.config.level_target_y = level_target_y;
  g_plate_task_context.config.advanced_stay_ms = PLATE_TASK_ADVANCED_STAY_MS;
  g_plate_task_context.config.advanced_loop_stay_ms = PLATE_TASK_ADVANCED_LOOP_STAY_MS;
  g_plate_task_context.config.advanced4_final_stay_ms = PLATE_TASK_ADVANCED4_FINAL_STAY_MS;

  PlateTaskFsm_LoadDefaultTargetPoints();

  g_plate_task_context.state.mode = PLATE_TASK_MODE_LEVEL;
  g_plate_task_context.state.task_id = PLATE_TASK_ID_NONE;
  g_plate_task_context.state.run_state = PLATE_TASK_RUN_IDLE;
  PlateTaskFsm_SetLevelTarget();
}

/**
  * @brief  主动请求进入默认调平模式，并清除已缓存的任务输入。
  * @param  None
  * @retval None
  */
void PlateTaskFsm_RequestLevelReset(void)
{
  g_plate_task_context.state.mode = PLATE_TASK_MODE_LEVEL;
  PlateTaskFsm_ClearTaskRuntime();
  PlateTaskFsm_SetLevelTarget();
}

/**
  * @brief  处理来自串口屏的单字节协议输入。
  * @param  command: 串口屏发送的单字节命令。
  * @retval 1 表示该字节已被状态机接收，0 表示不属于当前状态机协议。
  */
uint8_t PlateTaskFsm_HandleCommand(uint8_t command)
{
  uint8_t target_id;

  g_plate_task_context.state.last_command = command;

  if (command == SERIAL_SCREEN_CMD_BASIC1)
  {
    PlateTaskFsm_StartWaitInputTask(PLATE_TASK_ID_BASIC1);
    return 1U;
  }

  if (command == SERIAL_SCREEN_CMD_BASIC2)
  {
    PlateTaskFsm_StartSingleTargetTask(PLATE_TASK_ID_BASIC2, 5U);
    g_plate_task_context.input_task_id = PLATE_TASK_ID_BASIC2;
    PlateTaskFsm_ClearInputBuffer();
    return 1U;
  }

  if (command == SERIAL_SCREEN_CMD_ADVANCED1)
  {
    PlateTaskFsm_StartWaitInputTask(PLATE_TASK_ID_ADVANCED1);
    return 1U;
  }

  if (command == SERIAL_SCREEN_CMD_ADVANCED2)
  {
    PlateTaskFsm_StartWaitInputTask(PLATE_TASK_ID_ADVANCED2);
    return 1U;
  }

  if (command == SERIAL_SCREEN_CMD_ADVANCED3)
  {
    PlateTaskFsm_StartWaitInputTask(PLATE_TASK_ID_ADVANCED3);
    return 1U;
  }

  if (command == SERIAL_SCREEN_CMD_ADVANCED4)
  {
    PlateTaskFsm_StartWaitInputTask(PLATE_TASK_ID_ADVANCED4);
    return 1U;
  }

  if (command == SERIAL_SCREEN_CMD_ADVANCED5)
  {
    PlateTaskFsm_StartVisionRegionTask();
    return 1U;
  }

  if (command == SERIAL_SCREEN_CMD_INPUT_BEGIN)
  {
    if (g_plate_task_context.state.task_id == PLATE_TASK_ID_NONE)
    {
      return 0U;
    }

    PlateTaskFsm_ClearInputBuffer();
    g_plate_task_context.state.input_active = 1U;

    /* 只有明确依赖输入的任务，在开始输入时才切到等待输入状态。
       基础二已经有默认目标 5，因此允许边运行边输入覆盖。 */
    if (g_plate_task_context.state.task_id != PLATE_TASK_ID_BASIC2)
    {
      g_plate_task_context.state.run_state = PLATE_TASK_RUN_WAIT_INPUT;
      g_plate_task_context.state.current_target_id = 0U;
      g_plate_task_context.state.target_count = 0U;
      PlateTaskFsm_SetLevelTarget();
    }

    return 1U;
  }

  target_id = PlateTaskFsm_CommandToTargetId(command);
  if ((target_id != 0U) && (g_plate_task_context.state.input_active != 0U))
  {
    if (g_plate_task_context.input_count < PLATE_TASK_MAX_INPUT_POINT_COUNT)
    {
      g_plate_task_context.input_points[g_plate_task_context.input_count] = target_id;
      g_plate_task_context.input_count++;
    }
    return 1U;
  }

  if (command == SERIAL_SCREEN_CMD_INPUT_END)
  {
    if (g_plate_task_context.state.input_active == 0U)
    {
      return 0U;
    }

    PlateTaskFsm_ApplyInputSequence();
    return 1U;
  }

  return 0U;
}

/**
  * @brief  按当前任务模式更新目标点选择和题目推进状态。
  * @param  now_ms: 当前系统毫秒节拍。
  * @param  current_x: 当前视觉检测到的 X 坐标。
  * @param  current_y: 当前视觉检测到的 Y 坐标。
  * @param  external_target_id: 外部输入的目标点编号。
  * @param  external_target_valid: 外部目标点编号是否有效。
  * @param  vision_valid: 当前视觉数据是否有效。
  * @retval None
  */
void PlateTaskFsm_Update(uint32_t now_ms,
                         uint16_t current_x,
                         uint16_t current_y,
                         uint8_t external_target_id,
                         uint8_t external_target_valid,
                         uint8_t vision_valid)
{
  if (g_plate_task_context.state.mode == PLATE_TASK_MODE_LEVEL)
  {
    PlateTaskFsm_SetLevelTarget();
    g_plate_task_context.state.run_state = PLATE_TASK_RUN_IDLE;
    return;
  }

  if (g_plate_task_context.state.task_id == PLATE_TASK_ID_ADVANCED5)
  {
    if ((external_target_valid != 0U) &&
        (external_target_id >= 1U) &&
        (external_target_id <= PLATE_TASK_TARGET_POINT_COUNT) &&
        (external_target_id != g_plate_task_context.state.current_target_id))
    {
      g_plate_task_context.state.current_target_id = external_target_id;
      g_plate_task_context.state.current_target_index = 0U;
      g_plate_task_context.state.target_count = 1U;
      g_plate_task_context.state.task_finished = 0U;
      g_plate_task_context.state.task_timeout = 0U;
      g_plate_task_context.state.stay_start_tick_ms = 0U;
      g_plate_task_context.state.run_state = PLATE_TASK_RUN_MOVING;
      PlateTaskFsm_LoadActiveTargetFromCurrentId();
    }
  }

  PlateTaskFsm_LoadActiveTargetFromCurrentId();

  if ((g_plate_task_context.state.task_start_tick_ms != 0U) &&
      (g_plate_task_context.state.task_finished == 0U) &&
      (now_ms >= g_plate_task_context.state.task_start_tick_ms))
  {
    g_plate_task_context.state.task_elapsed_ms =
      now_ms - g_plate_task_context.state.task_start_tick_ms;
  }

  /* 除 0xB5“视觉持续下发目标点”模式外，其余题目一旦判定完成，
     后续控制拍就直接保持在 HOLDING，不能再继续往下走状态推进逻辑。
     否则像发挥二这种循环题，虽然已经置位 task_finished，
     下一拍仍会重新进入停留分支，导致又被切回下一个目标点。 */
  if ((g_plate_task_context.state.task_finished != 0U) &&
      (g_plate_task_context.state.task_id != PLATE_TASK_ID_ADVANCED5))
  {
    g_plate_task_context.state.run_state = PLATE_TASK_RUN_HOLDING;
    return;
  }

  if (g_plate_task_context.state.current_target_id == 0U)
  {
    if (g_plate_task_context.state.task_id == PLATE_TASK_ID_ADVANCED5)
    {
      g_plate_task_context.state.run_state = PLATE_TASK_RUN_WAIT_INPUT;
    }
    return;
  }

  if (vision_valid == 0U)
  {
    return;
  }

  if ((g_plate_task_context.state.task_id == PLATE_TASK_ID_BASIC1) ||
      (g_plate_task_context.state.task_id == PLATE_TASK_ID_BASIC2))
  {
    if (PlateTaskFsm_IsInsideRadius(current_x,
                                    current_y,
                                    PlateTaskFsm_GetCurrentArriveRadiusPx()) != 0U)
    {
      g_plate_task_context.state.task_finished = 1U;
      g_plate_task_context.state.run_state = PLATE_TASK_RUN_HOLDING;
    }
    else
    {
      g_plate_task_context.state.run_state = PLATE_TASK_RUN_MOVING;
    }

    return;
  }

  if (g_plate_task_context.state.task_id == PLATE_TASK_ID_ADVANCED5)
  {
    if (PlateTaskFsm_IsInsideRadius(current_x,
                                    current_y,
                                    PlateTaskFsm_GetCurrentArriveRadiusPx()) != 0U)
    {
      g_plate_task_context.state.run_state = PLATE_TASK_RUN_HOLDING;
    }
    else
    {
      g_plate_task_context.state.run_state = PLATE_TASK_RUN_MOVING;
    }

    return;
  }

  if (g_plate_task_context.state.run_state == PLATE_TASK_RUN_STAYING)
  {
    /* 已经进入目标点后，继续停留计时时改用“保持半径”。
       发挥题这里会自动切到比普通任务更严格的半径，
       避免球只是擦到目标边缘就开始计时，结果还没站稳就切下一个点。 */
    if (PlateTaskFsm_IsInsideRadius(current_x,
                                    current_y,
                                    PlateTaskFsm_GetCurrentStayRadiusPx()) == 0U)
    {
      g_plate_task_context.state.run_state = PLATE_TASK_RUN_MOVING;
      g_plate_task_context.state.stay_start_tick_ms = 0U;
      return;
    }
  }
  else if (PlateTaskFsm_IsInsideRadius(current_x,
                                       current_y,
                                       PlateTaskFsm_GetCurrentArriveRadiusPx()) == 0U)
  {
    g_plate_task_context.state.run_state = PLATE_TASK_RUN_MOVING;
    g_plate_task_context.state.stay_start_tick_ms = 0U;
    return;
  }

  /* 发挥四在中间点只要求“进入一次即可”，不需要中间停留。 */
  if ((g_plate_task_context.state.task_id == PLATE_TASK_ID_ADVANCED3) ||
      (g_plate_task_context.state.task_id == PLATE_TASK_ID_ADVANCED4))
  {
    if ((g_plate_task_context.state.current_target_index + 1U) <
        g_plate_task_context.run_point_count)
    {
      (void)PlateTaskFsm_MoveToNextPoint();
      return;
    }

    /* 发挥四最后一个点不再额外计时。
       只要已经进入最终目标区域，就直接切到 HOLDING 持续保持；
       这样中途点“路过即走”，最后一点“进入即停住”。 */
    g_plate_task_context.state.run_state = PLATE_TASK_RUN_STAYING;
    if (g_plate_task_context.state.stay_start_tick_ms == 0U)
    {
      g_plate_task_context.state.stay_start_tick_ms = now_ms;
      return;
    }

    if ((now_ms - g_plate_task_context.state.stay_start_tick_ms) <
        g_plate_task_context.config.advanced4_final_stay_ms)
    {
      return;
    }

    g_plate_task_context.state.task_finished = 1U;
    if (g_plate_task_context.state.task_start_tick_ms != 0U)
    {
      g_plate_task_context.state.task_elapsed_ms =
        now_ms - g_plate_task_context.state.task_start_tick_ms;
    }
    g_plate_task_context.state.run_state = PLATE_TASK_RUN_HOLDING;
    return;
  }

  /* 发挥一 / 发挥三 / 发挥二都需要在目标点上停留一定时间。 */
  g_plate_task_context.state.run_state = PLATE_TASK_RUN_STAYING;
  if (g_plate_task_context.state.stay_start_tick_ms == 0U)
  {
    g_plate_task_context.state.stay_start_tick_ms = now_ms;
  }

  if (g_plate_task_context.state.task_id == PLATE_TASK_ID_ADVANCED2)
  {
    if ((now_ms - g_plate_task_context.state.stay_start_tick_ms) <
        g_plate_task_context.config.advanced_loop_stay_ms)
    {
      return;
    }

    PlateTaskFsm_HandleAdvanced2StayComplete(now_ms);
    return;
  }

  if ((now_ms - g_plate_task_context.state.stay_start_tick_ms) <
      g_plate_task_context.config.advanced_stay_ms)
  {
    return;
  }

  g_plate_task_context.state.stay_start_tick_ms = 0U;
  if (PlateTaskFsm_MoveToNextPoint() == 0U)
  {
    g_plate_task_context.state.task_finished = 1U;
    if (g_plate_task_context.state.task_start_tick_ms != 0U)
    {
      g_plate_task_context.state.task_elapsed_ms =
        now_ms - g_plate_task_context.state.task_start_tick_ms;
    }
    g_plate_task_context.state.run_state = PLATE_TASK_RUN_HOLDING;
  }
}

/**
  * @brief  判断当前模式是否允许控制层继续跑闭环控制。
  * @param  None
  * @retval 1 表示允许控制，0 表示应直接回默认平衡状态。
  */
uint8_t PlateTaskFsm_IsControlEnabled(void)
{
  if (g_plate_task_context.state.mode == PLATE_TASK_MODE_LEVEL)
  {
    return 0U;
  }

  if (g_plate_task_context.state.current_target_id == 0U)
  {
    return 0U;
  }

  return 1U;
}

/**
  * @brief  获取当前题目状态机运行状态。
  * @param  None
  * @retval 返回状态机只读状态指针。
  */
const PlateTask_StateTypeDef *PlateTaskFsm_GetState(void)
{
  return &g_plate_task_context.state;
}

