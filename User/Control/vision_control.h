#ifndef __VISION_CONTROL_H__
#define __VISION_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_common.h"
#include "plate_task_fsm.h"
#include "vision_axis.h"
#include "vision_frame.h"

/* 整体控制配置。
   X/Y 两个轴各自使用一套双环参数。 */
typedef struct
{
  uint16_t center_x;
  uint16_t center_y;
  uint32_t frame_timeout_ms;
  VisionAxis_ConfigTypeDef x_axis;
  VisionAxis_ConfigTypeDef y_axis;
} VisionControl_ConfigTypeDef;

/* 控制层对外状态，便于调试和上位机观测。 */
typedef struct
{
  VisionFrame_TypeDef frame;
  VisionAxis_StateTypeDef x_axis;
  VisionAxis_StateTypeDef y_axis;
  uint16_t active_target_x;
  uint16_t active_target_y;
  uint8_t target_point_id;
  uint8_t vision_online;
  uint8_t control_enabled;
  uint8_t task_id;
  uint8_t task_finished;
  uint8_t task_timeout;
  PlateTask_ModeTypeDef mode;
  PlateTask_RunStateTypeDef task_state;
  uint32_t task_elapsed_ms;
  uint32_t control_tick_count;
  uint8_t control_tick_pending;
} VisionControl_StateTypeDef;

/**
  * @brief  获取顶层控制器配置结构体，便于外部调参。
  * @param  None
  * @retval 指向全局控制配置的指针。
  */
VisionControl_ConfigTypeDef *VisionControl_GetConfig(void);

/**
  * @brief  初始化顶层控制器及其依赖模块。
  * @param  None
  * @retval BSP_OK 表示初始化成功，BSP_ERROR 表示初始化失败。
  */
BSP_StatusTypeDef VisionControl_Init(void);

/**
  * @brief  顶层主循环任务，负责消费输入并执行待处理的控制拍。
  * @param  None
  * @retval None
  */
void VisionControl_Task(void);

/**
  * @brief  由 Devices 层在 TIM11 节拍到达时通知控制层新增 1 个控制拍。
  * @param  None
  * @retval None
  */
void VisionControl_OnControlTick(void);

/**
  * @brief  更新控制层中心参考点。
  * @param  center_x: 新的中心参考点 X 坐标。
  * @param  center_y: 新的中心参考点 Y 坐标。
  * @retval None
  */
void VisionControl_SetCenter(uint16_t center_x, uint16_t center_y);

/**
  * @brief  获取当前控制器运行状态。
  * @param  None
  * @retval 指向控制器状态结构体的常量指针。
  */
const VisionControl_StateTypeDef *VisionControl_GetState(void);

/**
  * @brief  主动请求进入默认调平模式，并清空串口屏与视觉端已缓存的输入。
  * @param  None
  * @retval None
  */
void VisionControl_RequestLevelReset(void);

#ifdef __cplusplus
}
#endif

#endif /* __VISION_CONTROL_H__ */

