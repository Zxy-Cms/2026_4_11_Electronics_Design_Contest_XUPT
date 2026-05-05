#include "control_tick_device.h"

#include "tim.h"

/* TIM11 设备层内部只保存一个回调指针。
   这样 HAL 中断和 Control 层之间通过函数指针解耦。 */
static ControlTick_DeviceCallbackTypeDef g_control_tick_callback = (ControlTick_DeviceCallbackTypeDef)0;

/**
  * @brief  初始化 TIM11 控制节拍设备并启动周期中断。
  * @param  callback: 每到 1 个 TIM11 周期时要调用的回调函数。
  * @retval BSP_OK 表示启动成功，BSP_ERROR 表示启动失败或回调为空。
  */
BSP_StatusTypeDef ControlTick_Device_Init(ControlTick_DeviceCallbackTypeDef callback)
{
  if (callback == (ControlTick_DeviceCallbackTypeDef)0)
  {
    return BSP_ERROR;
  }

  g_control_tick_callback = callback;

  if (HAL_TIM_Base_Start_IT(&htim11) != HAL_OK)
  {
    return BSP_ERROR;
  }

  return BSP_OK;
}

/**
  * @brief  HAL 定时器溢出统一回调中的 TIM11 分发处理。
  * @param  htim: 触发回调的 HAL 定时器句柄。
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if ((htim == (TIM_HandleTypeDef *)0) || (htim->Instance != TIM11))
  {
    return;
  }

  if (g_control_tick_callback != (ControlTick_DeviceCallbackTypeDef)0)
  {
    g_control_tick_callback();
  }
}
