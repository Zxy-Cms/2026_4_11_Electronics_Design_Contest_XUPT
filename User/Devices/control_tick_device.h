#ifndef __CONTROL_TICK_DEVICE_H__
#define __CONTROL_TICK_DEVICE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_common.h"

/* TIM11 控制节拍回调函数类型。
   Devices 层只负责触发固定周期回调，不关心控制层具体实现。 */
typedef void (*ControlTick_DeviceCallbackTypeDef)(void);

/**
  * @brief  初始化 TIM11 控制节拍设备并启动周期中断。
  * @param  callback: 每到 1 个 TIM11 周期时要调用的回调函数。
  * @retval BSP_OK 表示启动成功，BSP_ERROR 表示启动失败或回调为空。
  */
BSP_StatusTypeDef ControlTick_Device_Init(ControlTick_DeviceCallbackTypeDef callback);

#ifdef __cplusplus
}
#endif

#endif /* __CONTROL_TICK_DEVICE_H__ */
