#include "system_time_device.h"

#include "stm32f4xx_hal.h"

/* 当前直接复用 HAL 的系统节拍. */
uint32_t SystemTime_Device_GetTickMs(void)
{
  return HAL_GetTick();
}
