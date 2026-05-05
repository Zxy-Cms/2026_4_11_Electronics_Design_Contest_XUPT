#ifndef __SYSTEM_TIME_DEVICE_H__
#define __SYSTEM_TIME_DEVICE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 设备层时间接口, 供控制层统一获取系统毫秒节拍. */
uint32_t SystemTime_Device_GetTickMs(void);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_TIME_DEVICE_H__ */
