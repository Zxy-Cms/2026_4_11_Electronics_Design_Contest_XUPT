#ifndef __USB_SERVO_ANGLE_H__
#define __USB_SERVO_ANGLE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_common.h"

BSP_StatusTypeDef UsbServoAngle_Init(void);
void UsbServoAngle_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __USB_SERVO_ANGLE_H__ */
