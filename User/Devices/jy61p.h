#ifndef __JY61P_H
#define __JY61P_H

#include "bsp_common.h"

/**
  * @brief  初始化 JY61P 设备层并启动串口接收。
  * @param  None
  * @retval BSP_OK 表示初始化成功，BSP_ERROR 表示初始化失败。
  */
BSP_StatusTypeDef JY61P_Init(void);

/**
  * @brief  JY61P 设备层保留的周期任务接口。
  * @param  None
  * @retval None
  */
void JY61P_Task(void);

/**
  * @brief  UART4 接收完成后的设备层处理函数。
  * @param  None
  * @retval None
  */
void JY61P_OnRxComplete(void);

/**
  * @brief  UART4 出错后的恢复处理函数。
  * @param  None
  * @retval None
  */
void JY61P_OnError(void);

/**
  * @brief  读取一次线程安全的欧拉角快照。
  * @param  p_roll: 输出横滚角指针，可传 NULL。
  * @param  p_pitch: 输出俯仰角指针，可传 NULL。
  * @param  p_yaw: 输出航向角指针，可传 NULL。
  * @retval None
  */
void JY61P_GetEuler(float *p_roll, float *p_pitch, float *p_yaw);

extern volatile float Roll;
extern volatile float Pitch;
extern volatile float Yaw;

#endif
