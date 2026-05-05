#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* PID 参数结构体, 方便在控制层集中调参. */
typedef struct
{
  float kp;
  float ki;
  float kd;
  float integral_limit;
  float output_limit;
  float deadband;
} PID_ParamsTypeDef;

/* PID 运行状态结构体. */
typedef struct
{
  float integral;
  float prev_error;
  float prev_output;
  uint8_t initialized;
} PID_StateTypeDef;

/* PID 对象, 把参数和运行状态封装在一起. */
typedef struct
{
  PID_ParamsTypeDef params;
  PID_StateTypeDef state;
} PID_TypeDef;

/* 初始化 PID 对象, 同时清空内部状态. */
void PID_Init(PID_TypeDef *p_pid, const PID_ParamsTypeDef *p_params);

/* 运行中更新 PID 参数, 不主动清空状态. */
void PID_SetParams(PID_TypeDef *p_pid, const PID_ParamsTypeDef *p_params);

/* 清空积分项和历史误差. */
void PID_Reset(PID_TypeDef *p_pid);

/* 执行一次 PID 计算. */
float PID_Update(PID_TypeDef *p_pid, float target, float measurement, float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */
