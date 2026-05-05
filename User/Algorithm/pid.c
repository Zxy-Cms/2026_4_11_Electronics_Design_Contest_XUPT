#include "pid.h"

/* 取绝对值, 避免依赖标准库浮点函数. */
static float PID_Abs(float value)
{
  if (value < 0.0f)
  {
    return -value;
  }

  return value;
}

/* 通用限幅函数, 输出和积分都复用这里. */
static float PID_Clamp(float value, float limit)
{
  if (limit <= 0.0f)
  {
    return value;
  }

  if (value > limit)
  {
    return limit;
  }

  if (value < -limit)
  {
    return -limit;
  }

  return value;
}

void PID_SetParams(PID_TypeDef *p_pid, const PID_ParamsTypeDef *p_params)
{
  if ((p_pid == (PID_TypeDef *)0) || (p_params == (const PID_ParamsTypeDef *)0))
  {
    return;
  }

  p_pid->params = *p_params;
}

void PID_Init(PID_TypeDef *p_pid, const PID_ParamsTypeDef *p_params)
{
  if (p_pid == (PID_TypeDef *)0)
  {
    return;
  }

  if (p_params != (const PID_ParamsTypeDef *)0)
  {
    PID_SetParams(p_pid, p_params);
  }

  PID_Reset(p_pid);
}

void PID_Reset(PID_TypeDef *p_pid)
{
  if (p_pid == (PID_TypeDef *)0)
  {
    return;
  }

  p_pid->state.integral = 0.0f;
  p_pid->state.prev_error = 0.0f;
  p_pid->state.prev_output = 0.0f;
  p_pid->state.initialized = 0U;
}

float PID_Update(PID_TypeDef *p_pid, float target, float measurement, float dt_s)
{
  float error;
  float derivative;
  float output;

  if ((p_pid == (PID_TypeDef *)0) || (dt_s <= 0.0f))
  {
    return 0.0f;
  }

  error = target - measurement;

  /* 误差落入死区时直接清积分, 让机构更稳. */
  if ((p_pid->params.deadband > 0.0f) && (PID_Abs(error) <= p_pid->params.deadband))
  {
    error = 0.0f;
    p_pid->state.integral = 0.0f;
  }
  else
  {
    p_pid->state.integral += error * dt_s;
    p_pid->state.integral =
      PID_Clamp(p_pid->state.integral, p_pid->params.integral_limit);
  }

  /* 首次计算不做微分, 避免上电时出现尖峰. */
  if (p_pid->state.initialized == 0U)
  {
    derivative = 0.0f;
    p_pid->state.initialized = 1U;
  }
  else
  {
    derivative = (error - p_pid->state.prev_error) / dt_s;
  }

  output = (p_pid->params.kp * error) +
           (p_pid->params.ki * p_pid->state.integral) +
           (p_pid->params.kd * derivative);

  output = PID_Clamp(output, p_pid->params.output_limit);

  p_pid->state.prev_error = error;
  p_pid->state.prev_output = output;

  return output;
}
