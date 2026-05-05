/* Includes ------------------------------------------------------------------*/
#include "bsp_pwm.h"

/**
  * @brief  初始化并启动指定 PWM 通道。
  * @param  p_pwm: PWM 配置结构体指针。
  * @retval BSP_OK 表示成功，其余表示失败。
  */
BSP_StatusTypeDef BSP_PWM_Init(const BSP_PWM_TypeDef *p_pwm)
{
  TIM_HandleTypeDef *p_htim;
  HAL_TIM_ChannelStateTypeDef channel_state;

  if ((p_pwm == NULL) || (p_pwm->p_resource == NULL))
  {
    return BSP_ERROR;
  }

  p_htim = p_pwm->p_resource->htim;

  if ((p_htim == NULL) ||
      (p_pwm->p_resource->init == NULL))
  {
    return BSP_ERROR;
  }

  /* 同一个定时器可能挂多个 PWM 通道，例如 TIM12_CH1/CH2。
     仅在句柄处于 RESET 状态时做一次底层初始化，避免重复重配整个定时器。 */
  if (HAL_TIM_PWM_GetState(p_htim) == HAL_TIM_STATE_RESET)
  {
    p_pwm->p_resource->init();
  }

  if (p_pwm->compare > p_htim->Init.Period)
  {
    return BSP_ERROR;
  }

  /* TIM_CHANNEL_1 的宏值本身就是 0，不能用 channel==0 来判断非法。
     这里改为直接检查 HAL 维护的通道状态。 */
  channel_state = HAL_TIM_GetChannelState(p_htim, p_pwm->p_resource->channel);

  /* 如果通道之前处于忙状态，先停止一次，再重新启动。 */
  if (channel_state == HAL_TIM_CHANNEL_STATE_BUSY)
  {
    if (HAL_TIM_PWM_Stop(p_htim, p_pwm->p_resource->channel) != HAL_OK)
    {
      return BSP_ERROR;
    }

    channel_state = HAL_TIM_GetChannelState(p_htim, p_pwm->p_resource->channel);
  }

  if (channel_state != HAL_TIM_CHANNEL_STATE_READY)
  {
    return BSP_ERROR;
  }

  if (HAL_TIM_PWM_Start(p_htim, p_pwm->p_resource->channel) != HAL_OK)
  {
    return BSP_ERROR;
  }

  __HAL_TIM_SET_COMPARE(p_htim, p_pwm->p_resource->channel, p_pwm->compare);

  return BSP_OK;
}

/**
  * @brief  设置指定 PWM 通道的比较值。
  * @param  p_pwm: PWM 配置结构体指针。
  * @retval BSP_OK 表示成功，其余表示失败。
  */
BSP_StatusTypeDef BSP_PWM_Set(const BSP_PWM_TypeDef *p_pwm)
{
  TIM_HandleTypeDef *p_htim;

  if ((p_pwm == NULL) || (p_pwm->p_resource == NULL))
  {
    return BSP_ERROR;
  }

  p_htim = p_pwm->p_resource->htim;

  if (p_htim == NULL)
  {
    return BSP_ERROR;
  }

  if (p_pwm->compare > p_htim->Init.Period)
  {
    return BSP_ERROR;
  }

  __HAL_TIM_SET_COMPARE(p_htim, p_pwm->p_resource->channel, p_pwm->compare);

  return BSP_OK;
}
