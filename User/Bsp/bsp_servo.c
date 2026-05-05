/* Includes ------------------------------------------------------------------*/
#include "bsp_servo.h"
#include "bsp_pwm.h"

/* Private variables ---------------------------------------------------------*/
/* 舵机底层 PWM 资源映射表。
   后续如果增加 TIM3、TIM4 等舵机，只需要在这里继续添加资源配置。 */
static const BSP_PWM_ResourceTypeDef bsp_servo_resource_list[SERVO_TIM_MAX] =
{
  /* SERVO_TIM14 -> TIM14_CH1 -> PF9 */
  [SERVO_TIM14] = {&htim14, MX_TIM14_Init, TIM_CHANNEL_1},
  /* SERVO_TIM13 -> TIM13_CH1 -> PF8 */
  [SERVO_TIM13] = {&htim13, MX_TIM13_Init, TIM_CHANNEL_1},
  /* SERVO_TIM2_1 -> TIM2_CH1 -> PA15 */
  [SERVO_TIM2_1] = {&htim2, MX_TIM2_Init, TIM_CHANNEL_1},
  /* SERVO_TIM2_2 -> TIM2_CH2 -> PB3 */
  [SERVO_TIM2_2] = {&htim2, MX_TIM2_Init, TIM_CHANNEL_2},
  /* SERVO_TIM12_1 -> TIM12_CH1 -> PB14 */
  [SERVO_TIM12_1] = {&htim12, MX_TIM12_Init, TIM_CHANNEL_1},
  /* SERVO_TIM12_2 -> TIM12_CH2 -> PB15 */
  [SERVO_TIM12_2] = {&htim12, MX_TIM12_Init, TIM_CHANNEL_2},
  /* 后续如果增加 TIM3、TIM4 等舵机，在这里继续往下加。 */

};

/* 当前各路舵机对应的 PWM 控制对象。 */
static BSP_PWM_TypeDef bsp_servo_pwm_list[SERVO_TIM_MAX] =
{
  [SERVO_TIM14] = {&bsp_servo_resource_list[SERVO_TIM14], 0U},

  [SERVO_TIM13] = {&bsp_servo_resource_list[SERVO_TIM13], 0U},

  [SERVO_TIM2_1] = {&bsp_servo_resource_list[SERVO_TIM2_1], 0U},
  [SERVO_TIM2_2] = {&bsp_servo_resource_list[SERVO_TIM2_2], 0U},
  [SERVO_TIM12_1] = {&bsp_servo_resource_list[SERVO_TIM12_1], 0U},
  [SERVO_TIM12_2] = {&bsp_servo_resource_list[SERVO_TIM12_2], 0U}
   /* 后续如果增加 TIM3、TIM4 等舵机，在这里继续往下加。 */
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  检查舵机定时器编号是否有效。
  * @param  servo_tim: 舵机定时器编号。
  * @retval 有效返回 1，无效返回 0。
  */
static uint8_t BSP_Servo_IsValidTim(Servo_TimTypeDef servo_tim)
{
  if ((uint32_t)servo_tim >= (uint32_t)SERVO_TIM_MAX)
  {
    return 0U;
  }

  return 1U;
}

/**
  * @brief  初始化指定舵机定时器对应的底层 PWM 输出。
  * @param  servo_tim: 舵机定时器编号。
  * @param  compare: 初始比较值。
  * @retval BSP_OK 表示成功，其余表示失败。
  */
BSP_StatusTypeDef BSP_Servo_Init(Servo_TimTypeDef servo_tim, uint16_t compare)
{
  if (BSP_Servo_IsValidTim(servo_tim) == 0U)
  {
    return BSP_ERROR;
  }

  bsp_servo_pwm_list[(uint32_t)servo_tim].compare = compare;

  return BSP_PWM_Init(&bsp_servo_pwm_list[(uint32_t)servo_tim]);
}

/**
  * @brief  设置指定舵机定时器对应的底层 PWM 比较值。
  * @param  servo_tim: 舵机定时器编号。
  * @param  compare: 目标比较值。
  * @retval BSP_OK 表示成功，其余表示失败。
  */
BSP_StatusTypeDef BSP_Servo_SetCompare(Servo_TimTypeDef servo_tim, uint16_t compare)
{
  if (BSP_Servo_IsValidTim(servo_tim) == 0U)
  {
    return BSP_ERROR;
  }

  bsp_servo_pwm_list[(uint32_t)servo_tim].compare = compare;

  return BSP_PWM_Set(&bsp_servo_pwm_list[(uint32_t)servo_tim]);
}


