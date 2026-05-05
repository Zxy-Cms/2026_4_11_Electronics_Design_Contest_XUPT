#include "jy61p.h"

#include "usart.h"

/* JY61P 设备层职责:
   1. 从 UART4 持续接收角度数据帧
   2. 解析出 Roll/Pitch/Yaw
   3. 向控制层提供线程安全的角度快照
   这里不再通过 USART1 打印调试字符串, 避免和串口屏冲突. */
static uint8_t g_jy61p_rx_buffer[11];
static volatile uint8_t g_jy61p_rx_state = 0U;
static uint8_t g_jy61p_rx_index = 0U;
static uint8_t g_jy61p_rx_byte = 0U;

volatile float Roll = 0.0f;
volatile float Pitch = 0.0f;
volatile float Yaw = 0.0f;

/**
  * @brief  启动 UART4 的单字节中断接收。
  * @param  None
  * @retval BSP_OK 表示启动成功，BSP_ERROR 表示启动失败。
  */
static BSP_StatusTypeDef JY61P_StartReceiveIT(void)
{
  return (HAL_UART_Receive_IT(&huart4, &g_jy61p_rx_byte, 1U) == HAL_OK) ?
         BSP_OK : BSP_ERROR;
}

/**
  * @brief  按 JY61P 的角度帧协议逐字节解析输入数据。
  * @param  rx_data: 当前收到的单个原始字节。
  * @retval None
  */
static void JY61P_ParseByte(uint8_t rx_data)
{
  uint8_t i;
  uint8_t sum = 0U;

  /* 帧头第 1 字节固定为 0x55。 */
  if (g_jy61p_rx_state == 0U)
  {
    if (rx_data == 0x55U)
    {
      g_jy61p_rx_buffer[0] = rx_data;
      g_jy61p_rx_state = 1U;
      g_jy61p_rx_index = 1U;
    }
    return;
  }

  /* 帧头第 2 字节固定为 0x53，代表角度数据帧。 */
  if (g_jy61p_rx_state == 1U)
  {
    if (rx_data == 0x53U)
    {
      g_jy61p_rx_buffer[g_jy61p_rx_index] = rx_data;
      g_jy61p_rx_state = 2U;
      g_jy61p_rx_index = 2U;
    }
    else
    {
      g_jy61p_rx_state = 0U;
      g_jy61p_rx_index = 0U;
    }
    return;
  }

  /* 继续填充剩余 9 个字节，最后做校验和检查。 */
  if (g_jy61p_rx_state == 2U)
  {
    g_jy61p_rx_buffer[g_jy61p_rx_index++] = rx_data;
    if (g_jy61p_rx_index >= 11U)
    {
      for (i = 0U; i < 10U; i++)
      {
        sum = (uint8_t)(sum + g_jy61p_rx_buffer[i]);
      }

      if (sum == g_jy61p_rx_buffer[10])
      {
        Roll = ((int16_t)(((int16_t)g_jy61p_rx_buffer[3] << 8) |
                          (int16_t)g_jy61p_rx_buffer[2])) / 32768.0f * 180.0f;
        Pitch = ((int16_t)(((int16_t)g_jy61p_rx_buffer[5] << 8) |
                           (int16_t)g_jy61p_rx_buffer[4])) / 32768.0f * 180.0f;
        Yaw = ((int16_t)(((int16_t)g_jy61p_rx_buffer[7] << 8) |
                         (int16_t)g_jy61p_rx_buffer[6])) / 32768.0f * 180.0f;
      }

      g_jy61p_rx_state = 0U;
      g_jy61p_rx_index = 0U;
    }
    return;
  }

  g_jy61p_rx_state = 0U;
  g_jy61p_rx_index = 0U;
}

/**
  * @brief  初始化 JY61P 接收状态，并启动 UART4 中断接收。
  * @param  None
  * @retval BSP_OK 表示初始化成功，BSP_ERROR 表示接收中断挂接失败。
  */
BSP_StatusTypeDef JY61P_Init(void)
{
  g_jy61p_rx_state = 0U;
  g_jy61p_rx_index = 0U;
  Roll = 0.0f;
  Pitch = 0.0f;
  Yaw = 0.0f;

  __HAL_UART_CLEAR_PEFLAG(&huart4);
  huart4.ErrorCode = HAL_UART_ERROR_NONE;

  return JY61P_StartReceiveIT();
}

/**
  * @brief  JY61P 设备层保留的周期任务接口。
  * @param  None
  * @retval None
  */
void JY61P_Task(void)
{
  /* 当前版本无需在主循环中额外处理，保留该接口方便后续扩展校准逻辑。 */
}

/**
  * @brief  UART4 接收完成后的设备层处理函数。
  * @param  None
  * @retval None
  */
void JY61P_OnRxComplete(void)
{
  JY61P_ParseByte(g_jy61p_rx_byte);
  (void)JY61P_StartReceiveIT();
}

/**
  * @brief  UART4 出错后的恢复处理函数。
  * @param  None
  * @retval None
  */
void JY61P_OnError(void)
{
  __HAL_UART_CLEAR_PEFLAG(&huart4);
  huart4.ErrorCode = HAL_UART_ERROR_NONE;
  g_jy61p_rx_state = 0U;
  g_jy61p_rx_index = 0U;
  (void)JY61P_StartReceiveIT();
}

/**
  * @brief  读取一次线程安全的欧拉角快照。
  * @param  p_roll: 输出横滚角指针，可传 NULL。
  * @param  p_pitch: 输出俯仰角指针，可传 NULL。
  * @param  p_yaw: 输出航向角指针，可传 NULL。
  * @retval None
  */
void JY61P_GetEuler(float *p_roll, float *p_pitch, float *p_yaw)
{
  uint32_t primask;
  float roll;
  float pitch;
  float yaw;

  primask = __get_PRIMASK();
  __disable_irq();
  roll = Roll;
  pitch = Pitch;
  yaw = Yaw;
  if (primask == 0U)
  {
    __enable_irq();
  }

  if (p_roll != (float *)0)
  {
    *p_roll = roll;
  }

  if (p_pitch != (float *)0)
  {
    *p_pitch = pitch;
  }

  if (p_yaw != (float *)0)
  {
    *p_yaw = yaw;
  }
}
