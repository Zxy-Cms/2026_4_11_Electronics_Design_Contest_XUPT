#include "serial_screen_device.h"

#include <stdio.h>
#include <string.h>

#include "uart_device.h"
#include "usart.h"

#define SERIAL_SCREEN_DEVICE_QUEUE_SIZE  32U

/* 串口屏设备层只做三件事:
   1. 用 USART1 中断逐字节收协议数据
   2. 把字节缓存到环形队列，交给状态机解释
   3. 提供统一的串口屏发送接口，供上层更新页面控件 */
typedef struct
{
  uint8_t queue[SERIAL_SCREEN_DEVICE_QUEUE_SIZE];
  uint8_t head;
  uint8_t tail;
  uint8_t count;
} SerialScreen_DeviceContextTypeDef;

static SerialScreen_DeviceContextTypeDef g_serial_screen_context;
static uint8_t g_serial_screen_rx_byte = 0U;

/**
  * @brief  启动 USART1 的单字节中断接收。
  * @param  None
  * @retval HAL_OK 表示挂接成功，其余表示串口当前状态异常。
  */
static HAL_StatusTypeDef SerialScreen_Device_StartReceiveIT(void)
{
  return HAL_UART_Receive_IT(&huart1, &g_serial_screen_rx_byte, 1U);
}

/**
  * @brief  把串口屏收到的字节压入环形队列。
  * @param  command: 当前收到的单字节数据。
  * @retval None
  */
static void SerialScreen_Device_PushCommand(uint8_t command)
{
  if (g_serial_screen_context.count >= SERIAL_SCREEN_DEVICE_QUEUE_SIZE)
  {
    /* 队列满时丢掉最旧字节，保证后续新命令还能进入。 */
    g_serial_screen_context.tail =
      (uint8_t)((g_serial_screen_context.tail + 1U) % SERIAL_SCREEN_DEVICE_QUEUE_SIZE);
    g_serial_screen_context.count--;
  }

  g_serial_screen_context.queue[g_serial_screen_context.head] = command;
  g_serial_screen_context.head =
    (uint8_t)((g_serial_screen_context.head + 1U) % SERIAL_SCREEN_DEVICE_QUEUE_SIZE);
  g_serial_screen_context.count++;
}

/**
  * @brief  初始化串口屏设备层并打开 USART1 接收中断。
  * @param  None
  * @retval BSP_OK 表示初始化成功，BSP_ERROR 表示中断接收启动失败。
  */
BSP_StatusTypeDef SerialScreen_Device_Init(void)
{
  memset(&g_serial_screen_context, 0, sizeof(g_serial_screen_context));
  g_serial_screen_rx_byte = 0U;

  __HAL_UART_CLEAR_PEFLAG(&huart1);
  huart1.ErrorCode = HAL_UART_ERROR_NONE;

  return (SerialScreen_Device_StartReceiveIT() == HAL_OK) ? BSP_OK : BSP_ERROR;
}

/**
  * @brief  从串口屏命令队列中取出 1 个待处理字节。
  * @param  p_command: 输出参数，返回取出的字节。
  * @retval 1 表示成功取到数据，0 表示当前队列为空或参数无效。
  */
uint8_t SerialScreen_Device_PopCommand(uint8_t *p_command)
{
  if ((p_command == (uint8_t *)0) || (g_serial_screen_context.count == 0U))
  {
    return 0U;
  }

  *p_command = g_serial_screen_context.queue[g_serial_screen_context.tail];
  g_serial_screen_context.tail =
    (uint8_t)((g_serial_screen_context.tail + 1U) % SERIAL_SCREEN_DEVICE_QUEUE_SIZE);
  g_serial_screen_context.count--;

  return 1U;
}

/**
  * @brief  清空串口屏命令队列，供后续“复位调平接口”调用。
  * @param  None
  * @retval None
  */
void SerialScreen_Device_ClearCommandQueue(void)
{
  memset(&g_serial_screen_context, 0, sizeof(g_serial_screen_context));
}

/**
  * @brief  USART1 接收完成回调的设备层处理函数。
  * @param  None
  * @retval None
  */
void SerialScreen_Device_OnRxComplete(void)
{
  SerialScreen_Device_PushCommand(g_serial_screen_rx_byte);
  (void)SerialScreen_Device_StartReceiveIT();
}

/**
  * @brief  USART1 出错后的恢复处理函数。
  * @param  None
  * @retval None
  */
void SerialScreen_Device_OnError(void)
{
  __HAL_UART_CLEAR_PEFLAG(&huart1);
  huart1.ErrorCode = HAL_UART_ERROR_NONE;
  (void)SerialScreen_Device_StartReceiveIT();
}

/**
  * @brief  通过串口屏使用的 USART1 发送一段原始数据。
  * @param  p_data: 待发送数据缓冲区首地址。
  * @param  length: 需要发送的字节数。
  * @retval BSP_OK 表示发送成功，BSP_BUSY 表示串口忙，BSP_ERROR 表示参数或底层异常。
  */
BSP_StatusTypeDef SerialScreen_Device_SendBytes(const uint8_t *p_data, uint16_t length)
{
  return Uart_Device_Send(UART_DEVICE_USART1, p_data, length);
}

/**
  * @brief  向串口屏数值控件发送 1 条 "控件名.val=数值" 指令，并自动补齐 3 个 0xFF 结束字节。
  * @param  p_component_name: 串口屏上的数值控件名称，例如 "n0"、"n1"、"n2"。
  * @param  value: 需要显示到控件中的无符号整数值。
  * @retval BSP_OK 表示发送成功，BSP_BUSY 表示串口忙，BSP_ERROR 表示参数非法、格式化失败或缓冲区不足。
  */
BSP_StatusTypeDef SerialScreen_Device_SendNumberValue(const char *p_component_name, uint32_t value)
{
  int length;
  uint8_t tx_buffer[32];

  if ((p_component_name == (const char *)0) || (p_component_name[0] == '\0'))
  {
    return BSP_ERROR;
  }

  length = snprintf((char *)tx_buffer,
                    sizeof(tx_buffer),
                    "%s.val=%lu",
                    p_component_name,
                    (unsigned long)value);
  if ((length <= 0) || ((uint32_t)length > (sizeof(tx_buffer) - 3U)))
  {
    return BSP_ERROR;
  }

  tx_buffer[length] = 0xFFU;
  tx_buffer[length + 1] = 0xFFU;
  tx_buffer[length + 2] = 0xFFU;

  return SerialScreen_Device_SendBytes(tx_buffer, (uint16_t)(length + 3));
}

