/* Includes ------------------------------------------------------------------*/
#include "uart_device.h"

#include <string.h>

#include "bsp_uart.h"
#include "serial_screen_device.h"

/* Private macros ------------------------------------------------------------*/
#define UART_DEVICE_DEFAULT_TIMEOUT_MS   UART_DEVICE_WAIT_FOREVER
#define UART_DEVICE_BYTE_QUEUE_SIZE      128U

/* Private types -------------------------------------------------------------*/
typedef struct
{
  uint8_t buffer[UART_DEVICE_BYTE_QUEUE_SIZE];
  uint16_t head;
  uint16_t tail;
  uint16_t count;
  uint8_t rx_byte;
} Uart_DeviceByteQueueTypeDef;

/* Private variables ---------------------------------------------------------*/
static const BSP_UART_ResourceTypeDef g_uart_device_resource_list[UART_DEVICE_MAX] =
{
  [UART_DEVICE_USART1] = {&huart1, MX_USART1_UART_Init},
  [UART_DEVICE_USART6] = {&huart6, MX_USART6_UART_Init},
  [UART_DEVICE_UART4] = {&huart4, MX_UART4_Init}
};

static BSP_UART_TypeDef g_uart_device_list[UART_DEVICE_MAX] =
{
  [UART_DEVICE_USART1] = {&g_uart_device_resource_list[UART_DEVICE_USART1], UART_DEVICE_DEFAULT_TIMEOUT_MS},
  [UART_DEVICE_USART6] = {&g_uart_device_resource_list[UART_DEVICE_USART6], UART_DEVICE_DEFAULT_TIMEOUT_MS},
  [UART_DEVICE_UART4] = {&g_uart_device_resource_list[UART_DEVICE_UART4], UART_DEVICE_DEFAULT_TIMEOUT_MS}
};

/* UART4 视觉输入采用“单字节中断接收 + 环形队列”。
   这样 Control 层只需要从设备层按字节取数据，不需要直接接触 HAL 中断。 */
static Uart_DeviceByteQueueTypeDef g_uart_device_byte_queue[UART_DEVICE_MAX];

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  判断端口编号是否落在设备层支持范围内。
  * @param  uart_port: 待检查的设备层串口端口编号。
  * @retval 1 表示端口合法，0 表示端口越界。
  */
static uint8_t Uart_Device_IsValidPort(Uart_DevicePortTypeDef uart_port)
{
  if ((uint32_t)uart_port >= (uint32_t)UART_DEVICE_MAX)
  {
    return 0U;
  }

  return 1U;
}

/**
  * @brief  把 1 个收到的字节压入指定串口的内部环形队列。
  * @param  uart_port: 需要写入队列的串口端口。
  * @param  data: 本次收到的单字节数据。
  * @retval None
  */
static void Uart_Device_PushByte(Uart_DevicePortTypeDef uart_port, uint8_t data)
{
  Uart_DeviceByteQueueTypeDef *p_queue;

  if (Uart_Device_IsValidPort(uart_port) == 0U)
  {
    return;
  }

  p_queue = &g_uart_device_byte_queue[(uint32_t)uart_port];

  if (p_queue->count >= UART_DEVICE_BYTE_QUEUE_SIZE)
  {
    /* 队列满时丢掉最旧数据，优先保留最新视觉字节，避免解析器长时间吃旧帧。 */
    p_queue->tail = (uint16_t)((p_queue->tail + 1U) % UART_DEVICE_BYTE_QUEUE_SIZE);
    p_queue->count--;
  }

  p_queue->buffer[p_queue->head] = data;
  p_queue->head = (uint16_t)((p_queue->head + 1U) % UART_DEVICE_BYTE_QUEUE_SIZE);
  p_queue->count++;
}

/**
  * @brief  为指定串口重新挂接一次单字节中断接收。
  * @param  uart_port: 需要启动中断接收的串口端口。
  * @retval HAL_OK 表示启动成功，其余返回值表示当前串口状态异常。
  */
static HAL_StatusTypeDef Uart_Device_RestartReceiveIT(Uart_DevicePortTypeDef uart_port)
{
  UART_HandleTypeDef *p_huart;

  if (Uart_Device_IsValidPort(uart_port) == 0U)
  {
    return HAL_ERROR;
  }

  p_huart = g_uart_device_resource_list[(uint32_t)uart_port].huart;
  if (p_huart == (UART_HandleTypeDef *)0)
  {
    return HAL_ERROR;
  }

  __HAL_UART_CLEAR_PEFLAG(p_huart);
  p_huart->ErrorCode = HAL_UART_ERROR_NONE;

  return HAL_UART_Receive_IT(p_huart,
                             &g_uart_device_byte_queue[(uint32_t)uart_port].rx_byte,
                             1U);
}

/**
  * @brief  初始化指定 UART 设备。
  * @param  uart_port: 设备层 UART 端口编号。
  * @retval BSP_OK 表示成功，BSP_ERROR 表示端口非法或初始化失败。
  */
BSP_StatusTypeDef Uart_Device_Init(Uart_DevicePortTypeDef uart_port)
{
  if (Uart_Device_IsValidPort(uart_port) == 0U)
  {
    return BSP_ERROR;
  }

  return BSP_UART_Init(&g_uart_device_list[(uint32_t)uart_port]);
}

/**
  * @brief  设置指定 UART 设备的默认阻塞超时时间。
  * @param  uart_port: 设备层 UART 端口编号。
  * @param  timeout_ms: 超时时间，单位 ms。
  * @retval BSP_OK 表示成功，BSP_ERROR 表示端口非法。
  */
BSP_StatusTypeDef Uart_Device_SetTimeout(Uart_DevicePortTypeDef uart_port, uint32_t timeout_ms)
{
  if (Uart_Device_IsValidPort(uart_port) == 0U)
  {
    return BSP_ERROR;
  }

  g_uart_device_list[(uint32_t)uart_port].timeout_ms = timeout_ms;

  return BSP_OK;
}

/**
  * @brief  通过指定 UART 设备发送一段数据。
  * @param  uart_port: 设备层 UART 端口编号。
  * @param  p_data: 待发送数据缓冲区指针。
  * @param  length: 待发送字节数。
  * @retval BSP_OK 表示发送成功，BSP_BUSY 表示串口忙，BSP_ERROR 表示参数非法或底层错误。
  */
BSP_StatusTypeDef Uart_Device_Send(Uart_DevicePortTypeDef uart_port,
                                   const uint8_t *p_data,
                                   uint16_t length)
{
  if (Uart_Device_IsValidPort(uart_port) == 0U)
  {
    return BSP_ERROR;
  }

  return BSP_UART_Transmit(&g_uart_device_list[(uint32_t)uart_port], p_data, length);
}

/**
  * @brief  通过指定 UART 设备发送一段以 0 结尾的字符串。
  * @param  uart_port: 设备层 UART 端口编号。
  * @param  p_string: 以 0 结尾的字符串指针。
  * @retval BSP_OK 表示成功，BSP_BUSY 表示串口忙，BSP_ERROR 表示参数非法或发送失败。
  */
BSP_StatusTypeDef Uart_Device_SendString(Uart_DevicePortTypeDef uart_port, const char *p_string)
{
  uint16_t length = 0U;

  if (p_string == (const char *)0)
  {
    return BSP_ERROR;
  }

  while ((p_string[length] != '\0') && (length < 0xFFFFU))
  {
    length++;
  }

  return Uart_Device_Send(uart_port, (const uint8_t *)p_string, length);
}

/**
  * @brief  通过指定 UART 设备以阻塞方式接收数据。
  * @param  uart_port: 设备层 UART 端口编号。
  * @param  p_data: 接收缓冲区指针。
  * @param  length: 期望接收的字节数。
  * @retval BSP_OK 表示成功，BSP_BUSY 表示串口忙，BSP_ERROR 表示参数非法或底层错误。
  */
BSP_StatusTypeDef Uart_Device_Read(Uart_DevicePortTypeDef uart_port,
                                   uint8_t *p_data,
                                   uint16_t length)
{
  if (Uart_Device_IsValidPort(uart_port) == 0U)
  {
    return BSP_ERROR;
  }

  return BSP_UART_Receive(&g_uart_device_list[(uint32_t)uart_port], p_data, length);
}

/**
  * @brief  获取指定 UART 设备当前发送状态。
  * @param  uart_port: 设备层 UART 端口编号。
  * @retval BSP_OK 表示发送端空闲，BSP_BUSY 表示发送端忙，BSP_ERROR 表示端口非法。
  */
BSP_StatusTypeDef Uart_Device_GetTxStatus(Uart_DevicePortTypeDef uart_port)
{
  if (Uart_Device_IsValidPort(uart_port) == 0U)
  {
    return BSP_ERROR;
  }

  return BSP_UART_GetTxStatus(&g_uart_device_list[(uint32_t)uart_port]);
}

/**
  * @brief  获取指定 UART 设备当前接收状态。
  * @param  uart_port: 设备层 UART 端口编号。
  * @retval BSP_OK 表示接收端空闲，BSP_BUSY 表示接收端忙，BSP_ERROR 表示端口非法。
  */
BSP_StatusTypeDef Uart_Device_GetRxStatus(Uart_DevicePortTypeDef uart_port)
{
  if (Uart_Device_IsValidPort(uart_port) == 0U)
  {
    return BSP_ERROR;
  }

  return BSP_UART_GetRxStatus(&g_uart_device_list[(uint32_t)uart_port]);
}

/**
  * @brief  为指定串口启动“单字节中断接收 + 内部环形缓冲”功能。
  * @param  uart_port: 需要开启字节流接收的串口端口。
  * @retval BSP_OK 表示启动成功，BSP_ERROR 表示端口非法或中断接收启动失败。
  */
BSP_StatusTypeDef Uart_Device_StartByteReceiveIT(Uart_DevicePortTypeDef uart_port)
{
  if (Uart_Device_Init(uart_port) != BSP_OK)
  {
    return BSP_ERROR;
  }

  Uart_Device_ClearByteQueue(uart_port);

  return (Uart_Device_RestartReceiveIT(uart_port) == HAL_OK) ? BSP_OK : BSP_ERROR;
}

/**
  * @brief  从指定串口的内部字节队列中取出 1 个接收字节。
  * @param  uart_port: 需要读取的串口端口。
  * @param  p_data: 输出参数，返回取出的单字节数据。
  * @retval 1 表示成功取到数据，0 表示队列为空或参数无效。
  */
uint8_t Uart_Device_PopByte(Uart_DevicePortTypeDef uart_port, uint8_t *p_data)
{
  Uart_DeviceByteQueueTypeDef *p_queue;

  if ((Uart_Device_IsValidPort(uart_port) == 0U) || (p_data == (uint8_t *)0))
  {
    return 0U;
  }

  p_queue = &g_uart_device_byte_queue[(uint32_t)uart_port];
  if (p_queue->count == 0U)
  {
    return 0U;
  }

  *p_data = p_queue->buffer[p_queue->tail];
  p_queue->tail = (uint16_t)((p_queue->tail + 1U) % UART_DEVICE_BYTE_QUEUE_SIZE);
  p_queue->count--;

  return 1U;
}

/**
  * @brief  清空指定串口的内部字节接收队列。
  * @param  uart_port: 需要清空队列的串口端口。
  * @retval None
  */
void Uart_Device_ClearByteQueue(Uart_DevicePortTypeDef uart_port)
{
  if (Uart_Device_IsValidPort(uart_port) == 0U)
  {
    return;
  }

  memset(&g_uart_device_byte_queue[(uint32_t)uart_port],
         0,
         sizeof(g_uart_device_byte_queue[(uint32_t)uart_port]));
}

/**
  * @brief  HAL 串口接收完成统一分发回调。
  * @param  huart: 触发回调的 HAL 串口句柄。
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == (UART_HandleTypeDef *)0)
  {
    return;
  }

  if (huart->Instance == USART1)
  {
    SerialScreen_Device_OnRxComplete();
    return;
  }

  if (huart->Instance == UART4)
  {
    Uart_Device_PushByte(UART_DEVICE_UART4,
                         g_uart_device_byte_queue[(uint32_t)UART_DEVICE_UART4].rx_byte);
    (void)Uart_Device_RestartReceiveIT(UART_DEVICE_UART4);
  }
}

/**
  * @brief  HAL 串口出错统一分发回调。
  * @param  huart: 触发回调的 HAL 串口句柄。
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == (UART_HandleTypeDef *)0)
  {
    return;
  }

  if (huart->Instance == USART1)
  {
    SerialScreen_Device_OnError();
    return;
  }

  if (huart->Instance == UART4)
  {
    (void)Uart_Device_RestartReceiveIT(UART_DEVICE_UART4);
  }
}
