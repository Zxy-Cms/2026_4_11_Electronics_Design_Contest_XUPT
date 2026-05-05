/* Includes ------------------------------------------------------------------*/
#include "bsp_uart.h"

/* Private functions ---------------------------------------------------------*/
static BSP_StatusTypeDef BSP_UART_FromHALStatus(HAL_StatusTypeDef hal_status)
{
  if (hal_status == HAL_OK)
  {
    return BSP_OK;
  }

  if ((hal_status == HAL_BUSY) || (hal_status == HAL_TIMEOUT))
  {
    return BSP_BUSY;
  }

  return BSP_ERROR;
}

static BSP_StatusTypeDef BSP_UART_CheckObject(const BSP_UART_TypeDef *p_uart)
{
  if ((p_uart == (const BSP_UART_TypeDef *)0) ||
      (p_uart->p_resource == (const BSP_UART_ResourceTypeDef *)0) ||
      (p_uart->p_resource->huart == (UART_HandleTypeDef *)0) ||
      (p_uart->p_resource->init == (BSP_UART_InitFunc)0))
  {
    return BSP_ERROR;
  }

  return BSP_OK;
}

static BSP_StatusTypeDef BSP_UART_EnsureInit(const BSP_UART_TypeDef *p_uart)
{
  UART_HandleTypeDef *p_huart;

  if (BSP_UART_CheckObject(p_uart) != BSP_OK)
  {
    return BSP_ERROR;
  }

  p_huart = p_uart->p_resource->huart;

  if ((p_huart->gState == HAL_UART_STATE_RESET) ||
      (p_huart->RxState == HAL_UART_STATE_RESET))
  {
    p_uart->p_resource->init();
  }

  if ((p_huart->gState == HAL_UART_STATE_RESET) ||
      (p_huart->RxState == HAL_UART_STATE_RESET))
  {
    return BSP_ERROR;
  }

  return BSP_OK;
}

/**
  * @brief  按需初始化指定 UART 资源。
  * @param  p_uart: UART 控制对象指针。
  * @retval BSP_OK 表示成功，BSP_ERROR 表示失败。
  */
BSP_StatusTypeDef BSP_UART_Init(const BSP_UART_TypeDef *p_uart)
{
  return BSP_UART_EnsureInit(p_uart);
}

/**
  * @brief  通过指定 UART 以阻塞方式发送数据。
  * @param  p_uart: UART 控制对象指针。
  * @param  p_data: 待发送数据缓冲区指针。
  * @param  length: 待发送字节数。
  * @retval BSP_OK 表示成功，BSP_BUSY 表示忙或超时，
  *         BSP_ERROR 表示参数非法或 HAL 返回错误。
  */
BSP_StatusTypeDef BSP_UART_Transmit(const BSP_UART_TypeDef *p_uart,
                                    const uint8_t *p_data,
                                    uint16_t length)
{
  if ((p_data == (const uint8_t *)0) && (length > 0U))
  {
    return BSP_ERROR;
  }

  if (length == 0U)
  {
    return BSP_OK;
  }

  if (BSP_UART_EnsureInit(p_uart) != BSP_OK)
  {
    return BSP_ERROR;
  }

  return BSP_UART_FromHALStatus(
    HAL_UART_Transmit(p_uart->p_resource->huart, p_data, length, p_uart->timeout_ms));
}

/**
  * @brief  通过指定 UART 以阻塞方式接收数据。
  * @param  p_uart: UART 控制对象指针。
  * @param  p_data: 接收缓冲区指针。
  * @param  length: 期望接收的字节数。
  * @retval BSP_OK 表示成功，BSP_BUSY 表示忙或超时，
  *         BSP_ERROR 表示参数非法或 HAL 返回错误。
  */
BSP_StatusTypeDef BSP_UART_Receive(const BSP_UART_TypeDef *p_uart,
                                   uint8_t *p_data,
                                   uint16_t length)
{
  if ((p_data == (uint8_t *)0) || (length == 0U))
  {
    return BSP_ERROR;
  }

  if (BSP_UART_EnsureInit(p_uart) != BSP_OK)
  {
    return BSP_ERROR;
  }

  return BSP_UART_FromHALStatus(
    HAL_UART_Receive(p_uart->p_resource->huart, p_data, length, p_uart->timeout_ms));
}

/**
  * @brief  获取指定 UART 当前发送状态。
  * @param  p_uart: UART 控制对象指针。
  * @retval BSP_OK 表示发送端空闲，BSP_BUSY 表示发送端忙，
  *         BSP_ERROR 表示对象非法或资源未初始化。
  */
BSP_StatusTypeDef BSP_UART_GetTxStatus(const BSP_UART_TypeDef *p_uart)
{
  if (BSP_UART_CheckObject(p_uart) != BSP_OK)
  {
    return BSP_ERROR;
  }

  if (p_uart->p_resource->huart->gState == HAL_UART_STATE_RESET)
  {
    return BSP_ERROR;
  }

  if (p_uart->p_resource->huart->gState == HAL_UART_STATE_READY)
  {
    return BSP_OK;
  }

  return BSP_BUSY;
}

/**
  * @brief  获取指定 UART 当前接收状态。
  * @param  p_uart: UART 控制对象指针。
  * @retval BSP_OK 表示接收端空闲，BSP_BUSY 表示接收端忙，
  *         BSP_ERROR 表示对象非法或资源未初始化。
  */
BSP_StatusTypeDef BSP_UART_GetRxStatus(const BSP_UART_TypeDef *p_uart)
{
  if (BSP_UART_CheckObject(p_uart) != BSP_OK)
  {
    return BSP_ERROR;
  }

  if (p_uart->p_resource->huart->RxState == HAL_UART_STATE_RESET)
  {
    return BSP_ERROR;
  }

  if (p_uart->p_resource->huart->RxState == HAL_UART_STATE_READY)
  {
    return BSP_OK;
  }

  return BSP_BUSY;
}
