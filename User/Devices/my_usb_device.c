/* Includes ------------------------------------------------------------------*/
#include "my_usb_device.h"
#include "bsp_usb.h"

/**
  * @brief  通过 USB CDC 发送一段数据。
  * @param  p_data: 待发送数据指针。
  * @param  length: 待发送字节数。
  * @retval BSP_OK 表示成功，BSP_BUSY 表示忙，BSP_ERROR 表示失败。
  */
BSP_StatusTypeDef Usb_Device_Send(const uint8_t *p_data, uint16_t length)
{
  return BSP_USB_Send(p_data, length);
}

/**
  * @brief  通过 USB CDC 发送字符串。
  * @param  p_string: 以 0 结尾的字符串指针。
  * @retval BSP_OK 表示成功，BSP_BUSY 表示忙，BSP_ERROR 表示失败。
  */
BSP_StatusTypeDef Usb_Device_SendString(const char *p_string)
{
  uint16_t length = 0U;

  // if (p_string == NULL)
  // {
  //   return BSP_ERROR;
  // }

  while ((p_string[length] != '\0') && (length < 0xFFFFU))
  {
    length++;
  }

  return BSP_USB_Send((const uint8_t *)p_string, length);
}

/**
  * @brief  从本地 USB 接收缓冲区读取数据。
  * @param  p_data: 输出缓冲区指针。
  * @param  length: 最大读取字节数。
  * @retval 实际读出的字节数。
  */
uint16_t Usb_Device_Read(uint8_t *p_data, uint16_t length)
{
  return BSP_USB_Read(p_data, length);
}

/**
  * @brief  获取 USB 接收缓冲区内当前可读的数据长度。
  * @retval 当前可读取的字节数。
  */
uint16_t Usb_Device_GetRxCount(void)
{
  return BSP_USB_GetRxCount();
}

/**
  * @brief  查询 USB 接收缓冲区是否发生溢出。
  * @retval 发生溢出返回 1，否则返回 0。
  */
uint8_t Usb_Device_HasRxOverflow(void)
{
  return BSP_USB_HasRxOverflow();
}

/**
  * @brief  清空全部 USB 接收缓存数据。
  * @retval 无
  */
void Usb_Device_ClearRxBuffer(void)
{
  BSP_USB_ClearRxBuffer();
}

/**
  * @brief  清除 USB 接收缓冲区溢出标志。
  * @retval 无
  */
void Usb_Device_ClearRxOverflow(void)
{
  BSP_USB_ClearRxOverflow();
}
