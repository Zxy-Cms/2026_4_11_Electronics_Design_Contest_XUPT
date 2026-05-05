/* Includes ------------------------------------------------------------------*/
#include "bsp_usb.h"
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"

/* Private macros ------------------------------------------------------------*/
/* USB CDC 接收环形缓冲区大小。 */
#define BSP_USB_RX_BUFFER_SIZE  512U

/* Private variables ---------------------------------------------------------*/
/* USB 接收数据缓存。 */
static uint8_t bsp_usb_rx_buffer[BSP_USB_RX_BUFFER_SIZE];

/* 环形缓冲区写指针。 */
static volatile uint16_t bsp_usb_rx_head = 0U;

/* 环形缓冲区读指针。 */
static volatile uint16_t bsp_usb_rx_tail = 0U;

/* 当前缓冲区内有效数据长度。 */
static volatile uint16_t bsp_usb_rx_count = 0U;

/* 接收数据超过缓冲区容量时置位。 */
static volatile uint8_t bsp_usb_rx_overflow = 0U;

/* CubeMX 生成的 USB 设备句柄。 */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  进入临界区，保护 USB 共享数据。
  * @retval 返回进入前的 PRIMASK 状态。
  */
static uint32_t BSP_USB_EnterCritical(void)
{
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();

  return primask;
}

/**
  * @brief  退出临界区，恢复之前的中断状态。
  * @param  primask: 由 BSP_USB_EnterCritical 保存的 PRIMASK 状态。
  * @retval 无
  */
static void BSP_USB_ExitCritical(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

/**
  * @brief  将新收到的 USB CDC 数据写入本地环形缓冲区。
  * @param  p_data: 接收到的数据指针。
  * @param  length: 本次接收到的字节数。
  * @retval 无
  */
void BSP_USB_RxCallback(uint8_t *p_data, uint16_t length)
{
  uint16_t index;
  uint32_t primask;

  if ((p_data == NULL) || (length == 0U))
  {
    return;
  }

  primask = BSP_USB_EnterCritical();

  for (index = 0U; index < length; index++)
  {
    if (bsp_usb_rx_count >= BSP_USB_RX_BUFFER_SIZE)
    {
      bsp_usb_rx_overflow = 1U;
      break;
    }

    bsp_usb_rx_buffer[bsp_usb_rx_head] = p_data[index];
    bsp_usb_rx_head++;

    if (bsp_usb_rx_head >= BSP_USB_RX_BUFFER_SIZE)
    {
      bsp_usb_rx_head = 0U;
    }

    bsp_usb_rx_count++;
  }

  BSP_USB_ExitCritical(primask);
}

/**
  * @brief  通过 USB CDC 发送一段数据。
  * @param  p_data: 待发送数据指针。
  * @param  length: 待发送字节数。
  * @retval BSP_OK 表示成功，BSP_BUSY 表示忙，BSP_ERROR 表示失败。
  */
BSP_StatusTypeDef BSP_USB_Send(const uint8_t *p_data, uint16_t length)
{
  uint8_t usb_status;

  if ((p_data == NULL) && (length > 0U))
  {
    return BSP_ERROR;
  }

  if (length == 0U)
  {
    return BSP_OK;
  }

  if ((hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) ||
      (hUsbDeviceFS.pClassData == NULL))
  {
    return BSP_ERROR;
  }

  usb_status = CDC_Transmit_FS((uint8_t *)p_data, length);

  if (usb_status == USBD_OK)
  {
    return BSP_OK;
  }

  if (usb_status == USBD_BUSY)
  {
    return BSP_BUSY;
  }

  return BSP_ERROR;
}

/**
  * @brief  从本地 USB 接收环形缓冲区读取数据。
  * @param  p_data: 输出缓冲区指针。
  * @param  length: 最大读取字节数。
  * @retval 实际读出的字节数。
  */
uint16_t BSP_USB_Read(uint8_t *p_data, uint16_t length)
{
  uint16_t read_length = 0U;
  uint32_t primask;

  if ((p_data == NULL) || (length == 0U))
  {
    return 0U;
  }

  primask = BSP_USB_EnterCritical();

  while ((read_length < length) && (bsp_usb_rx_count > 0U))
  {
    p_data[read_length] = bsp_usb_rx_buffer[bsp_usb_rx_tail];
    bsp_usb_rx_tail++;

    if (bsp_usb_rx_tail >= BSP_USB_RX_BUFFER_SIZE)
    {
      bsp_usb_rx_tail = 0U;
    }

    bsp_usb_rx_count--;
    read_length++;
  }

  BSP_USB_ExitCritical(primask);

  return read_length;
}

/**
  * @brief  获取 USB 接收缓冲区内当前可读的数据长度。
  * @retval 当前可读取的字节数。
  */
uint16_t BSP_USB_GetRxCount(void)
{
  uint16_t rx_count;
  uint32_t primask;

  primask = BSP_USB_EnterCritical();
  rx_count = bsp_usb_rx_count;
  BSP_USB_ExitCritical(primask);

  return rx_count;
}

/**
  * @brief  查询 USB 接收环形缓冲区是否发生溢出。
  * @retval 发生溢出返回 1，否则返回 0。
  */
uint8_t BSP_USB_HasRxOverflow(void)
{
  uint8_t overflow_flag;
  uint32_t primask;

  primask = BSP_USB_EnterCritical();
  overflow_flag = bsp_usb_rx_overflow;
  BSP_USB_ExitCritical(primask);

  return overflow_flag;
}

/**
  * @brief  清空全部 USB 接收缓存数据。
  * @retval 无
  */
void BSP_USB_ClearRxBuffer(void)
{
  uint32_t primask;

  primask = BSP_USB_EnterCritical();
  bsp_usb_rx_head = 0U;
  bsp_usb_rx_tail = 0U;
  bsp_usb_rx_count = 0U;
  BSP_USB_ExitCritical(primask);
}

/**
  * @brief  清除 USB 接收缓冲区溢出标志。
  * @retval 无
  */
void BSP_USB_ClearRxOverflow(void)
{
  uint32_t primask;

  primask = BSP_USB_EnterCritical();
  bsp_usb_rx_overflow = 0U;
  BSP_USB_ExitCritical(primask);
}
