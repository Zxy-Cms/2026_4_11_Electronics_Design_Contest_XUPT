#include "vision_frame.h"

#include <string.h>

#include "uart_device.h"

/**
  * @brief  按大端格式读取 16 位无符号整数。
  * @param  p_data: 指向两个字节数据的首地址。
  * @retval 解析后的 16 位无符号整数。
  */
static uint16_t VisionFrame_ReadBe16(const uint8_t *p_data)
{
  if (p_data == (const uint8_t *)0)
  {
    return 0U;
  }

  return (uint16_t)(((uint16_t)p_data[0] << 8) | (uint16_t)p_data[1]);
}

/**
  * @brief  仅重置视觉解析器的“正在收帧”状态，不清空最近一次有效帧。
  * @param  p_parser: 视觉帧解析器对象指针。
  * @retval None
  */
static void VisionFrame_ResetParserState(VisionFrameParser_TypeDef *p_parser)
{
  if (p_parser == (VisionFrameParser_TypeDef *)0)
  {
    return;
  }

  p_parser->payload_index = 0U;
  p_parser->receiving = 0U;
}

/**
  * @brief  在收到完整 8 字节负载后提交 1 帧最新视觉数据。
  * @param  p_parser: 视觉帧解析器对象指针。
  * @param  now_ms: 当前系统毫秒节拍，用于给该帧打时间戳。
  * @retval 1 表示成功提交新帧，0 表示当前负载长度无效。
  */
static uint8_t VisionFrame_CommitPayload(VisionFrameParser_TypeDef *p_parser, uint32_t now_ms)
{
  VisionFrame_TypeDef frame;
  uint16_t region_a;
  uint16_t region_b;

  if ((p_parser == (VisionFrameParser_TypeDef *)0) ||
      (p_parser->payload_index != VISION_FRAME_PAYLOAD_SIZE))
  {
    return 0U;
  }

  memset(&frame, 0, sizeof(frame));

  frame.current_x = VisionFrame_ReadBe16(&p_parser->payload[0]);
  frame.current_y = VisionFrame_ReadBe16(&p_parser->payload[2]);

  /* 视觉端当前把目标区域号也按“高八位+低八位”的 16 位格式发了两遍。
     这里直接解析成两个 uint16，只要两次内容一致且范围在 1~9 内，
     就认为本帧带了一个有效目标区域号。 */
  region_a = VisionFrame_ReadBe16(&p_parser->payload[4]);
  region_b = VisionFrame_ReadBe16(&p_parser->payload[6]);

  if ((region_a == region_b) &&
      (region_a >= 1U) &&
      (region_a <= 9U))
  {
    frame.target_region_id = (uint8_t)region_a;
    frame.target_region_valid = 1U;
  }

  frame.frame_valid = 1U;
  frame.recv_tick_ms = now_ms;

  p_parser->latest_frame = frame;

  return 1U;
}

/**
  * @brief  把 1 个原始字节送入视觉协议状态机。
  * @param  p_parser: 视觉帧解析器对象指针。
  * @param  data: 本次收到的单个原始字节。
  * @param  now_ms: 当前系统毫秒节拍。
  * @retval 1 表示本字节触发了解析出完整新帧，0 表示仍在收帧或收帧失败。
  */
static uint8_t VisionFrame_PushByte(VisionFrameParser_TypeDef *p_parser,
                                    uint8_t data,
                                    uint32_t now_ms)
{
  if (p_parser == (VisionFrameParser_TypeDef *)0)
  {
    return 0U;
  }

  if (p_parser->receiving == 0U)
  {
    if (data == VISION_FRAME_HEAD)
    {
      p_parser->receiving = 1U;
      p_parser->payload_index = 0U;
    }
    return 0U;
  }

  if (p_parser->payload_index < VISION_FRAME_PAYLOAD_SIZE)
  {
    p_parser->payload[p_parser->payload_index] = data;
    p_parser->payload_index++;
    return 0U;
  }

  if (data == VISION_FRAME_TAIL)
  {
    uint8_t is_new_frame;

    is_new_frame = VisionFrame_CommitPayload(p_parser, now_ms);
    VisionFrame_ResetParserState(p_parser);
    return is_new_frame;
  }

  /* 若期望帧尾时又收到新的帧头，则直接重新同步到新帧。 */
  if (data == VISION_FRAME_HEAD)
  {
    p_parser->payload_index = 0U;
    p_parser->receiving = 1U;
    return 0U;
  }

  VisionFrame_ResetParserState(p_parser);
  return 0U;
}

/**
  * @brief  初始化视觉帧解析器对象。
  * @param  p_parser: 视觉帧解析器对象指针。
  * @retval None
  */
void VisionFrameParser_Init(VisionFrameParser_TypeDef *p_parser)
{
  if (p_parser == (VisionFrameParser_TypeDef *)0)
  {
    return;
  }

  memset(p_parser, 0, sizeof(*p_parser));
}

/**
  * @brief  清空视觉解析器的收帧状态和最近一次成功帧。
  * @param  p_parser: 视觉帧解析器对象指针。
  * @retval None
  */
void VisionFrameParser_Clear(VisionFrameParser_TypeDef *p_parser)
{
  if (p_parser == (VisionFrameParser_TypeDef *)0)
  {
    return;
  }

  memset(p_parser, 0, sizeof(*p_parser));
}

/**
  * @brief  轮询 UART4 视觉字节队列，并尝试解析出新的视觉帧。
  * @param  p_parser: 视觉帧解析器对象指针。
  * @param  now_ms: 当前系统毫秒节拍。
  * @retval 1 表示至少解析出 1 帧新数据，0 表示本次没有新帧。
  */
uint8_t VisionFrameParser_PollInput(VisionFrameParser_TypeDef *p_parser, uint32_t now_ms)
{
  uint8_t rx_byte;
  uint8_t has_new_frame = 0U;

  if (p_parser == (VisionFrameParser_TypeDef *)0)
  {
    return 0U;
  }

  while (Uart_Device_PopByte(UART_DEVICE_UART4, &rx_byte) != 0U)
  {
    if (VisionFrame_PushByte(p_parser, rx_byte, now_ms) != 0U)
    {
      has_new_frame = 1U;
    }
  }

  return has_new_frame;
}

/**
  * @brief  获取最近一次解析成功的视觉帧。
  * @param  p_parser: 视觉帧解析器对象指针。
  * @retval 指向最近视觉帧的常量指针；若参数无效则返回 NULL。
  */
const VisionFrame_TypeDef *VisionFrameParser_GetLatest(const VisionFrameParser_TypeDef *p_parser)
{
  if (p_parser == (const VisionFrameParser_TypeDef *)0)
  {
    return (const VisionFrame_TypeDef *)0;
  }

  return &p_parser->latest_frame;
}
