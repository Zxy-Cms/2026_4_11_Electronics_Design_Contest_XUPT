#ifndef __VISION_FRAME_H__
#define __VISION_FRAME_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 当前视觉串口协议固定为:
   0xAA + current_x(2B) + current_y(2B) + region_a(2B) + region_b(2B) + 0xA5

   其中:
   1. current_x/current_y 为当前值，按大端发送。
   2. region_a/region_b 为视觉给出的目标区域号，也按大端发送。
   3. 普通题目里这两个 16 位字段可以都发 0。
   4. 只有发挥 0xB5 模式会使用这两个字段。
   5. 当 region_a == region_b 且范围在 1~9 内时，认为视觉给出了有效目标区域号。

   示例:
   AA 01 40 01 3C 00 05 00 05 A5
   表示 current=(320,316)，目标区域=5。 */
#define VISION_FRAME_HEAD                 0xAAU
#define VISION_FRAME_TAIL                 0xA5U
#define VISION_FRAME_PAYLOAD_SIZE            8U

typedef struct
{
  uint16_t current_x;
  uint16_t current_y;
  uint8_t target_region_id;
  uint8_t target_region_valid;
  uint8_t frame_valid;
  uint32_t recv_tick_ms;
} VisionFrame_TypeDef;

typedef struct
{
  uint8_t payload[VISION_FRAME_PAYLOAD_SIZE];
  uint8_t payload_index;
  uint8_t receiving;
  VisionFrame_TypeDef latest_frame;
} VisionFrameParser_TypeDef;

/**
  * @brief  初始化视觉帧解析器对象。
  * @param  p_parser: 视觉帧解析器对象指针。
  * @retval None
  */
void VisionFrameParser_Init(VisionFrameParser_TypeDef *p_parser);

/**
  * @brief  清空视觉解析器的收帧状态和最近一次成功帧。
  * @param  p_parser: 视觉帧解析器对象指针。
  * @retval None
  */
void VisionFrameParser_Clear(VisionFrameParser_TypeDef *p_parser);

/**
  * @brief  轮询 UART4 视觉字节队列，并尝试解析出新的视觉帧。
  * @param  p_parser: 视觉帧解析器对象指针。
  * @param  now_ms: 当前系统毫秒节拍。
  * @retval 1 表示至少解析出 1 帧新数据，0 表示本次没有新帧。
  */
uint8_t VisionFrameParser_PollInput(VisionFrameParser_TypeDef *p_parser, uint32_t now_ms);

/**
  * @brief  获取最近一次解析成功的视觉帧。
  * @param  p_parser: 视觉帧解析器对象指针。
  * @retval 指向最近视觉帧的常量指针；若参数无效则返回 NULL。
  */
const VisionFrame_TypeDef *VisionFrameParser_GetLatest(const VisionFrameParser_TypeDef *p_parser);

#ifdef __cplusplus
}
#endif

#endif /* __VISION_FRAME_H__ */
