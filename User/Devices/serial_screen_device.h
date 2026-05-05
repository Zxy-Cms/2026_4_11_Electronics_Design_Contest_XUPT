#ifndef __SERIAL_SCREEN_DEVICE_H__
#define __SERIAL_SCREEN_DEVICE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_common.h"

/* 串口屏单字节协议定义。
   题目选择用 A1/A2/B1/B2/B3/B4/B5；
   复位调平命令使用 CC；
   目标点输入使用 AA ... F1~F9 ... BB。
   其中 B5 为视觉激光选点模式，不再依赖 AA...BB 手动输入目标。 */
#define SERIAL_SCREEN_CMD_BASIC1        0xA1U
#define SERIAL_SCREEN_CMD_BASIC2        0xA2U
#define SERIAL_SCREEN_CMD_ADVANCED1     0xB1U
#define SERIAL_SCREEN_CMD_ADVANCED2     0xB2U
#define SERIAL_SCREEN_CMD_ADVANCED3     0xB3U
#define SERIAL_SCREEN_CMD_ADVANCED4     0xB4U
#define SERIAL_SCREEN_CMD_ADVANCED5     0xB5U
#define SERIAL_SCREEN_CMD_RESET_LEVEL   0xCCU
#define SERIAL_SCREEN_CMD_INPUT_BEGIN   0xAAU
#define SERIAL_SCREEN_CMD_INPUT_END     0xBBU
#define SERIAL_SCREEN_CMD_TARGET1       0xF1U
#define SERIAL_SCREEN_CMD_TARGET2       0xF2U
#define SERIAL_SCREEN_CMD_TARGET3       0xF3U
#define SERIAL_SCREEN_CMD_TARGET4       0xF4U
#define SERIAL_SCREEN_CMD_TARGET5       0xF5U
#define SERIAL_SCREEN_CMD_TARGET6       0xF6U
#define SERIAL_SCREEN_CMD_TARGET7       0xF7U
#define SERIAL_SCREEN_CMD_TARGET8       0xF8U
#define SERIAL_SCREEN_CMD_TARGET9       0xF9U

/**
  * @brief  初始化串口屏设备层并打开 USART1 接收中断。
  * @param  None
  * @retval BSP_OK 表示初始化成功，BSP_ERROR 表示中断接收启动失败。
  */
BSP_StatusTypeDef SerialScreen_Device_Init(void);

/**
  * @brief  从串口屏命令队列中取出 1 个待处理命令字节。
  * @param  p_command: 输出参数，返回取出的命令字节。
  * @retval 1 表示成功取到命令，0 表示当前队列为空或参数无效。
  */
uint8_t SerialScreen_Device_PopCommand(uint8_t *p_command);

/**
  * @brief  清空串口屏命令队列。
  * @param  None
  * @retval None
  */
void SerialScreen_Device_ClearCommandQueue(void);

/**
  * @brief  USART1 接收完成回调的设备层处理函数。
  * @param  None
  * @retval None
  */
void SerialScreen_Device_OnRxComplete(void);

/**
  * @brief  USART1 出错后的恢复处理函数。
  * @param  None
  * @retval None
  */
void SerialScreen_Device_OnError(void);

/**
  * @brief  通过串口屏使用的 USART1 发送一段原始数据。
  * @param  p_data: 待发送数据缓冲区首地址。
  * @param  length: 需要发送的字节数。
  * @retval BSP_OK 表示发送成功，BSP_BUSY 表示串口忙，BSP_ERROR 表示参数或底层异常。
  */
BSP_StatusTypeDef SerialScreen_Device_SendBytes(const uint8_t *p_data, uint16_t length);

/**
  * @brief  向串口屏的数值控件发送 1 个整数值。
  * @param  p_component_name: 串口屏控件名称，例如 "n0"、"n1"、"n2"。
  * @param  value: 需要写入该控件的整数值。
  * @retval BSP_OK 表示发送成功，BSP_BUSY 表示串口忙，BSP_ERROR 表示参数非法或底层发送失败。
  */
BSP_StatusTypeDef SerialScreen_Device_SendNumberValue(const char *p_component_name, uint32_t value);

#ifdef __cplusplus
}
#endif

#endif /* __SERIAL_SCREEN_DEVICE_H__ */



