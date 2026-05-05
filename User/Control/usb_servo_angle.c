#include "usb_servo_angle.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Servo_device.h"
#include "my_usb_device.h"

#define USB_SERVO_ANGLE_FRAME_HEAD   0xAAU
#define USB_SERVO_ANGLE_FRAME_TAIL   0xA5U
#define USB_SERVO_ANGLE_BUFFER_SIZE  32U

typedef struct
{
  char rx_buffer[USB_SERVO_ANGLE_BUFFER_SIZE];
  uint16_t rx_length;
  uint8_t in_frame;
} UsbServoAngle_ContextTypeDef;

static UsbServoAngle_ContextTypeDef g_usb_servo_angle;

static const char *UsbServoAngle_SkipBlank(const char *p_text)
{
  while ((*p_text == ' ') || (*p_text == '\t') || (*p_text == '\r') || (*p_text == '\n'))
  {
    p_text++;
  }

  return p_text;
}

static void UsbServoAngle_ResetParser(void)
{
  g_usb_servo_angle.rx_length = 0U;
  g_usb_servo_angle.rx_buffer[0] = '\0';
  g_usb_servo_angle.in_frame = 0U;
}

static BSP_StatusTypeDef UsbServoAngle_SendReply(const char *p_text)
{
  uint16_t retry;

  if (p_text == (const char *)0)
  {
    return BSP_ERROR;
  }

  for (retry = 0U; retry < 20U; retry++)
  {
    BSP_StatusTypeDef status;

    status = Usb_Device_SendString(p_text);
    if (status == BSP_OK)
    {
      return BSP_OK;
    }
  }

  return BSP_BUSY;
}

static uint8_t UsbServoAngle_ParseCommand(const char *p_text,
                                          Servo_TimTypeDef *p_servo_tim,
                                          uint16_t *p_angle)
{
  char *p_end;
  long first_value;
  long second_value;

  if ((p_text == (const char *)0) ||
      (p_servo_tim == (Servo_TimTypeDef *)0) ||
      (p_angle == (uint16_t *)0))
  {
    return 0U;
  }

  p_text = UsbServoAngle_SkipBlank(p_text);
  if (*p_text == '\0')
  {
    return 0U;
  }

  first_value = strtol(p_text, &p_end, 10);
  if (p_end == p_text)
  {
    return 0U;
  }

  p_text = UsbServoAngle_SkipBlank(p_end);

  if (*p_text == '\0')
  {
    if ((first_value < 0L) || (first_value > 180L))
    {
      return 0U;
    }

    *p_servo_tim = SERVO_TIM14;
    *p_angle = (uint16_t)first_value;
    return 1U;
  }

  second_value = strtol(p_text, &p_end, 10);
  if (p_end == p_text)
  {
    return 0U;
  }

  p_text = UsbServoAngle_SkipBlank(p_end);
  if (*p_text != '\0')
  {
    return 0U;
  }

  if ((second_value < 0L) || (second_value > 180L))
  {
    return 0U;
  }

  if (first_value == 14L)
  {
    *p_servo_tim = SERVO_TIM14;
  }
  else if (first_value == 13L)
  {
    *p_servo_tim = SERVO_TIM13;
  }
  else
  {
    return 0U;
  }

  *p_angle = (uint16_t)second_value;
  return 1U;
}

static void UsbServoAngle_HandleFrame(void)
{
  Servo_TimTypeDef servo_tim;
  uint16_t angle;
  char reply[32];

  if (UsbServoAngle_ParseCommand(g_usb_servo_angle.rx_buffer, &servo_tim, &angle) == 0U)
  {
    (void)UsbServoAngle_SendReply("ERR CMD\r\n");
    UsbServoAngle_ResetParser();
    return;
  }

  if (Servo_SetAngle(servo_tim, angle) != BSP_OK)
  {
    (void)UsbServoAngle_SendReply("ERR SET\r\n");
    UsbServoAngle_ResetParser();
    return;
  }

  (void)snprintf(reply,
                 sizeof(reply),
                 "OK T%u A%u\r\n",
                 (servo_tim == SERVO_TIM14) ? 14U : 13U,
                 angle);
  (void)UsbServoAngle_SendReply(reply);
  UsbServoAngle_ResetParser();
}

static void UsbServoAngle_PushByte(uint8_t data)
{
  if (data == USB_SERVO_ANGLE_FRAME_HEAD)
  {
    UsbServoAngle_ResetParser();
    g_usb_servo_angle.in_frame = 1U;
    return;
  }

  if (data == USB_SERVO_ANGLE_FRAME_TAIL)
  {
    if (g_usb_servo_angle.in_frame != 0U)
    {
      UsbServoAngle_HandleFrame();
    }
    else
    {
      UsbServoAngle_ResetParser();
    }
    return;
  }

  if (g_usb_servo_angle.in_frame == 0U)
  {
    return;
  }

  if (g_usb_servo_angle.rx_length >= (USB_SERVO_ANGLE_BUFFER_SIZE - 1U))
  {
    (void)UsbServoAngle_SendReply("ERR LEN\r\n");
    UsbServoAngle_ResetParser();
    return;
  }

  g_usb_servo_angle.rx_buffer[g_usb_servo_angle.rx_length] = (char)data;
  g_usb_servo_angle.rx_length++;
  g_usb_servo_angle.rx_buffer[g_usb_servo_angle.rx_length] = '\0';
}

BSP_StatusTypeDef UsbServoAngle_Init(void)
{
  memset(&g_usb_servo_angle, 0, sizeof(g_usb_servo_angle));

  if (Servo_Init(SERVO_TIM14) != BSP_OK)
  {
    return BSP_ERROR;
  }

  if (Servo_Init(SERVO_TIM13) != BSP_OK)
  {
    return BSP_ERROR;
  }

  (void)Servo_SetAngle(SERVO_TIM14, 90U);
  (void)Servo_SetAngle(SERVO_TIM13, 90U);

  Usb_Device_ClearRxBuffer();
  Usb_Device_ClearRxOverflow();
  UsbServoAngle_ResetParser();
  (void)UsbServoAngle_SendReply("USB SERVO READY\r\n");

  return BSP_OK;
}

void UsbServoAngle_Task(void)
{
  uint8_t rx_temp[64];
  uint16_t read_length;
  uint16_t index;

  if (Usb_Device_HasRxOverflow() != 0U)
  {
    Usb_Device_ClearRxOverflow();
    Usb_Device_ClearRxBuffer();
    (void)UsbServoAngle_SendReply("ERR OVR\r\n");
    UsbServoAngle_ResetParser();
  }

  while (Usb_Device_GetRxCount() > 0U)
  {
    read_length = Usb_Device_Read(rx_temp, sizeof(rx_temp));
    if (read_length == 0U)
    {
      break;
    }

    for (index = 0U; index < read_length; index++)
    {
      UsbServoAngle_PushByte(rx_temp[index]);
    }
  }
}
