#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "main.h"

/* 上行帧打包，格式见 docs/protocol.md（2026-10-28 定稿）
 * 只做打包：发送在主循环，解析在上位机
 *
 * TODO: 帧内暂无 I2C 错误计数与芯片温度，新增字段需升版本号或另开 CMD，
 *       保证旧帧长可继续解析 */

#define PROTO_SOF          0xAAu
#define PROTO_EOF          0x55u
#define PROTO_CMD_DATA     0x01u
#define PROTO_CMD_HEART    0x02u

/* 数据域字节数 = TICK 4 + ADC_MV 2 + 六轴 12，不含 SOF/CMD/LEN/CRC/EOF */
#define PROTO_LEN_DATA     18u

#define PROTO_FRAME_LEN    24u   /* 调用方缓冲区不得小于此值 */

/* 字段顺序即帧偏移顺序，改字段须同步 docs/protocol.md 与上位机解包 */
typedef struct
{
  uint32_t tick_ms;       /* HAL_GetTick()，49.7 天回绕 */
  uint16_t adc_mv;        /* 码值已在 adc_dma.c 换算；VREF_MV=3300 为标称值，待万用表校准 */
  int16_t  accX;
  int16_t  accY;
  int16_t  accZ;
  int16_t  gyroX;
  int16_t  gyroY;
  int16_t  gyroZ;
} PROTO_Sample_t;

/* CRC16-Modbus，初值 0xFFFF、反射多项式 0xA001 */
uint16_t PROTO_CRC16(const uint8_t *p, uint16_t len);

/* 返回帧长，恒为 PROTO_FRAME_LEN；out 缓冲区由调用方提供 */
uint16_t PROTO_PackData(const PROTO_Sample_t *s, uint8_t *out);
uint16_t PROTO_PackHeartbeat(uint32_t tick_ms, uint8_t *out);

#endif /* __PROTOCOL_H */
