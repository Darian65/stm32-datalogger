/**
  ******************************************************************************
  * @file    protocol.c
  * @brief   上行帧序列化，格式见 docs/protocol.md 定稿版
  *
  * 帧布局（24 字节，全小端）：SOF CMD LEN TICK[4] ADC_MV[2] 六轴[12] CRC[2] EOF
  *
  * 不直接 memcpy 结构体发送：结构体对齐填充随编译链与优化等级变化，帧长一旦
  * 漂移，上位机同步即失效。逐字段移位写入，帧格式只由文档定义，与编译器无关。
  ******************************************************************************
  */

#include "protocol.h"

/* CRC 覆盖 byte0~20，其后 2 字节 CRC + 1 字节 EOF */
#define PROTO_CRC_POS      (PROTO_FRAME_LEN - 3u)   /* = 21 */

static void put_u16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v & 0xFFu);      /* 小端，与上位机 '<' 解包一致 */
  p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8)  & 0xFFu);
  p[2] = (uint8_t)((v >> 16) & 0xFFu);
  p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

/* 补码位型原样写入，符号交给上位机解释 */
static void put_i16(uint8_t *p, int16_t v)
{
  put_u16(p, (uint16_t)v);
}

uint16_t PROTO_CRC16(const uint8_t *p, uint16_t len)
{
  uint16_t crc = 0xFFFFu;
  uint16_t i;
  uint8_t  j;

  for (i = 0u; i < len; i++)
  {
    crc ^= (uint16_t)p[i];
    for (j = 0u; j < 8u; j++)
    {
      /* 0xA001 是 0x8005 的位反转形式，Modbus 规定用反射写法 */
      if (crc & 0x0001u)
      {
        crc = (uint16_t)((crc >> 1) ^ 0xA001u);
      }
      else
      {
        crc = (uint16_t)(crc >> 1);
      }
    }
  }
  return crc;
}

/* 填 CRC 与 EOF，必须在数据域写完后调用 */
static void frame_tail(uint8_t *out)
{
  uint16_t crc = PROTO_CRC16(out, PROTO_CRC_POS);
  put_u16(&out[PROTO_CRC_POS], crc);
  out[PROTO_FRAME_LEN - 1u] = (uint8_t)PROTO_EOF;
}

uint16_t PROTO_PackData(const PROTO_Sample_t *s, uint8_t *out)
{
  out[0] = (uint8_t)PROTO_SOF;
  out[1] = (uint8_t)PROTO_CMD_DATA;
  out[2] = (uint8_t)PROTO_LEN_DATA;

  put_u32(&out[3],  s->tick_ms);
  put_u16(&out[7],  s->adc_mv);
  put_i16(&out[9],  s->accX);
  put_i16(&out[11], s->accY);
  put_i16(&out[13], s->accZ);
  put_i16(&out[15], s->gyroX);
  put_i16(&out[17], s->gyroY);
  put_i16(&out[19], s->gyroZ);      /* 写到 20，紧接 CRC 位置 21 */

  frame_tail(out);
  return PROTO_FRAME_LEN;
}

uint16_t PROTO_PackHeartbeat(uint32_t tick_ms, uint8_t *out)
{
  uint16_t i;

  out[0] = (uint8_t)PROTO_SOF;
  out[1] = (uint8_t)PROTO_CMD_HEART;
  out[2] = 0x00u;                   /* 数据域为空，帧长仍保持 24 字节 */
  put_u32(&out[3], tick_ms);

  /* 缓冲区复用：残留的 mV 与六轴会被当成心跳帧内容发出去 */
  for (i = 7u; i < PROTO_CRC_POS; i++)
  {
    out[i] = 0x00u;
  }

  frame_tail(out);
  return PROTO_FRAME_LEN;
}
