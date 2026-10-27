#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"

/* 六轴原始数据容器，量程配置见 MPU6050_Init()：
 *   acc  ±2g     -> 16384 LSB/g
 *   gyro ±250°/s -> 131 LSB/(°/s)
 *   temp         -> T(℃) = temp / 340.0 + 36.53
 */
typedef struct
{
  int16_t accX;
  int16_t accY;
  int16_t accZ;
  int16_t temp;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
} MPU6050_t;

/* 初始化器件，返回 0 成功，非 0 为错误码（定义见 mpu6050.c） */
uint8_t  MPU6050_Init(void);

/* 一次读回 6 轴 + 温度，返回 0 成功；失败时内部累计错误计数 */
uint8_t  MPU6050_ReadAll(MPU6050_t *p);

/* 器件 WHO_AM_I 值（Init 时缓存），用于追溯批次 */
uint8_t  MPU6050_GetID(void);

/* 累计通信失败次数 */
uint32_t MPU6050_GetErrCount(void);

/* 1 = 初始化成功，可以调用 ReadAll() */
uint8_t  MPU6050_IsReady(void);

#endif /* __MPU6050_H */
