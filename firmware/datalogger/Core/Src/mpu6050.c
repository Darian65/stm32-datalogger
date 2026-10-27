/**
  ******************************************************************************
  * @file    mpu6050.c
  * @brief   MPU6050 六轴传感器驱动（I2C2，轮询方式）
  *
  * 接线：GY-521  SCL -> PB10
  *                SDA -> PB11
  *                AD0 -> 悬空（器件地址 0x68）
  *
  * 配置：加速度 ±2g、陀螺 ±250°/s、DLPF 42Hz、输出 100Hz
  *
  * @note  不使用 I2C 中断/IT 模式：F103 硬件 I2C 存在 BUSY 标志锁死的
  *        errata，通信被打断后总线会永久显示忙，后续读写全部超时。
  *        轮询单次读 14 字节约 1.6ms，占 100ms 帧周期 1.6%，可接受。
  ******************************************************************************
  */

#include "mpu6050.h"

extern I2C_HandleTypeDef hi2c2;        /* CubeMX 生成于 main.c */

/* ============================================================
 * 寄存器定义（RM-MPU-6000A 寄存器映射）
 * ============================================================ */
#define MPU6050_ADDR            (0x68 << 1)     /* 7bit 地址 0x68；最低位由 HAL 补读写标志 */

#define MPU6050_TIMEOUT         100             /* I2C 单次操作超时(ms) */

#define MPU6050_REG_SMPLRT_DIV  0x19            /* 陀螺输出分频 */
#define MPU6050_REG_CONFIG      0x1A            /* 数字低通滤波 */
#define MPU6050_REG_GYRO_CFG    0x1B            /* 陀螺量程 */
#define MPU6050_REG_ACCEL_CFG   0x1C            /* 加速度量程 */
#define MPU6050_REG_ACCEL_XOUT  0x3B            /* 数据区首地址，向后连续 14 字节 */
#define MPU6050_REG_PWR_MGMT_1  0x6B            /* 电源管理 */
#define MPU6050_REG_WHO_AM_I    0x75            /* 器件 ID */

/* ============================================================
 * 模块内部状态（外部只能经 IsReady() / GetErrCount() 只读访问）
 * ============================================================ */
static uint8_t  s_ready   = 0;
static uint32_t s_err_cnt = 0;
static uint8_t  s_id      = 0;         /* Init 时读到的 WHO_AM_I，用于追溯器件批次 */

/* 单字节写寄存器：成功 0 / 失败 1 */
static uint8_t MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
  if (HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, reg,
                        I2C_MEMADD_SIZE_8BIT, &data, 1, MPU6050_TIMEOUT) == HAL_OK)
  {
    return 0;
  }
  return 1;
}

/**
  * @brief  初始化器件
  * @retval 错误码，0 = 成功
  *         1  总线无应答（查 SDA/SCL 是否接反、是否共地、模块是否供电）
  *         2  ID 为 0x00 或 0xFF（总线悬空或受干扰）
  *         3  唤醒失败
  *         4  采样分频写入失败
  *         5  DLPF 配置写入失败
  *         6  陀螺量程写入失败
  *         7  加速度量程写入失败
  * @note   步骤顺序不可调换：器件休眠期间写入量程/滤波寄存器无效
  */
uint8_t MPU6050_Init(void)
{
  uint8_t id = 0;

  HAL_Delay(100);                     /* 上电到器件能响应 I2C 需数十 ms */

  if (HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, MPU6050_REG_WHO_AM_I,
                       I2C_MEMADD_SIZE_8BIT, &id, 1, MPU6050_TIMEOUT) != HAL_OK)
  {
    return 1;
  }

  /* 原版返回 0x68；兼容芯片（MPU6500 die 等）常见 0x70，寄存器布局一致，
   * 故仅拦截悬空/干扰值 TODO: 区分记录实际 ID 便于追溯器件批次 */
  if (id != 0x68)
  {
    if (id == 0x00 || id == 0xFF)
    {
      return 2;
    }
  }

  /* SLEEP=0 唤醒，同时 CLKSEL=0 选内部 8MHz 时钟源。
   * 若跳过此步，六轴读数恒为 0，且不报任何错误 */
  if (MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x00))
  {
    return 3;
  }
  HAL_Delay(10);

  /* 输出速率 = 1kHz / (1 + 9) = 100Hz */
  if (MPU6050_WriteReg(MPU6050_REG_SMPLRT_DIV, 0x09))
  {
    return 4;
  }

  /* DLPF_CFG = 3：加速度带宽 44Hz，陀螺带宽 42Hz */
  if (MPU6050_WriteReg(MPU6050_REG_CONFIG, 0x03))
  {
    return 5;
  }

  /* 陀螺 ±250°/s（最灵敏档） */
  if (MPU6050_WriteReg(MPU6050_REG_GYRO_CFG, 0x00))
  {
    return 6;
  }

  /* 加速度 ±2g（最灵敏档） */
  if (MPU6050_WriteReg(MPU6050_REG_ACCEL_CFG, 0x00))
  {
    return 7;
  }

  s_id    = id;
  s_ready = 1;
  return 0;
}

/**
  * @brief  一次性读回 6 轴 + 温度（0x3B~0x48 共 14 字节，地址连续，
  *         利用 I2C 地址自增单次读回，省去多次 START 开销）
  * @param  p 数据容器指针
  * @retval 0 = 成功，1 = 失败（内部错误计数 +1）
  */
uint8_t MPU6050_ReadAll(MPU6050_t *p)
{
  uint8_t buf[14];

  if (HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT,
                       I2C_MEMADD_SIZE_8BIT, buf, 14, MPU6050_TIMEOUT) != HAL_OK)
  {
    s_err_cnt++;
    return 1;
  }

  /* 各轴 16 位有符号，高字节在前 */
  p->accX  = (int16_t)(((uint16_t)buf[0]  << 8) | buf[1]);
  p->accY  = (int16_t)(((uint16_t)buf[2]  << 8) | buf[3]);
  p->accZ  = (int16_t)(((uint16_t)buf[4]  << 8) | buf[5]);
  p->temp  = (int16_t)(((uint16_t)buf[6]  << 8) | buf[7]);
  p->gyroX = (int16_t)(((uint16_t)buf[8]  << 8) | buf[9]);
  p->gyroY = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
  p->gyroZ = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);

  return 0;
}

uint32_t MPU6050_GetErrCount(void)
{
  return s_err_cnt;
}

uint8_t MPU6050_GetID(void)
{
  return s_id;
}

uint8_t MPU6050_IsReady(void)
{
  return s_ready;
}
