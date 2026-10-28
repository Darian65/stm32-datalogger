/**
  ******************************************************************************
  * @file    adc_dma.c
  * @brief   ADC1 通道0(PA0) 定时触发 + DMA 循环搬运驱动
  *
  * 数据通路：TIM3_TRGO(10ms) -> ADC1 转换 -> DMA1 CH1 -> s_buf[]
  * 搬运过程由硬件完成，CPU 仅在攒满一批后被通知一次
  *
  * @note    ADCCLK = PCLK2/6 = 12MHz；采样时间 28.5 cycles + 12.5 转换
  *          ≈ 3.4us/次，相对 10ms 节拍余量充足
  ******************************************************************************
  */

#include "adc_dma.h"

extern ADC_HandleTypeDef hadc1;        /* CubeMX 生成于 main.c */
extern TIM_HandleTypeDef htim3;

/* VDDA 实测电压(mV)，当前为标称值。
 * TODO: 用万用表量板上 3V3 引脚后修正，直接决定绝对精度 */
#define VREF_MV       3300

/* s_ready 由 ADC 转换完成回调（中断上下文）写、主循环读；
 * s_buf 由 DMA 硬件直接写。两条路径编译器都追踪不到，
 * __IO(volatile) 不可省，否则可能被优化掉重复读取 */
static __IO uint8_t  s_ready = 0;
static __IO uint16_t s_buf[ADC_BUF_LEN];

/**
  * @brief  启动采集，顺序不可调换
  * @note   ① 必须先启动 TIM3，否则 ADC 无外部触发源，表现为静默无输出
  *         ② 校准需在启动转换前完成
  */
void ADC_DMA_Start(void)
{
  HAL_TIM_Base_Start(&htim3);                                  /* ① 触发源  */
  HAL_ADCEx_Calibration_Start(&hadc1);                         /* ② 自校准  */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_buf, ADC_BUF_LEN);   /* ③ 启动    */
}

/**
  * @brief  DMA 搬满一批后的完成回调（函数名由 HAL 约定，不可改名）
  * @note   回调中只置标志，不做耗时操作
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)         /* 预留多路 ADC 分流 */
  {
    s_ready = 1;
  }
}

uint8_t ADC_DMA_IsReady(void)
{
  return s_ready;
}

/* 取本批平均值。先清标志再累加：Circular 模式下 DMA 会立即从头覆盖，
 * 尽早清标志可避免下一批数据被误擦除 */
uint16_t ADC_DMA_GetAvg(void)
{
  uint32_t sum = 0;
  uint16_t i;

  s_ready = 0;

  for (i = 0; i < ADC_BUF_LEN; i++)
  {
    sum += s_buf[i];
  }

  return (uint16_t)(sum / ADC_BUF_LEN);
}

/* ADC 输出为比例值：code / 4095 = Vin / VREF_MV
 * 先升至 32 位再乘，避免 uint16_t 溢出 */
uint16_t ADC_ToMillivolt(uint16_t code)
{
  return (uint16_t)((uint32_t)code * VREF_MV / 4095UL);
}
