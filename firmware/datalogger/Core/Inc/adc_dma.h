#ifndef __ADC_DMA_H
#define __ADC_DMA_H

#include "main.h"

/* ============================================================
 * ADC1 + DMA 采集驱动，对外只暴露四个接口
 * 定时器触发、DMA 搬运、缓冲区这些细节全部关在 .c 里
 * ============================================================ */

/* 每批样本数。统计窗口 = ADC_BUF_LEN × 10ms(TIM3 节拍)
 * 改动后需重新核对两点：
 *   1) 是否仍匹配协议约定的 100ms 一帧
 *   2) 累加值是否会超出 uint32_t（当前 10×4095 远小于上限）*/
#define ADC_BUF_LEN   10

/* 启动采集。此后 TIM3 打拍 → ADC 转换 → DMA 搬运，CPU 不再参与 */
void     ADC_DMA_Start(void);

/* 1 = 已攒满一批，可以取 */
uint8_t  ADC_DMA_IsReady(void);

/* 取本批平均值（0~4095 原始码），内部自动清标志
 * TODO: 取值期间 DMA 可能已开始覆盖，后续可改双缓冲消除该竞争 */
uint16_t ADC_DMA_GetAvg(void);

/* 原始码值 → 毫伏，参考电压见 .c 中的 VREF_MV */
uint16_t ADC_ToMillivolt(uint16_t code);

#endif /* __ADC_DMA_H */
