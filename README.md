# stm32-datalogger

STM32F103 + FreeRTOS 多传感器采集节点 + Python 上位机

## 项目简介

基于 STM32F103 的多通道传感器数据采集节点：
FreeRTOS 任务调度，UART 上传数据帧，Python 上位机接收、解析、绘图与存储。

**验收标准**：连续运行 2 小时零丢帧。

## 目录结构

| 目录 | 内容 |
|---|---|
| `firmware/` | STM32 固件工程（CubeMX + Keil，HAL 库 + FreeRTOS） |
| `host/` | Python 上位机（串口接收 / 解析 / 可视化） |
| `docs/` | 设计文档、协议定义、调试记录 |

## 进度

- [x] W0：仓库搭建、环境确认
- [ ] W1：CubeMX 工程 + FreeRTOS 移植，LED/串口跑通
- [ ] W2：多传感器采集（ADC + I2C）+ 数据帧协议
- [ ] W3：Python 上位机接收与解析
- [ ] W4：上位机绘图与存储 + 稳定性测试
- [ ] W5-W6：加固、文档、简历素材整理

## 技术栈

- 主控：STM32F103C8T6
- RTOS：FreeRTOS
- 工具链：CubeMX + Keil5 + ST-Link
- 上位机：Python（pyserial + matplotlib）
