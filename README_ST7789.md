# I2C1、I2C2 与 ST7789 接入说明

本次修改针对 STM32F103C8T6、YY-TFT154-SPI-8P / ST7789 240×240 屏幕。工程代码、CubeMX `I2C.ioc` 和 Keil `MDK-ARM/F1_balance_DMP.uvprojx` 同步修改。没有烧录开发板；屏幕点亮、颜色方向和供电仍需实际接线后验证。

## 1. 两路 I2C

| 总线 | SCL | SDA | 配置 | 当前用途 |
|---|---|---|---|---|
| I2C1 | PB8 | PB9 | 100kHz、7位地址、开漏，AFIO 重映射 | 扩展预留，初始化完成；尚无器件读写任务 |
| I2C2 | PB10 | PB11 | 100kHz、7位地址、开漏 | MPU6050 / MPU6500，保留原驱动及总线恢复逻辑 |

`main.c` 调用两路初始化；`i2c.c` 配置时钟和引脚；`i2c.h` 导出两路句柄。`balance.c` 仍使用 `hi2c2`，不需要把 MPU 换接到 I2C1。两组总线各自需要上拉到 3.3V；已有模块上拉时先核对现有电路。

## 2. 屏幕接线

按屏幕丝印接线，不按排针朝向猜顺序。屏幕 SCL/SDA 在这里代表 SPI SCK/MOSI。

| 屏幕引脚 | 接 STM32 / 电源 | 含义 |
|---|---|---|
| GND | GND | 与主控、电机驱动共地 |
| VCC | 3.3V | 先采用 3.3V 供电，不按其他品牌模块推定可接 5V |
| SCL | PB3 | SPI1 SCK |
| SDA | PB5 | SPI1 MOSI |
| RES | PA15 | 低电平复位 |
| DC | PA8 | 命令/数据选择 |
| CS | PA5 | 软件片选，低有效 |
| BLK | 常见高电平点亮版本接 3.3V | 本次不做 GPIO 调光；YY 板的具体背光要求以卖家资料为准 |

当前电机 STBY 为 **PB0**，PB5 已给屏幕数据线。如果驱动 STBY 实物仍接 PB5，需移到 PB0 后再使用此配置。PB1 仍是低有效启停按键。PB8/PB9 保留给 I2C1，不能再按早先建议把 PB9 当屏幕 CS。

屏幕没有 MISO，因此没有读取屏幕 ID/连线检测。日志 `LCD ... status=1` 仅表示本机 SPI/DMA 初始化和发送流程正常，不能证明屏幕已经接上或点亮。

## 3. CubeMX 配置

打开根目录 `I2C.ioc`，应看到：

- I2C1：PB8/PB9；I2C2：PB10/PB11；两者 Clock Speed 均 100000。
- SPI1：Transmit Only Master，PB3/PB5 重映射；8位、MSB first、软件 NSS；Mode 0（CPOL Low、CPHA 1Edge），Prescaler 4，72MHz / 4 = 18MHz。
- DMA：SPI1_TX 使用 DMA1_Channel3，Memory to Peripheral、Byte/Byte、内存地址递增、Normal、Low priority。
- NVIC：DMA1_Channel3 优先级 3；TIM1 更新中断优先级 2，USART1 优先级 1。
- PA5/PA8/PA15：LCD_CS/LCD_DC/LCD_RES，推挽输出，初始高；PB0：MOTOR_STBY；PC13：STATUS_LED；PC14：MPU_INT 预留输入。
- SYS 保留 Serial Wire，关闭 JTAG，继续使用 PA13/PA14 下载。`main.c` 在所有外设重映射后再次明确设置 SWD，避免 F1 MAPR 的 SWJ 位受到重映射操作影响。
- 初始化顺序中 GPIO 后先初始化 DMA，随后 SPI1，确保配置 DMA 通道前已打开 DMA 时钟。

CubeMX 的 Keep User Code 保持开启。屏幕应用代码放在 `Core/user`，主循环接入保存在 USER CODE 区域。重新生成后，确认 Keil 仍包含 `st7789.c`、`spi.c`、`dma.c`、`stm32f1xx_hal_spi.c`，再重新编译。

## 4. 新增代码怎么读

| 文件 | 作用 |
|---|---|
| `Core/Src/spi.c` / `Core/Inc/spi.h` | SPI1、PB3/PB5 重映射及 SPI TX DMA 初始化 |
| `Core/Src/dma.c` / `Core/Inc/dma.h` | DMA1 时钟、中断优先级 |
| `Core/user/st7789.c` | 屏幕复位、初始化命令、窗口设置、ASCII 渲染和 DMA 状态机 |
| `Core/user/st7789.h` | 显示快照、对外函数及面板方向/坐标偏移宏 |
| `Core/user/st7789_font.h` | 复用原工程的 6×8 ASCII 字体，放大为 12×16 |
| `Test/test_st7789.c` | 通过异步 DMA 和 LCD 命令模拟器测试实际驱动 |
| `Test/run_lcd_tests.py` | 本机编译运行屏幕测试 |

`ST7789_Init()` 只启动复位流程；`ST7789_Poll()` 在主循环中推进复位延时、命令和绘图，每次最多启动一次 DMA 或渲染 4 行。不调用阻塞式 `HAL_Delay` 或 `HAL_SPI_Transmit`。DMA 中断回调只设置完成/错误标志，SPI BSY 在主循环中检查。

初始化后先清屏，再约每 200ms 刷新状态、故障、角度、角速度、两轮编码器增量、两路真实 PWM 比较值及 STBY/按键状态。编码器显示单位仍是每个 5ms 采样周期的计数差，PWM 是 0～3599 比较计数，不是百分比。屏幕只读取控制状态，不能绕过启停、防倒或故障保护。

像素缓冲仅 240×4×2=1920 字节，不分配 115200 字节的全屏缓冲。像素按 RGB565 高字节先发。SPI1 TX DMA 由屏幕独占，不与其他代码混用 `HAL_SPI_Transmit_DMA`。若缺少 DMA 完成、DMA 出错或 SPI BSY 持续卡住，10ms 后停用屏幕传输；控制循环继续运行。

日志状态：0=初始化/清屏，1=本机发送流程正常，2=发送故障后停用。若只是显示方向或偏移不符，在 `st7789.h` 的 `ST7789_MADCTL`、`ST7789_X_OFFSET`、`ST7789_Y_OFFSET` 调整，修改后需重新编译；不要直接更换 I2C 引脚。

## 5. 验证与使用

代码已在 Keil ARM Compiler 5 工具链完整编译，现有控制测试 97086 项断言及新增屏幕模拟测试 142788 项断言通过。修改后的 `.ioc` 已由本机 CubeMX 6.6.1 实际加载、生成代码并导出引脚表；新增 SPI/DMA 初始化文件采用该次生成结果。实际板卡未烧录，屏幕未做实物点亮或示波器时序验证。

1. 核对 STBY 已接 PB0、PB1 按键接地有效，屏幕按上表接线。
2. 打开 `MDK-ARM/F1_balance_DMP.uvprojx`，Build/Rebuild 后下载；下载前按原工程操作固定车体，校准结束后仍按原设定自动运行。
3. 串口查看两路 I2C 的引脚提示及 LCD 状态。屏幕应显示与串口一致的状态、角度及 PWM。
4. 运行中的电机启停、防倒和恢复规则由原 `balance.c` 控制。屏幕数值为零时结合 STATE、FAULT、STBY 分析，不能只凭 PWM 一项判断。

参考：[STM32F103C8 数据手册](https://www.st.com/resource/en/datasheet/stm32f103c8.pdf)、[同类 1.54 英寸 ST7789 模块说明](https://www.lcdwiki.com/1.54inch_IPS_Module)。后者是同类模块参考，不能替代 YY 板的具体供电/背光说明。面板初始化参数沿用本机 `Desktop/STM_VSCode/st7789/Core/Src/st7789.c`。
