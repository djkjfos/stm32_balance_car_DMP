USART1 显示通信（STM32F103 / ESP32-S3）

CLion/CMake 工程已接入 UART1 状态发送，USART2 调参和遥测保留。STM32 本机屏幕调用已移除，原 st7789 源码仍保留。ESP32 接收解码器尚未接入。
接线：PA9 USART1_TX -> ESP32 RX，PA10 USART1_RX <- ESP32 TX，两板 GND 共地。115200 8N1，双方 3.3V 逻辑。
main 每 200 ms 非阻塞发送状态；串口忙时丢弃本帧。接收只提供原始字节缓冲，不执行控制命令。HAL 串口回调在 uart_cmd.c 分发，不能重复定义。

STM32 发出的固定 32 字节帧（整数小端）：
0..1: A5 5A；2: 版本 01；3: 载荷长度 26（十进制）；4..5: 帧序号 uint16。
6..9: angle int32 *100；10..13: raw_gx int32 *100；14..17: speedA int32 *100；18..21: speedB int32 *100。
22..23: PWM A uint16；24..25: PWM B uint16；26: state；27: fault；28: STBY；29: key。
30..31: CRC-16/CCITT-FALSE 小端，计算偏移 2..29 共 28 字节；初值 FFFF，多项式 1021，无反射，xorout 0。
ESP32 需要验证同步头、版本、长度、CRC；长时间未收到有效帧应显示通信中断。
仅 CMake 工程已同步；Keil/EWARM、CubeMX 配置未同步，CubeMX 重新生成后可能重新加入 SPI 初始化，应复查。未进行双板或电机实测。
该车可能上电校准后自动启动，首次联调时固定车身并使车轮离地。
