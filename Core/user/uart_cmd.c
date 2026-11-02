/*
 * USART1 命令、遥测变量和中断收发队列。硬件参数由 Core/Src/usart.c 配置。
 * 接收中断只存字节，主循环按行解析命令；发送先入队，由发送完成中断继续搬运。
 * 这样等待串口传输的时间不会逐字节阻塞 5ms 控制任务；格式化本身仍会消耗 CPU。
 */
#include "uart_cmd.h"
#include "balance.h"
#include "pid.h"
#include "motor.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

/*
 * 这些变量供 balance.c 更新、main.c 输出到上位机：
 * angle=滤波角度（校准时为重力角），raw_gx=已扣零偏的角速度，单位度和度/秒。
 * speedA/B=两轮本周期增量，speed_avg=平均值，speed_filt=平均值低通滤波结果。
 * balance_pwm=直立环控制量；gyro_filt 当前只保留变量，没有参与现行滤波或控制。
 */
volatile float angle, gyro_filt, speed_filt, speedA, speedB;
volatile float speed_avg, balance_pwm, raw_gx;
#define RX_SIZE 256U
#define TX_SIZE 1024U
static UART_HandleTypeDef *command_uart;
static uint8_t rx_byte, tx_byte;
static volatile uint8_t rx_overflow, tx_busy;
/*
 * 环形队列：head 指向下一次写入位置，tail 指向下一次读取位置。
 * 索引到数组末尾后取模回到开头；预留一个空位，用 head==tail 表示空。
 * 中断和主循环共享索引，所以使用 volatile；涉及组合更新时仍需短临界区保护。
 */
static volatile uint16_t rx_head, rx_tail, tx_head, tx_tail;
static uint8_t rx_data[RX_SIZE], tx_data[TX_SIZE];
static char frame[128];
static uint16_t frame_length;
static uint8_t drop_line;

/*
 * 如果串口空闲且队列非空，就启动 1 字节中断发送；完成回调会再次调用它。
 * tx_busy 防止同一时刻重复开启发送；HAL 启动失败时保留字节供后续重试。
 */
static void tx_kick(void)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    if (command_uart != NULL && !tx_busy && tx_head != tx_tail) {
        tx_byte = tx_data[tx_tail];
        tx_busy = 1U;
        if (HAL_UART_Transmit_IT(command_uart, &tx_byte, 1U) == HAL_OK)
            tx_tail = (uint16_t)((tx_tail + 1U) % TX_SIZE);
        else tx_busy = 0U;
    }
    __set_PRIMASK(saved);
}

/*
 * 绑定命令串口、清空队列，并先挂起一次 1 字节接收，之后由接收回调持续续接。
 */
HAL_StatusTypeDef UART_Start(UART_HandleTypeDef *uart)
{
    command_uart = uart;
    rx_head = rx_tail = tx_head = tx_tail = 0U;
    rx_overflow = tx_busy = drop_line = 0U;
    frame_length = 0U;
    return HAL_UART_Receive_IT(uart, &rx_byte, 1U);
}

/*
 * 单字节入队接口，主要供 main.c 的 fputc 使用。返回 0 表示队列已满或串口未绑定。
 */
uint8_t UART_SendByte(uint8_t byte)
{
    uint16_t next = (uint16_t)((tx_head + 1U) % TX_SIZE);
    if (command_uart == NULL || next == tx_tail) return 0U;
    tx_data[tx_head] = byte;
    tx_head = next;
    tx_kick();
    return 1U;
}

/*
 * 把一条消息格式化到局部缓冲，再在容量够时整条入队。
 * 消息过长或队列放不下时丢弃整条，避免将半条日志混入六通道数据。
 * 这里不是直接等待 HAL_UART_Transmit 把整条内容发完。
 */
void UART_Printf(const char *format, ...)
{
    char text[160];
    va_list arguments;
    uint16_t head, used;
    int length;
    uint32_t saved;
    va_start(arguments, format);
    length = vsnprintf(text, sizeof(text), format, arguments);
    va_end(arguments);
    if (length <= 0 || length >= (int)sizeof(text) || command_uart == NULL) return;
    saved = __get_PRIMASK();
    __disable_irq();
    head = tx_head;
    used = (uint16_t)((head + TX_SIZE - tx_tail) % TX_SIZE);
    if ((uint16_t)length < TX_SIZE - used) {
        for (int i = 0; i < length; ++i) {
            tx_data[head] = (uint8_t)text[i];
            head = (uint16_t)((head + 1U) % TX_SIZE);
        }
        tx_head = head;
    }
    __set_PRIMASK(saved);
    tx_kick();
}

/*
 * 发送 1 字节完成后，清除忙标志并继续发送队列中的下一个字节。
 * 这是 HAL 中断回调，不需要在 main 中主动调用。
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == command_uart) { tx_busy = 0U; tx_kick(); }
}

/*
 * 接收中断只把字节写进 RX 队列，并立即续接下一字节接收。
 * 不在中断里做 strtof、PID 参数修改或 IMU 重新初始化。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    uint16_t next;
    if (uart != command_uart) return;
    next = (uint16_t)((rx_head + 1U) % RX_SIZE);
    if (next == rx_tail) rx_overflow = 1U;
    else { rx_data[rx_head] = rx_byte; rx_head = next; }
    if (HAL_UART_Receive_IT(uart, &rx_byte, 1U) != HAL_OK) rx_overflow = 1U;
}

/*
 * 标记接收流可能不完整，让主循环丢弃损坏行，再尝试恢复后续接收。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart != command_uart) return;
    rx_overflow = 1U;
    (void)HAL_UART_Receive_IT(uart, &rx_byte, 1U);
}

/*
 * 主循环把 RX 字节组装为一行：忽略 CR，遇到真实 LF 才认为命令完成。
 * 过长、含零字节或溢出的行一直丢弃到下一换行，防止执行残留的半条指令。
 */
uint8_t rx_line_available(void)
{
    uint8_t byte;
    if (rx_overflow) {
        uint32_t saved = __get_PRIMASK();
        __disable_irq();
        rx_tail = rx_head; rx_overflow = 0U;
        __set_PRIMASK(saved);
        frame_length = 0U; drop_line = 1U;
        UART_Printf("ERR RX overflow; resend after newline\r\n");
    }
    while (rx_head != rx_tail) {
        byte = rx_data[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) % RX_SIZE);
        if (byte == '\r') continue;
        if (byte == '\n') {
            if (drop_line) { drop_line = 0U; frame_length = 0U; continue; }
            if (frame_length == 0U) continue;
            frame[frame_length] = '\0'; frame_length = 0U;
            return 1U;
        }
        if (drop_line) continue;
        if (byte == 0U || frame_length >= sizeof(frame) - 1U) {
            frame_length = 0U; drop_line = 1U;
            UART_Printf("ERR invalid/long command\r\n");
            continue;
        }
        frame[frame_length++] = (char)byte;
    }
    return 0U;
}

/* Show the bounded original frame on parse failure, before tokenization changed it.
 * CR/LF framing has already been removed by rx_line_available. */
/*
 * 解析失败时打印实际收到的字节，用于区分 ARM 1 文本、真正的十六进制字节，
 * 以及误发的“41 52 4D...”这类十六进制字符文本。该函数只诊断，不修改命令。
 */
static void print_received_frame(const uint8_t *bytes, uint16_t count, uint16_t total)
{
    static const char digits[] = "0123456789ABCDEF";
    char hex[96];
    hex[0] = '\0';
    for (uint16_t i = 0U; i < count; ++i) {
        hex[i * 3U] = digits[bytes[i] >> 4];
        hex[i * 3U + 1U] = digits[bytes[i] & 15U];
        hex[i * 3U + 2U] = i + 1U < count ? ' ' : '\0';
    }
    UART_Printf("RX FRAME LEN=%u HEX=%s%s\r\n", (unsigned)total, hex, total > count ? " ..." : "");
}

/*
 * 一行命令的统一格式为 NAME 空格 数值，例如 ARM 1、DKP 60、STAT 0。
 * 先检查数字完整且有限，再处理启停/校准/状态或参数设置；非法输入不会执行。
 * 收到 ARM 1 后仍由 balance.c 审核是否允许启动，不直接拉高 STBY。
 */
void Frame_Process(void)
{
    char *value_text = frame, *end;
    float value;
    uint8_t received[32];
    uint16_t received_length = (uint16_t)strlen(frame);
    uint16_t preview_length = received_length < sizeof(received) ? received_length : (uint16_t)sizeof(received);
    memcpy(received, frame, preview_length);
    while (*value_text && !isspace((unsigned char)*value_text)) ++value_text;
    if (!*value_text) {
        UART_Printf("ERR use NAME value followed by LF\r\n");
        print_received_frame(received, preview_length, received_length);
        return;
    }
    *value_text++ = '\0';
    while (isspace((unsigned char)*value_text)) ++value_text;
    value = strtof(value_text, &end);
    while (isspace((unsigned char)*end)) ++end;
    if (end == value_text || *end || !isfinite(value)) {
        UART_Printf("ERR invalid number CMD='%.12s' VALUE='%.32s'\r\n", frame, value_text);
        print_received_frame(received, preview_length, received_length);
        return;
    }
    if (!strcmp(frame, "ARM")) {
        if (value != 0.0f && value != 1.0f) {
            UART_Printf("ERR ARM value must be 0 or 1\r\n");
            print_received_frame(received, preview_length, received_length);
        } else if (!Balance_Arm(value == 1.0f)) {
            UART_Printf("ERR ARM: require READY, upright and fresh sensor\r\n");
            print_received_frame(received, preview_length, received_length);
            Balance_PrintDiagnostics();
        } else UART_Printf("ARM %u\r\n", (unsigned)(value == 1.0f));
        return;
    }
    if (!strcmp(frame, "CAL")) {
        if (value != 1.0f) { UART_Printf("ERR use CAL 1\r\n"); return; }
        UART_Printf(Balance_Calibrate() == HAL_OK ? "CAL started\r\n" : "ERR MPU6050\r\n");
        return;
    }
    if (!strcmp(frame, "STAT")) {
        UART_Printf("STATE=%u FAULT=%u KP=%.2f KD=%.2f MDB=%.0f\r\n",
                    (unsigned)balance_state, (unsigned)balance_fault,
                    pid_balance.kp, pid_balance.kd, MDB);
        Balance_PrintDiagnostics();
        return;
    }
    /*
     * 把命令名映射到参数地址，最后用同一段代码检查范围并赋值。
     * DKP/DKD 是直立增益，VKP/VKI 是速度增益，PKP 是位置增量，TKP 是转向增益，MDB 是死区补偿。
     * ASP 是目标平均轮速，TAG 是目标两轮速度差；这些修改只保存在 RAM，复位不保留。
     */
    float *parameter = NULL;
    float limit = 2000.0f;
    if (!strcmp(frame, "DKP")) parameter = &pid_balance.kp;
    else if (!strcmp(frame, "DKD")) parameter = &pid_balance.kd;
    else if (!strcmp(frame, "VKP")) { parameter = &pid_speed.kp; limit = 20.0f; }
    else if (!strcmp(frame, "VKI")) { parameter = &pid_speed.ki; limit = 20.0f; }
    else if (!strcmp(frame, "PKP")) { parameter = &pid_position.kp; limit = 0.002f; }
    else if (!strcmp(frame, "PLIM")) { parameter = &pid_position.output_max; limit = 25.0f; }
    else if (!strcmp(frame, "TKP")) parameter = &pid_turn.kp;
    else if (!strcmp(frame, "ASP")) { parameter = &target_speed; limit = 500.0f; }
    else if (!strcmp(frame, "TAG")) { parameter = &target_turn; limit = 500.0f; }
    else if (!strcmp(frame, "MDB")) { parameter = &MDB; limit = PWM_MAX; }
    if (parameter == NULL) { UART_Printf("ERR unknown command\r\n"); return; }
    if (fabsf(value) > limit || ((parameter == &MDB
        || parameter == &pid_position.kp || parameter == &pid_position.output_max) && value < 0.0f)) {
        UART_Printf("ERR value out of range\r\n"); return;
    }
    *parameter = value;
    UART_Printf("%s set to %.4f\r\n", frame, value);
}

/*
 * 主循环处理已经收到的完整命令，并推动尚未发送的队列；不等待用户输入。
 */
void UART_Poll(void)
{
    while (rx_line_available()) Frame_Process();
    tx_kick();
}
