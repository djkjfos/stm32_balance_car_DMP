/*
 * motor_Init 的主机端单元测试：编译真实的 Core/user/motor.c，
 * 用确定性的 TIM4 / GPIO 桩记录调用顺序、参数和失败后的回滚行为。
 * 运行方式见 Test/run_motor_tests.py。
 */
#include "motor.h"
#include "tim.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

MockTimer mock_tim4;
GPIO_TypeDef mock_gpio_b;
TIM_HandleTypeDef htim4 = {&mock_tim4};

static unsigned checks;
static unsigned pwm_start_calls;
static unsigned stby_write_calls;
static unsigned stby_low_writes;
static uint32_t started_channels[4];
static uint32_t ccr_a_at_start[4];
static uint32_t ccr_b_at_start[4];
static unsigned stby_high_at_start[4];
static HAL_StatusTypeDef pwm_a_status;
static HAL_StatusTypeDef pwm_b_status;
static unsigned fail_b_side_effect;
static unsigned stby_high;

#define CHECK(x) do { ++checks; if (!(x)) { printf("FAIL line %d: %s\n", __LINE__, #x); exit(1); } } while (0)

/* 记录调用顺序与调用瞬间的引脚/比较值，用来验证“先停止再启动”的顺序。 */
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
    unsigned call;

    CHECK(timer == &htim4);
    call = pwm_start_calls;
    CHECK(call < 4U);
    started_channels[call] = channel;
    ccr_a_at_start[call] = mock_tim4.CCR[0];
    ccr_b_at_start[call] = mock_tim4.CCR[1];
    stby_high_at_start[call] = stby_high;
    ++pwm_start_calls;

    /* 定时器外设尚未初始化时，HAL 无法启动通道。 */
    if (timer->Instance == NULL) return HAL_ERROR;
    if (call == 0U) return pwm_a_status;
    /* 模拟 B 通道启动失败时外设已被部分激活，用于验证回滚。 */
    if (fail_b_side_effect && pwm_b_status != HAL_OK) {
        mock_tim4.CCR[1] = 1234U;
        mock_gpio_b.ODR |= STBY_PIN;
        stby_high = 1U;
    }
    return pwm_b_status;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    CHECK(port == GPIOB);
    CHECK(pin == STBY_PIN);
    ++stby_write_calls;
    if (state == GPIO_PIN_SET) {
        stby_high = 1U;
        mock_gpio_b.ODR |= pin;
    } else {
        stby_high = 0U;
        ++stby_low_writes;
        mock_gpio_b.ODR &= ~pin;
    }
}

static void reset_environment(HAL_StatusTypeDef a_status, HAL_StatusTypeDef b_status)
{
    htim4.Instance = &mock_tim4;
    mock_tim4.CCR[0] = 111U;
    mock_tim4.CCR[1] = 222U;
    mock_gpio_b.ODR = STBY_PIN;
    stby_high = 1U;

    pwm_start_calls = 0U;
    stby_write_calls = 0U;
    stby_low_writes = 0U;
    pwm_a_status = a_status;
    pwm_b_status = b_status;
    fail_b_side_effect = 0U;

    memset(started_channels, 0, sizeof(started_channels));
    memset(ccr_a_at_start, 0, sizeof(ccr_a_at_start));
    memset(ccr_b_at_start, 0, sizeof(ccr_b_at_start));
    memset(stby_high_at_start, 0, sizeof(stby_high_at_start));
}

static void expect_stopped(void)
{
    CHECK(stby_high == 0U);
    CHECK((mock_gpio_b.ODR & STBY_PIN) == 0U);
    CHECK(mock_tim4.CCR[0] == 0U);
    CHECK(mock_tim4.CCR[1] == 0U);
}

/* 正常路径：先拉低 STBY 并清零比较值，再按 A、B 顺序启动两路 PWM。 */
static void test_success_starts_both_channels_in_order(void)
{
    reset_environment(HAL_OK, HAL_OK);

    CHECK(motor_Init() == HAL_OK);
    CHECK(pwm_start_calls == 2U);
    CHECK(started_channels[0] == PWMA_CH);
    CHECK(started_channels[1] == PWMB_CH);

    /* 两次启动之前都必须已经处于停止状态。 */
    CHECK(stby_high_at_start[0] == 0U && stby_high_at_start[1] == 0U);
    CHECK(ccr_a_at_start[0] == 0U && ccr_b_at_start[0] == 0U);
    CHECK(ccr_a_at_start[1] == 0U && ccr_b_at_start[1] == 0U);
    CHECK(stby_write_calls == 1U && stby_low_writes == 1U);

    expect_stopped();

    /* 初始化不得使能驱动：此后的控制量不能产生任何输出。 */
    MotorA_SetSpeed(3000.0f);
    MotorB_SetSpeed(-3000.0f);
    expect_stopped();
}

/* A 通道启动失败：立即原样返回错误，不启动 B 通道。 */
static void test_channel_a_failure_aborts_immediately(void)
{
    const HAL_StatusTypeDef failures[] = {HAL_ERROR, HAL_BUSY, HAL_TIMEOUT};
    unsigned i;

    for (i = 0U; i < sizeof(failures) / sizeof(failures[0]); ++i) {
        reset_environment(failures[i], HAL_OK);

        CHECK(motor_Init() == failures[i]);
        CHECK(pwm_start_calls == 1U);
        CHECK(started_channels[0] == PWMA_CH);
        CHECK(stby_write_calls == 1U && stby_low_writes == 1U);
        expect_stopped();

        MotorA_SetSpeed(1000.0f);
        MotorB_SetSpeed(1000.0f);
        expect_stopped();
    }
}

/* B 通道启动失败：必须再次执行停止（回滚外设可能的部分激活）并返回错误。 */
static void test_channel_b_failure_rolls_back(void)
{
    const HAL_StatusTypeDef failures[] = {HAL_ERROR, HAL_BUSY, HAL_TIMEOUT};
    unsigned i;

    for (i = 0U; i < sizeof(failures) / sizeof(failures[0]); ++i) {
        reset_environment(HAL_OK, failures[i]);
        fail_b_side_effect = 1U;

        CHECK(motor_Init() == failures[i]);
        CHECK(pwm_start_calls == 2U);
        CHECK(started_channels[0] == PWMA_CH);
        CHECK(started_channels[1] == PWMB_CH);
        /* 首次停止 + 失败后的回滚停止。 */
        CHECK(stby_write_calls == 2U && stby_low_writes == 2U);
        expect_stopped();

        MotorA_SetSpeed(1000.0f);
        MotorB_SetSpeed(-1000.0f);
        expect_stopped();
    }
}

/* 定时器实例为空：HAL 返回错误，且 Motor_Disable 不得解引用空指针改写 CCR。 */
static void test_missing_timer_instance_fails_safely(void)
{
    reset_environment(HAL_OK, HAL_OK);
    htim4.Instance = NULL;

    CHECK(motor_Init() == HAL_ERROR);
    CHECK(pwm_start_calls == 1U);
    CHECK(started_channels[0] == PWMA_CH);
    CHECK(stby_write_calls == 1U && stby_low_writes == 1U);
    CHECK((mock_gpio_b.ODR & STBY_PIN) == 0U);
    /* 没有可用的定时器实例，比较值保持原样，说明没有被写入。 */
    CHECK(mock_tim4.CCR[0] == 111U);
    CHECK(mock_tim4.CCR[1] == 222U);

    htim4.Instance = &mock_tim4;
}

int main(void)
{
    test_success_starts_both_channels_in_order();
    test_channel_a_failure_aborts_immediately();
    test_channel_b_failure_rolls_back();
    test_missing_timer_instance_fails_safely();

    printf("PASS: %u assertions for motor_Init\n", checks);
    return 0;
}
