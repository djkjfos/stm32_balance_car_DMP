/**
  ******************************************************************************
  * @file    test_fputc.c
  * @brief   Unit tests for fputc() defined in Core/Src/main.c:66-70.
  *
  * fputc() is the stdio retarget that forwards a single character to USART1
  * through HAL_UART_Transmit(&huart1, temp, 1, 2). To test it in isolation the
  * target hardware is replaced by:
  *   - a recording mock of HAL_UART_Transmit(), and
  *   - a stub huart1 handle.
  *
  * Build & run on the host (GCC / Clang):
  *   gcc -std=c11 -Wall -Wextra -Werror -o test_fputc Test/test_fputc.c
  *   ./test_fputc
  *
  * Exit code is 0 when every check passes, 1 otherwise.
  ******************************************************************************
  */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------------- */
/* Minimal mock of the STM32 HAL surface that fputc() depends on.            */
/* ------------------------------------------------------------------------- */

typedef enum
{
    HAL_OK      = 0x00U,
    HAL_ERROR   = 0x01U,
    HAL_BUSY    = 0x02U,
    HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

typedef struct
{
    void *Instance;
} UART_HandleTypeDef;

/* Stub of the global handle declared in Core/Src/usart.c. */
UART_HandleTypeDef huart1;

/* Call recorder. */
static int                 g_call_count   = 0;
static UART_HandleTypeDef *g_last_huart   = NULL;
static uint16_t            g_last_size    = 0;
static uint32_t            g_last_timeout = 0;
static uint8_t             g_last_buf[8];
static size_t              g_last_buf_len = 0;
static HAL_StatusTypeDef   g_next_status  = HAL_OK;

static void mock_reset(void)
{
    g_call_count   = 0;
    g_last_huart   = NULL;
    g_last_size    = 0;
    g_last_timeout = 0;
    g_last_buf_len = 0;
    g_next_status  = HAL_OK;
    memset(g_last_buf, 0, sizeof(g_last_buf));
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart,
                                    const uint8_t      *pData,
                                    uint16_t            Size,
                                    uint32_t            Timeout)
{
    g_call_count++;
    g_last_huart   = huart;
    g_last_size    = Size;
    g_last_timeout = Timeout;

    if ((pData != NULL) && (Size <= sizeof(g_last_buf)))
    {
        memcpy(g_last_buf, pData, Size);
        g_last_buf_len = Size;
    }

    return g_next_status;
}

/* ------------------------------------------------------------------------- */
/* Unit under test - mirrors Core/Src/main.c:66-70 verbatim.                  */
/* ------------------------------------------------------------------------- */

int fputc(int ch, FILE *f)
{
    uint8_t temp[1] = {ch};
    HAL_UART_Transmit(&huart1, temp, 1, 2);
    return ch;
}

/* ------------------------------------------------------------------------- */
/* Tiny assertion harness (no external framework required).                   */
/* ------------------------------------------------------------------------- */

static int g_checks_run    = 0;
static int g_checks_failed = 0;
static int g_tests_run     = 0;
static int g_tests_failed  = 0;
static const char *g_current_test = "";

#define CHECK_TRUE(cond, msg)                                                  \
    do {                                                                       \
        g_checks_run++;                                                        \
        if (cond) {                                                            \
            printf("    [PASS] %s\n", (msg));                                  \
        } else {                                                               \
            printf("    [FAIL] %s  (line %d)\n", (msg), __LINE__);             \
            g_checks_failed++;                                                 \
        }                                                                      \
    } while (0)

#define CHECK_INT(actual, expected, msg)                                       \
    do {                                                                       \
        long _a = (long)(actual);                                              \
        long _e = (long)(expected);                                            \
        g_checks_run++;                                                        \
        if (_a == _e) {                                                        \
            printf("    [PASS] %s\n", (msg));                                  \
        } else {                                                               \
            printf("    [FAIL] %s: expected %ld, got %ld  (line %d)\n",        \
                   (msg), _e, _a, __LINE__);                                   \
            g_checks_failed++;                                                 \
        }                                                                      \
    } while (0)

#define RUN_TEST(fn)                                                           \
    do {                                                                       \
        int _before = g_checks_failed;                                         \
        g_current_test = #fn;                                                  \
        g_tests_run++;                                                         \
        printf("RUN  %s\n", #fn);                                              \
        mock_reset();                                                          \
        fn();                                                                  \
        if (g_checks_failed != _before) {                                      \
            g_tests_failed++;                                                  \
        }                                                                      \
    } while (0)

/* ------------------------------------------------------------------------- */
/* Test cases                                                                 */
/* ------------------------------------------------------------------------- */

/* Normal input: an ordinary printable character is forwarded once. */
static void test_transmits_single_normal_byte(void)
{
    int ret = fputc('A', stderr);

    CHECK_INT(ret, 'A', "returns the character that was written");
    CHECK_INT(g_call_count, 1, "HAL_UART_Transmit is called exactly once");
    CHECK_TRUE(g_last_huart == &huart1, "transmits on the huart1 handle");
    CHECK_INT(g_last_size, 1, "transmits exactly one byte");
    CHECK_INT(g_last_timeout, 2, "uses a 2 ms timeout");
    CHECK_INT(g_last_buf_len, 1, "one byte was captured by the mock");
    CHECK_INT(g_last_buf[0], 0x41, "the captured byte equals 'A' (0x41)");
}

/* Return value contract: fputc returns the value it was given. */
static void test_returns_original_char(void)
{
    int ret = fputc('z', stderr);
    CHECK_INT(ret, 'z', "return value equals the input character");
}

/* Boundary: NUL (0x00) must still be transmitted, not treated as "empty". */
static void test_null_character_boundary(void)
{
    int ret = fputc(0x00, stderr);

    CHECK_INT(ret, 0x00, "returns 0x00 for a NUL input");
    CHECK_INT(g_call_count, 1, "NUL still triggers one transmission");
    CHECK_INT(g_last_size, 1, "length is still 1 for NUL");
    CHECK_INT(g_last_buf[0], 0x00, "the transmitted byte is 0x00");
}

/* Boundary: largest value representable in a byte (0xFF). */
static void test_max_byte_boundary_0xFF(void)
{
    int ret = fputc(0xFF, stderr);

    CHECK_INT(ret, 0xFF, "returns 0xFF unchanged");
    CHECK_INT(g_last_buf[0], 0xFF, "the transmitted byte is 0xFF");
    CHECK_INT(g_last_size, 1, "length is still 1");
}

/* Truncation: values wider than 8 bits are narrowed to uint8_t for the wire,
   while the return value keeps the original int. */
static void test_truncates_values_above_255(void)
{
    int ret = fputc(0x1234, stderr);

    CHECK_INT(g_last_buf[0], 0x34, "only the low byte 0x34 is transmitted");
    CHECK_INT(ret, 0x1234, "the full-width value 0x1234 is returned");
}

/* Truncation boundary: exactly 256 wraps to 0x00. */
static void test_truncates_exactly_256_to_zero(void)
{
    int ret = fputc(256, stderr);

    CHECK_INT(g_last_buf[0], 0x00, "256 wraps to 0x00 on the wire");
    CHECK_INT(ret, 256, "the original value 256 is returned");
}

/* Negative input: -1 is converted to uint8_t 0xFF, but returned as -1. */
static void test_negative_char_wraps_to_uint8(void)
{
    int ret = fputc(-1, stderr);

    CHECK_INT(g_last_buf[0], 0xFF, "-1 converts to 0xFF on the wire");
    CHECK_INT(ret, -1, "the original value -1 is returned");
}

/* The FILE* argument is unused: a NULL stream must not change behaviour. */
static void test_ignores_file_pointer_when_null(void)
{
    int ret = fputc('Z', NULL);

    CHECK_INT(ret, 'Z', "still returns the character with a NULL FILE*");
    CHECK_INT(g_call_count, 1, "still transmits exactly once");
    CHECK_INT(g_last_buf[0], 0x5A, "transmits 'Z' (0x5A)");
}

/* Control characters (e.g. newline) are transmitted like any other byte. */
static void test_transmits_control_character(void)
{
    int ret = fputc('\n', stderr);

    CHECK_INT(ret, '\n', "returns the newline character");
    CHECK_INT(g_last_buf[0], 0x0A, "transmits LF (0x0A)");
    CHECK_INT(g_last_size, 1, "length is 1");
}

/* Contract check repeated across several inputs: size == 1 and timeout == 2. */
static void test_length_and_timeout_contract(void)
{
    const int samples[] = {0, 1, 127, 128, 200, 254, 255};
    size_t    i;
    int       all_size_ok    = 1;
    int       all_timeout_ok = 1;

    for (i = 0; i < (sizeof(samples) / sizeof(samples[0])); ++i)
    {
        mock_reset();
        (void)fputc(samples[i], stderr);

        if (g_last_size != 1)
        {
            all_size_ok = 0;
        }
        if (g_last_timeout != 2)
        {
            all_timeout_ok = 0;
        }
    }

    CHECK_TRUE(all_size_ok, "every call transmits exactly 1 byte");
    CHECK_TRUE(all_timeout_ok, "every call uses a 2 ms timeout");
}

/* The HAL result is ignored: an error status does not alter the return value
   and does not prevent the (single) transmission attempt. */
static void test_hal_error_status_does_not_change_return(void)
{
    int ret;

    g_next_status = HAL_ERROR;
    ret = fputc('E', stderr);

    CHECK_INT(ret, 'E', "returns the character even when HAL reports ERROR");
    CHECK_INT(g_call_count, 1, "the transmission was still attempted once");
    CHECK_INT(g_last_buf[0], 0x45, "the byte 'E' was handed to the HAL");
}

/* Same guarantee for HAL_BUSY and HAL_TIMEOUT. */
static void test_hal_busy_and_timeout_statuses(void)
{
    int ret_busy;
    int ret_timeout;

    g_next_status = HAL_BUSY;
    ret_busy = fputc('B', stderr);
    CHECK_INT(ret_busy, 'B', "returns the character when HAL reports BUSY");
    CHECK_INT(g_last_buf[0], 0x42, "the byte 'B' was handed to the HAL");

    mock_reset();
    g_next_status = HAL_TIMEOUT;
    ret_timeout = fputc('T', stderr);
    CHECK_INT(ret_timeout, 'T', "returns the character when HAL reports TIMEOUT");
    CHECK_INT(g_last_buf[0], 0x54, "the byte 'T' was handed to the HAL");
}

/* Every invocation performs exactly one transmission, including repeated use. */
static void test_exactly_one_transmission_per_call(void)
{
    int i;

    for (i = 0; i < 5; ++i)
    {
        (void)fputc('0' + i, stderr);
    }

    CHECK_INT(g_call_count, 5, "five fputc() calls produce five transmissions");
}

/* ------------------------------------------------------------------------- */
/* Runner                                                                     */
/* ------------------------------------------------------------------------- */

int main(void)
{
    printf("=== test_fputc ===\n");

    RUN_TEST(test_transmits_single_normal_byte);
    RUN_TEST(test_returns_original_char);
    RUN_TEST(test_null_character_boundary);
    RUN_TEST(test_max_byte_boundary_0xFF);
    RUN_TEST(test_truncates_values_above_255);
    RUN_TEST(test_truncates_exactly_256_to_zero);
    RUN_TEST(test_negative_char_wraps_to_uint8);
    RUN_TEST(test_ignores_file_pointer_when_null);
    RUN_TEST(test_transmits_control_character);
    RUN_TEST(test_length_and_timeout_contract);
    RUN_TEST(test_hal_error_status_does_not_change_return);
    RUN_TEST(test_hal_busy_and_timeout_statuses);
    RUN_TEST(test_exactly_one_transmission_per_call);

    printf("--------------------------------------------------\n");
    printf("tests : %d run, %d failed\n", g_tests_run, g_tests_failed);
    printf("checks: %d run, %d failed\n", g_checks_run, g_checks_failed);

    if (g_tests_failed == 0 && g_checks_failed == 0)
    {
        printf("RESULT: ALL TESTS PASSED\n");
        return 0;
    }

    printf("RESULT: FAILURES DETECTED\n");
    return 1;
}
