#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
#include <stddef.h>
#define UNUSED(x) ((void)(x))
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct { uint32_t ODR; } GPIO_TypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;
typedef enum { HAL_I2C_STATE_RESET=0, HAL_I2C_STATE_READY=0x20, HAL_I2C_STATE_BUSY=0x24 } HAL_I2C_StateTypeDef;
extern uint32_t test_i2c_busy_flag;
#define RESET 0U
#define I2C_FLAG_BUSY 2U
#define __HAL_I2C_GET_FLAG(bus,flag) test_i2c_busy_flag
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_I2C2_CLK_ENABLE() ((void)0)
#define GPIO_MODE_OUTPUT_OD 1U
#define GPIO_MODE_AF_OD 2U
#define GPIO_NOPULL 0U
#define GPIO_SPEED_FREQ_HIGH 3U
typedef struct { uint32_t CNT, CCR[4]; } MockTimer;
typedef struct { MockTimer *Instance; } TIM_HandleTypeDef;
typedef struct { void *Instance; } UART_HandleTypeDef;
typedef struct { void *Instance; } I2C_HandleTypeDef;
extern MockTimer mock_tim1, mock_tim2, mock_tim3, mock_tim4;
extern GPIO_TypeDef mock_gpio_b;
#define TIM1 (&mock_tim1)
#define GPIOB (&mock_gpio_b)
#define GPIO_PIN_0 (1U << 0)
#define GPIO_PIN_1 (1U << 1)
#define GPIO_PIN_5 (1U << 5)
#define GPIO_PIN_10 (1U << 10)
#define GPIO_PIN_11 (1U << 11)
#define GPIO_PIN_12 (1U << 12)
#define GPIO_PIN_13 (1U << 13)
#define GPIO_PIN_14 (1U << 14)
#define GPIO_PIN_15 (1U << 15)
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_ALL 60U
#define I2C_MEMADD_SIZE_8BIT 1U
#define __HAL_TIM_GET_COMPARE(h,ch) ((h)->Instance->CCR[(ch)/4U])
#define __HAL_TIM_GET_COUNTER(h) ((h)->Instance->CNT)
#define __HAL_TIM_SET_COMPARE(h,ch,val) ((h)->Instance->CCR[(ch)/4U]=(val))
static inline uint32_t __get_PRIMASK(void) { return 0U; }
static inline void __disable_irq(void) { }
static inline void __set_PRIMASK(uint32_t x) { (void)x; }
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *bus);
HAL_I2C_StateTypeDef HAL_I2C_GetState(I2C_HandleTypeDef *bus);
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *bus);
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *bus);
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *pins);
void HAL_Delay(uint32_t delay);
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef*,uint16_t,uint16_t,uint16_t,uint8_t*,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef*,uint16_t,uint16_t,uint16_t,uint8_t*,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef*,uint32_t);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef*,uint32_t);
void HAL_GPIO_WritePin(GPIO_TypeDef*,uint16_t,GPIO_PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef*,uint16_t);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef*,uint8_t*,uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef*,uint8_t*,uint16_t);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef*);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef*);
#endif
