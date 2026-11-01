#ifndef LCD_TEST_MAIN_H
#define LCD_TEST_MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct { uint32_t CR2, DR, flags, enabled; } SPI_TypeDef;
typedef struct DMA_HandleTypeDef DMA_HandleTypeDef;
struct DMA_HandleTypeDef {
    void (*XferCpltCallback)(DMA_HandleTypeDef *);
    void (*XferErrorCallback)(DMA_HandleTypeDef *);
    void (*XferHalfCpltCallback)(DMA_HandleTypeDef *);
};
typedef struct { SPI_TypeDef *Instance; } SPI_HandleTypeDef;
#define LCD_CS_GPIO_Port ((void *)1)
#define LCD_DC_GPIO_Port ((void *)1)
#define LCD_RES_GPIO_Port ((void *)1)
#define LCD_CS_Pin 1U
#define LCD_DC_Pin 2U
#define LCD_RES_Pin 4U
#define SPI_CR2_TXDMAEN 1U
#define SPI_FLAG_BSY 1U
#define RESET 0U
#define DMA_IT_TC 1U
#define DMA_IT_HT 2U
#define DMA_IT_TE 4U
#define CLEAR_BIT(REG,BITS) ((REG) &= ~(BITS))
#define SET_BIT(REG,BITS) ((REG) |= (BITS))
#define __HAL_SPI_ENABLE(h) ((h)->Instance->enabled=1)
#define __HAL_SPI_DISABLE(h) ((h)->Instance->enabled=0)
#define __HAL_SPI_GET_FLAG(h,f) ((h)->Instance->flags & (f))
#define __HAL_SPI_CLEAR_OVRFLAG(h) ((void)(h))
void test_disable_dma(void);
#define __HAL_DMA_DISABLE(h) test_disable_dma()
#define __HAL_DMA_DISABLE_IT(h,flags) ((void)(h))
uint32_t HAL_GetTick(void);
void HAL_GPIO_WritePin(void *, uint16_t, GPIO_PinState);
HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *, uint32_t, uint32_t, uint32_t);
#endif
