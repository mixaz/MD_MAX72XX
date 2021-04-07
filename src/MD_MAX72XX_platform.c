/*
 * Implementation for SPI transfer on STM32
 */
#include "MD_MAX72xx.h"
#include "MD_MAX72xx_lib.h"

#include <stm32f1xx_ll_spi.h>
#include <stm32f1xx_ll_gpio.h>

#define CS_SET() LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4)
#define CS_RESET() LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4)

void MD_MAX72XX_spiInit(MD_MAX72XX_t *m)
{
}

void MD_MAX72XX_spiSend(MD_MAX72XX_t *m)
{
    CS_SET();
    for (uint16_t i = 0; i < SPI_DATA_SIZE; i++)
    {
        while(!LL_SPI_IsActiveFlag_TXE(SPI1)) {}
        LL_SPI_TransmitData8 (SPI1, m->_spiData[i]);
        while(!LL_SPI_IsActiveFlag_RXNE(SPI1)) {}
        (void) SPI1->DR;
    }
    CS_RESET();
}

