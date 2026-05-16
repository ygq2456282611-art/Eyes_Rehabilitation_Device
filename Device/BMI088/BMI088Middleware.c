#include "BMI088Middleware.h"

#define BMI088_USING_SPI_UNIT hspi2

static uint32_t bmi088_cpu_mhz = 0U;
static uint8_t bmi088_dwt_ready = 0U;

static void BMI088_DWT_Init(void)
{
    if (bmi088_dwt_ready != 0U)
    {
        return;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    bmi088_cpu_mhz = HAL_RCC_GetSysClockFreq() / 1000000U;
    if (bmi088_cpu_mhz == 0U)
    {
        bmi088_cpu_mhz = 1U;
    }

    bmi088_dwt_ready = 1U;
}

void BMI088_GPIO_init(void)
{
}

void BMI088_com_init(void)
{
    BMI088_DWT_Init();
}

uint32_t BMI088_get_time_us(void)
{
    if (bmi088_dwt_ready == 0U)
    {
        BMI088_DWT_Init();
    }

    return DWT->CYCCNT / bmi088_cpu_mhz;
}

void BMI088_delay_ms(uint16_t ms)
{
    while (ms-- != 0U)
    {
        BMI088_delay_us(1000U);
    }
}

void BMI088_delay_us(uint16_t us)
{
    uint32_t start_us = BMI088_get_time_us();

    while ((uint32_t)(BMI088_get_time_us() - start_us) < (uint32_t)us)
    {
    }
}

void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);
}

void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
}

void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data = 0U;

    (void)HAL_SPI_TransmitReceive(&BMI088_USING_SPI_UNIT, &txdata, &rx_data, 1U, 1000U);
    return rx_data;
}

HAL_StatusTypeDef BMI088_spi_transmit_receive_dma(uint8_t *tx_data, uint8_t *rx_data, uint16_t size)
{
    return HAL_SPI_TransmitReceive_DMA(&BMI088_USING_SPI_UNIT, tx_data, rx_data, size);
}

HAL_StatusTypeDef BMI088_spi_abort_dma(void)
{
    return HAL_SPI_DMAStop(&BMI088_USING_SPI_UNIT);
}
