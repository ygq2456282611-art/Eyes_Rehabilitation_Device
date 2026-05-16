#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

#include <stdint.h>
#include "main.h"
#include "spi.h"

#define BMI088_USE_SPI
//#define BMI088_USE_IIC

void BMI088_GPIO_init(void);
void BMI088_com_init(void);
void BMI088_delay_ms(uint16_t ms);
void BMI088_delay_us(uint16_t us);
uint32_t BMI088_get_time_us(void);

#if defined(BMI088_USE_SPI)
void BMI088_ACCEL_NS_L(void);
void BMI088_ACCEL_NS_H(void);
void BMI088_GYRO_NS_L(void);
void BMI088_GYRO_NS_H(void);
uint8_t BMI088_read_write_byte(uint8_t reg);
HAL_StatusTypeDef BMI088_spi_transmit_receive_dma(uint8_t *tx_data, uint8_t *rx_data, uint16_t size);
HAL_StatusTypeDef BMI088_spi_abort_dma(void);
#elif defined(BMI088_USE_IIC)

#endif

#endif
