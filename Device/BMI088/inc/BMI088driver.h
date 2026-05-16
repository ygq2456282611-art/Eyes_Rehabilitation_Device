#ifndef BMI088DRIVER_H
#define BMI088DRIVER_H

#include <stdint.h>
#include "main.h"

#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_WRITE_ACCEL_REG_NUM 6
#define BMI088_WRITE_GYRO_REG_NUM 6

#define BMI088_GYRO_DATA_READY_BIT 0
#define BMI088_ACCEL_DATA_READY_BIT 1
#define BMI088_ACCEL_TEMP_DATA_READY_BIT 2

#define BMI088_LONG_DELAY_TIME 80
#define BMI088_COM_WAIT_SENSOR_TIME 150

#define BMI088_ACCEL_IIC_ADDRESSE (0x18 << 1)
#define BMI088_GYRO_IIC_ADDRESSE (0x68 << 1)

#define BMI088_ACCEL_RANGE_3G
//#define BMI088_ACCEL_RANGE_6G
//#define BMI088_ACCEL_RANGE_12G
//#define BMI088_ACCEL_RANGE_24G

#define BMI088_GYRO_RANGE_2000
//#define BMI088_GYRO_RANGE_1000
//#define BMI088_GYRO_RANGE_500
//#define BMI088_GYRO_RANGE_250
//#define BMI088_GYRO_RANGE_125

#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f

/* Gyro output is rad/s in the async driver. */
#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f

typedef struct BMI088_RAW_DATA
{
    uint8_t status;
    int16_t accel[3];
    int16_t temp;
    int16_t gyro[3];
    uint32_t accel_timestamp_us;
    uint32_t gyro_timestamp_us;
    uint32_t temp_timestamp_us;
} bmi088_raw_data_t;

typedef struct BMI088_REAL_DATA
{
    uint8_t status;
    float accel[3];
    float temp;
    float gyro[3];
    float time;
    uint32_t accel_timestamp_us;
    uint32_t gyro_timestamp_us;
    uint32_t temp_timestamp_us;
} bmi088_real_data_t;

typedef struct BMI088_ATTITUDE
{
    float roll;
    float pitch;
    float yaw;
    float quat[4];
    float gyro_bias[3];
    float dt;
    uint32_t timestamp_us;
    uint8_t calibrated;
    uint8_t is_static;
} bmi088_attitude_t;

typedef struct BMI088_EULER_DATA
{
    float pitch;    /* head turn left/right, degrees */
    float roll;     /* nod forward/backward, degrees */
    float yaw;      /* lateral tilt, degrees */
} bmi088_euler_data_t;

typedef struct BMI088_DEBUG_DATA
{
    uint32_t acc_exti_count;
    uint32_t gyro_exti_count;
    uint32_t gyro_dma_count;
    uint32_t accel_dma_count;
    uint32_t temp_dma_count;
    uint32_t dma_error_count;
    uint8_t last_error;
    uint8_t spi_busy;
    uint8_t active_transaction;
    uint8_t gyro_pending;
    uint8_t accel_pending;
    uint8_t temp_pending;
    uint32_t gyro_timestamp_us;
    uint32_t accel_timestamp_us;
    uint32_t temp_timestamp_us;
} bmi088_debug_data_t;

enum
{
    BMI088_NO_ERROR = 0x00,
    BMI088_ACC_PWR_CTRL_ERROR = 0x01,
    BMI088_ACC_PWR_CONF_ERROR = 0x02,
    BMI088_ACC_CONF_ERROR = 0x03,
    BMI088_ACC_SELF_TEST_ERROR = 0x04,
    BMI088_ACC_RANGE_ERROR = 0x05,
    BMI088_INT1_IO_CTRL_ERROR = 0x06,
    BMI088_INT_MAP_DATA_ERROR = 0x07,
    BMI088_GYRO_RANGE_ERROR = 0x08,
    BMI088_GYRO_BANDWIDTH_ERROR = 0x09,
    BMI088_GYRO_LPM1_ERROR = 0x0A,
    BMI088_GYRO_CTRL_ERROR = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,

    BMI088_SELF_TEST_ACCEL_ERROR = 0x80,
    BMI088_SELF_TEST_GYRO_ERROR = 0x40,
    BMI088_NO_SENSOR = 0xFF,
};

uint8_t BMI088_init(void);
uint8_t bmi088_accel_init(void);
uint8_t bmi088_gyro_init(void);

void BMI088_AsyncStart(void);
void BMI088_Task(void);
void BMI088_EXTI_Callback(uint16_t GPIO_Pin);
void BMI088_GetLatestRaw(bmi088_raw_data_t *raw);
void BMI088_GetLatestFloat(float gyro[3], float accel[3], float *temperate, uint32_t *timestamp_us);
void BMI088_GetEuler(float *roll, float *pitch, float *yaw, float *dt, uint32_t *timestamp_us);
void BMI088_GetGyroBias(float bias[3]);
void BMI088_GetQuaternion(float quat[4], uint8_t relative);
void BMI088_GetDebug(bmi088_debug_data_t *debug);
uint8_t BMI088_IsCalibrated(void);
uint8_t BMI088_IsStatic(void);
void BMI088_ResetReference(void);
uint8_t BMI088_HasNewGyroSample(void);
void BMI088_ClearNewSampleFlag(void);
void BMI088_DMA_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void BMI088_DMA_ErrorCallback(SPI_HandleTypeDef *hspi);

void BMI088_euler_init(void);
void BMI088_read_euler(bmi088_euler_data_t *euler, float *temp);
void BMI088_read(float gyro[3], float accel[3], float *temperate);

#endif
