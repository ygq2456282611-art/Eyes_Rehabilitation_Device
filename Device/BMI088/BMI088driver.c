#include "BMI088driver.h"

#include <math.h>
#include <string.h>

#include "BMI088Middleware.h"
#include "BMI088reg.h"

#define BMI088_DUMMY_BYTE 0x55U
#define BMI088_TEMP_READ_DIV 50U
#define BMI088_RAD_TO_DEG 57.2957795f
#define BMI088_GRAVITY_NORM 9.80665f
#define BMI088_STATIC_GYRO_NORM_THRESHOLD 0.08f
#define BMI088_STATIC_ACCEL_NORM_THRESHOLD 1.0f
#define BMI088_BIAS_CALIB_SAMPLES 800U
#define BMI088_BIAS_TRACK_ALPHA 0.0025f
#define BMI088_AHRS_CORRECTION_GAIN 2.8f
#define BMI088_GYRO_FALLBACK_INTERVAL_US 3000U
#define BMI088_ACCEL_FALLBACK_INTERVAL_US 3000U

float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

#if defined(BMI088_USE_SPI)
#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }

#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_read_write_byte((reg) | 0x80U);  \
        BMI088_read_write_byte(BMI088_DUMMY_BYTE); \
        (data) = BMI088_read_write_byte(BMI088_DUMMY_BYTE); \
        BMI088_ACCEL_NS_H();                    \
    }

#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }

#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }

static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
#elif defined(BMI088_USE_IIC)

#endif

typedef enum
{
    BMI088_DMA_TRANS_NONE = 0,
    BMI088_DMA_TRANS_GYRO,
    BMI088_DMA_TRANS_ACCEL,
    BMI088_DMA_TRANS_TEMP
} bmi088_dma_transaction_t;

static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
{
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
    {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
    {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}
};

static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
{
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
    {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}
};

static volatile uint8_t bmi088_async_started = 0U;
static volatile uint8_t bmi088_spi_busy = 0U;
static volatile uint8_t bmi088_gyro_pending = 0U;
static volatile uint8_t bmi088_accel_pending = 0U;
static volatile uint8_t bmi088_temp_pending = 0U;
static volatile uint8_t bmi088_new_gyro_sample = 0U;
static volatile uint8_t bmi088_last_error = 0U;
static volatile uint32_t bmi088_accel_sample_count = 0U;
static volatile uint32_t bmi088_last_gyro_update_us = 0U;
static volatile uint32_t bmi088_bias_calib_count = 0U;
static volatile bmi088_dma_transaction_t bmi088_active_transaction = BMI088_DMA_TRANS_NONE;
static volatile uint32_t bmi088_acc_exti_count = 0U;
static volatile uint32_t bmi088_gyro_exti_count = 0U;
static volatile uint32_t bmi088_gyro_dma_count = 0U;
static volatile uint32_t bmi088_accel_dma_count = 0U;
static volatile uint32_t bmi088_temp_dma_count = 0U;
static volatile uint32_t bmi088_dma_error_count = 0U;

static volatile bmi088_raw_data_t bmi088_raw_cache;
static volatile bmi088_real_data_t bmi088_real_cache;
static volatile bmi088_attitude_t bmi088_attitude_cache;
static volatile float bmi088_quat_now[4];
static volatile float bmi088_quat_ref[4];
static volatile float bmi088_gyro_bias[3];
static volatile float bmi088_gyro_bias_sum[3];
static volatile uint8_t bmi088_attitude_calibrated = 0U;
static volatile uint8_t bmi088_is_static = 0U;

static uint8_t bmi088_tx_buffer[8];
static uint8_t bmi088_rx_buffer[8];

static uint32_t BMI088_EnterCritical(void);
static void BMI088_ExitCritical(uint32_t primask);
static void BMI088_ResetRuntimeState(void);
static void BMI088_try_start_next_transaction(void);
static void BMI088_restore_pending_for_transaction(bmi088_dma_transaction_t transaction);
static HAL_StatusTypeDef BMI088_start_transaction(bmi088_dma_transaction_t transaction);
static void BMI088_prepare_gyro_frame(uint16_t *size);
static void BMI088_prepare_accel_frame(uint16_t *size);
static void BMI088_prepare_temp_frame(uint16_t *size);
static void BMI088_parse_gyro_frame(uint32_t timestamp_us);
static void BMI088_parse_accel_frame(uint32_t timestamp_us);
static void BMI088_parse_temp_frame(uint32_t timestamp_us);
static void BMI088_update_attitude(uint32_t timestamp_us);
static void BMI088_quat_identity(volatile float quat[4]);
static void BMI088_quat_copy(float dst[4], const volatile float src[4]);
static void BMI088_quat_multiply(float out[4], const float a[4], const float b[4]);
static void BMI088_quat_conjugate(float out[4], const float quat[4]);
static void BMI088_quat_normalize(float quat[4]);
static void BMI088_quat_integrate(float quat[4], const float gyro[3], float dt);
static void BMI088_quat_from_accel(float quat[4], const float accel[3]);
static void BMI088_roll_pitch_from_accel(const float accel[3], float *roll, float *pitch);
static void BMI088_quat_from_euler(float quat[4], float roll, float pitch, float yaw);
static void BMI088_quat_slerp(float out[4], const float from[4], const float to[4], float t);
static void BMI088_relative_quat_from_state(float quat_rel[4]);
static void BMI088_euler_from_quat(const float quat[4], float *roll, float *pitch, float *yaw);
static float BMI088_vector_norm3(const float vec[3]);
static void BMI088_update_static_and_bias(const float gyro_meas[3], const float accel[3]);
static void BMI088_refresh_attitude_cache(uint32_t timestamp_us, float dt);

uint8_t BMI088_init(void)
{
    uint8_t error = BMI088_NO_ERROR;

    BMI088_GPIO_init();
    BMI088_com_init();

    error |= bmi088_accel_init();
    error |= bmi088_gyro_init();

    return error;
}

uint8_t bmi088_accel_init(void)
{
    uint8_t res = 0U;
    uint8_t write_reg_num = 0U;

    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    for (write_reg_num = 0U; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {
        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}

uint8_t bmi088_gyro_init(void)
{
    uint8_t write_reg_num = 0U;
    uint8_t res = 0U;

    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    for (write_reg_num = 0U; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {
        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}

void BMI088_AsyncStart(void)
{
    uint32_t primask = 0U;

    __HAL_GPIO_EXTI_CLEAR_IT(ACC_INT_Pin);
    __HAL_GPIO_EXTI_CLEAR_IT(GYRO_INT_Pin);

    primask = BMI088_EnterCritical();
    BMI088_ResetRuntimeState();
    bmi088_async_started = 1U;
    bmi088_gyro_pending = 1U;
    bmi088_accel_pending = 1U;
    bmi088_temp_pending = 1U;
    BMI088_ExitCritical(primask);

    BMI088_try_start_next_transaction();
}

void BMI088_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t primask = 0U;

    if (bmi088_async_started == 0U)
    {
        return;
    }

    primask = BMI088_EnterCritical();

    if (GPIO_Pin == GYRO_INT_Pin)
    {
        bmi088_gyro_exti_count++;
        bmi088_gyro_pending = 1U;
    }
    else if (GPIO_Pin == ACC_INT_Pin)
    {
        bmi088_acc_exti_count++;
        bmi088_accel_pending = 1U;
    }
    else
    {
        BMI088_ExitCritical(primask);
        return;
    }

    BMI088_ExitCritical(primask);
    BMI088_try_start_next_transaction();
}

void BMI088_Task(void)
{
    uint32_t now_us = 0U;
    uint32_t gyro_age_us = 0U;
    uint32_t accel_age_us = 0U;
    uint32_t primask = 0U;

    if (bmi088_async_started == 0U)
    {
        return;
    }

    now_us = BMI088_get_time_us();

    primask = BMI088_EnterCritical();

    if (bmi088_real_cache.gyro_timestamp_us == 0U)
    {
        bmi088_gyro_pending = 1U;
    }
    else
    {
        gyro_age_us = now_us - bmi088_real_cache.gyro_timestamp_us;
        if (gyro_age_us >= BMI088_GYRO_FALLBACK_INTERVAL_US)
        {
            bmi088_gyro_pending = 1U;
        }
    }

    if (bmi088_real_cache.accel_timestamp_us == 0U)
    {
        bmi088_accel_pending = 1U;
    }
    else
    {
        accel_age_us = now_us - bmi088_real_cache.accel_timestamp_us;
        if (accel_age_us >= BMI088_ACCEL_FALLBACK_INTERVAL_US)
        {
            bmi088_accel_pending = 1U;
        }
    }

    BMI088_ExitCritical(primask);

    BMI088_try_start_next_transaction();
}

void BMI088_GetLatestRaw(bmi088_raw_data_t *raw)
{
    uint32_t primask = 0U;
    int index = 0;

    if (raw == 0)
    {
        return;
    }

    primask = BMI088_EnterCritical();

    raw->status = bmi088_raw_cache.status;
    raw->temp = bmi088_raw_cache.temp;
    raw->accel_timestamp_us = bmi088_raw_cache.accel_timestamp_us;
    raw->gyro_timestamp_us = bmi088_raw_cache.gyro_timestamp_us;
    raw->temp_timestamp_us = bmi088_raw_cache.temp_timestamp_us;

    for (index = 0; index < 3; index++)
    {
        raw->accel[index] = bmi088_raw_cache.accel[index];
        raw->gyro[index] = bmi088_raw_cache.gyro[index];
    }

    BMI088_ExitCritical(primask);
}

void BMI088_GetLatestFloat(float gyro[3], float accel[3], float *temperate, uint32_t *timestamp_us)
{
    uint32_t primask = 0U;
    int index = 0;
    uint32_t latest_timestamp = 0U;

    primask = BMI088_EnterCritical();

    if (gyro != 0)
    {
        for (index = 0; index < 3; index++)
        {
            gyro[index] = bmi088_real_cache.gyro[index];
        }
    }

    if (accel != 0)
    {
        for (index = 0; index < 3; index++)
        {
            accel[index] = bmi088_real_cache.accel[index];
        }
    }

    if (temperate != 0)
    {
        *temperate = bmi088_real_cache.temp;
    }

    latest_timestamp = bmi088_real_cache.gyro_timestamp_us;
    if (latest_timestamp == 0U)
    {
        latest_timestamp = bmi088_real_cache.accel_timestamp_us;
    }

    if (timestamp_us != 0)
    {
        *timestamp_us = latest_timestamp;
    }

    BMI088_ExitCritical(primask);
}

void BMI088_GetEuler(float *roll, float *pitch, float *yaw, float *dt, uint32_t *timestamp_us)
{
    uint32_t primask = 0U;

    primask = BMI088_EnterCritical();

    if (roll != 0)
    {
        *roll = bmi088_attitude_cache.roll * BMI088_RAD_TO_DEG;
    }

    if (pitch != 0)
    {
        *pitch = bmi088_attitude_cache.pitch * BMI088_RAD_TO_DEG;
    }

    if (yaw != 0)
    {
        *yaw = bmi088_attitude_cache.yaw * BMI088_RAD_TO_DEG;
    }

    if (dt != 0)
    {
        *dt = bmi088_attitude_cache.dt;
    }

    if (timestamp_us != 0)
    {
        *timestamp_us = bmi088_attitude_cache.timestamp_us;
    }

    BMI088_ExitCritical(primask);
}

void BMI088_GetGyroBias(float bias[3])
{
    uint32_t primask = 0U;
    int index = 0;

    if (bias == 0)
    {
        return;
    }

    primask = BMI088_EnterCritical();
    for (index = 0; index < 3; index++)
    {
        bias[index] = bmi088_gyro_bias[index];
    }
    BMI088_ExitCritical(primask);
}

void BMI088_GetQuaternion(float quat[4], uint8_t relative)
{
    uint32_t primask = 0U;
    int index = 0;
    float quat_local[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    if (quat == 0)
    {
        return;
    }

    primask = BMI088_EnterCritical();
    if (relative != 0U)
    {
        BMI088_relative_quat_from_state(quat_local);
    }
    else
    {
        for (index = 0; index < 4; index++)
        {
            quat_local[index] = bmi088_quat_now[index];
        }
    }
    BMI088_ExitCritical(primask);

    for (index = 0; index < 4; index++)
    {
        quat[index] = quat_local[index];
    }
}

void BMI088_GetDebug(bmi088_debug_data_t *debug)
{
    uint32_t primask = 0U;

    if (debug == 0)
    {
        return;
    }

    primask = BMI088_EnterCritical();

    debug->acc_exti_count = bmi088_acc_exti_count;
    debug->gyro_exti_count = bmi088_gyro_exti_count;
    debug->gyro_dma_count = bmi088_gyro_dma_count;
    debug->accel_dma_count = bmi088_accel_dma_count;
    debug->temp_dma_count = bmi088_temp_dma_count;
    debug->dma_error_count = bmi088_dma_error_count;
    debug->last_error = bmi088_last_error;
    debug->spi_busy = bmi088_spi_busy;
    debug->active_transaction = (uint8_t)bmi088_active_transaction;
    debug->gyro_pending = bmi088_gyro_pending;
    debug->accel_pending = bmi088_accel_pending;
    debug->temp_pending = bmi088_temp_pending;
    debug->gyro_timestamp_us = bmi088_real_cache.gyro_timestamp_us;
    debug->accel_timestamp_us = bmi088_real_cache.accel_timestamp_us;
    debug->temp_timestamp_us = bmi088_real_cache.temp_timestamp_us;

    BMI088_ExitCritical(primask);
}

uint8_t BMI088_IsCalibrated(void)
{
    return bmi088_attitude_calibrated;
}

uint8_t BMI088_IsStatic(void)
{
    return bmi088_is_static;
}

void BMI088_ResetReference(void)
{
    uint32_t primask = BMI088_EnterCritical();
    int index = 0;

    for (index = 0; index < 4; index++)
    {
        bmi088_quat_ref[index] = bmi088_quat_now[index];
    }
    BMI088_refresh_attitude_cache(bmi088_attitude_cache.timestamp_us, bmi088_attitude_cache.dt);
    BMI088_ExitCritical(primask);
}

uint8_t BMI088_HasNewGyroSample(void)
{
    return bmi088_new_gyro_sample;
}

void BMI088_ClearNewSampleFlag(void)
{
    uint32_t primask = BMI088_EnterCritical();

    bmi088_new_gyro_sample = 0U;

    BMI088_ExitCritical(primask);
}

void BMI088_DMA_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    bmi088_dma_transaction_t transaction = BMI088_DMA_TRANS_NONE;
    uint32_t timestamp_us = 0U;
    uint32_t primask = 0U;

    if ((hspi == 0) || (hspi->Instance != SPI2))
    {
        return;
    }

    transaction = bmi088_active_transaction;
    timestamp_us = BMI088_get_time_us();

    if (transaction == BMI088_DMA_TRANS_GYRO)
    {
        BMI088_GYRO_NS_H();
        bmi088_gyro_dma_count++;
        BMI088_parse_gyro_frame(timestamp_us);
    }
    else if (transaction == BMI088_DMA_TRANS_ACCEL)
    {
        BMI088_ACCEL_NS_H();
        bmi088_accel_dma_count++;
        BMI088_parse_accel_frame(timestamp_us);
    }
    else if (transaction == BMI088_DMA_TRANS_TEMP)
    {
        BMI088_ACCEL_NS_H();
        bmi088_temp_dma_count++;
        BMI088_parse_temp_frame(timestamp_us);
    }

    primask = BMI088_EnterCritical();
    bmi088_active_transaction = BMI088_DMA_TRANS_NONE;
    bmi088_spi_busy = 0U;
    BMI088_ExitCritical(primask);

    BMI088_try_start_next_transaction();
}

void BMI088_DMA_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    bmi088_dma_transaction_t transaction = BMI088_DMA_TRANS_NONE;
    uint32_t primask = 0U;

    if ((hspi == 0) || (hspi->Instance != SPI2))
    {
        return;
    }

    transaction = bmi088_active_transaction;

    if (transaction == BMI088_DMA_TRANS_GYRO)
    {
        BMI088_GYRO_NS_H();
    }
    else if ((transaction == BMI088_DMA_TRANS_ACCEL) || (transaction == BMI088_DMA_TRANS_TEMP))
    {
        BMI088_ACCEL_NS_H();
    }

    (void)BMI088_spi_abort_dma();

    primask = BMI088_EnterCritical();
    bmi088_dma_error_count++;
    bmi088_last_error = (uint8_t)hspi->ErrorCode;
    bmi088_active_transaction = BMI088_DMA_TRANS_NONE;
    bmi088_spi_busy = 0U;
    BMI088_ExitCritical(primask);

    BMI088_try_start_next_transaction();
}

void BMI088_read(float gyro[3], float accel[3], float *temperate)
{
    BMI088_GetLatestFloat(gyro, accel, temperate, 0);
}

void BMI088_euler_init(void)
{
    BMI088_ResetReference();
}

void BMI088_read_euler(bmi088_euler_data_t *euler, float *temp)
{
    float std_roll = 0.0f;
    float std_pitch = 0.0f;
    float std_yaw = 0.0f;

    BMI088_GetLatestFloat(0, 0, temp, 0);
    BMI088_GetEuler(&std_roll, &std_pitch, &std_yaw, 0, 0);

    if (euler != 0)
    {
        euler->roll = std_roll;
        euler->pitch = std_pitch;
        euler->yaw = std_yaw;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    BMI088_EXTI_Callback(GPIO_Pin);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    BMI088_DMA_TxRxCpltCallback(hspi);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    BMI088_DMA_ErrorCallback(hspi);
}

#if defined(BMI088_USE_SPI)
static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}

static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80U);
    *return_data = BMI088_read_write_byte(BMI088_DUMMY_BYTE);
}
#elif defined(BMI088_USE_IIC)

#endif

static uint32_t BMI088_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void BMI088_ExitCritical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static void BMI088_ResetRuntimeState(void)
{
    int index = 0;

    bmi088_spi_busy = 0U;
    bmi088_gyro_pending = 0U;
    bmi088_accel_pending = 0U;
    bmi088_temp_pending = 0U;
    bmi088_new_gyro_sample = 0U;
    bmi088_last_error = 0U;
    bmi088_accel_sample_count = 0U;
    bmi088_last_gyro_update_us = 0U;
    bmi088_bias_calib_count = 0U;
    bmi088_active_transaction = BMI088_DMA_TRANS_NONE;
    bmi088_acc_exti_count = 0U;
    bmi088_gyro_exti_count = 0U;
    bmi088_gyro_dma_count = 0U;
    bmi088_accel_dma_count = 0U;
    bmi088_temp_dma_count = 0U;
    bmi088_dma_error_count = 0U;
    bmi088_attitude_calibrated = 0U;
    bmi088_is_static = 0U;

    bmi088_raw_cache.status = 0U;
    bmi088_raw_cache.temp = 0;
    bmi088_raw_cache.accel_timestamp_us = 0U;
    bmi088_raw_cache.gyro_timestamp_us = 0U;
    bmi088_raw_cache.temp_timestamp_us = 0U;

    bmi088_real_cache.status = 0U;
    bmi088_real_cache.temp = 0.0f;
    bmi088_real_cache.time = 0.0f;
    bmi088_real_cache.accel_timestamp_us = 0U;
    bmi088_real_cache.gyro_timestamp_us = 0U;
    bmi088_real_cache.temp_timestamp_us = 0U;

    bmi088_attitude_cache.roll = 0.0f;
    bmi088_attitude_cache.pitch = 0.0f;
    bmi088_attitude_cache.yaw = 0.0f;
    bmi088_attitude_cache.quat[0] = 1.0f;
    bmi088_attitude_cache.quat[1] = 0.0f;
    bmi088_attitude_cache.quat[2] = 0.0f;
    bmi088_attitude_cache.quat[3] = 0.0f;
    bmi088_attitude_cache.gyro_bias[0] = 0.0f;
    bmi088_attitude_cache.gyro_bias[1] = 0.0f;
    bmi088_attitude_cache.gyro_bias[2] = 0.0f;
    bmi088_attitude_cache.dt = 0.0f;
    bmi088_attitude_cache.timestamp_us = 0U;
    bmi088_attitude_cache.calibrated = 0U;
    bmi088_attitude_cache.is_static = 0U;

    for (index = 0; index < 3; index++)
    {
        bmi088_raw_cache.accel[index] = 0;
        bmi088_raw_cache.gyro[index] = 0;
        bmi088_real_cache.accel[index] = 0.0f;
        bmi088_real_cache.gyro[index] = 0.0f;
        bmi088_gyro_bias[index] = 0.0f;
        bmi088_gyro_bias_sum[index] = 0.0f;
    }

    BMI088_quat_identity(bmi088_quat_now);
    BMI088_quat_identity(bmi088_quat_ref);

    (void)memset(bmi088_tx_buffer, 0, sizeof(bmi088_tx_buffer));
    (void)memset(bmi088_rx_buffer, 0, sizeof(bmi088_rx_buffer));
}

static void BMI088_try_start_next_transaction(void)
{
    bmi088_dma_transaction_t transaction = BMI088_DMA_TRANS_NONE;
    uint32_t primask = 0U;

    if (bmi088_async_started == 0U)
    {
        return;
    }

    primask = BMI088_EnterCritical();

    if (bmi088_spi_busy != 0U)
    {
        BMI088_ExitCritical(primask);
        return;
    }

    if (bmi088_gyro_pending != 0U)
    {
        bmi088_gyro_pending = 0U;
        transaction = BMI088_DMA_TRANS_GYRO;
    }
    else if (bmi088_accel_pending != 0U)
    {
        bmi088_accel_pending = 0U;
        transaction = BMI088_DMA_TRANS_ACCEL;
    }
    else if (bmi088_temp_pending != 0U)
    {
        bmi088_temp_pending = 0U;
        transaction = BMI088_DMA_TRANS_TEMP;
    }

    if (transaction != BMI088_DMA_TRANS_NONE)
    {
        bmi088_spi_busy = 1U;
        bmi088_active_transaction = transaction;
    }

    BMI088_ExitCritical(primask);

    if (transaction == BMI088_DMA_TRANS_NONE)
    {
        return;
    }

    if (BMI088_start_transaction(transaction) != HAL_OK)
    {
        primask = BMI088_EnterCritical();
        bmi088_last_error = 0xFEU;
        bmi088_spi_busy = 0U;
        bmi088_active_transaction = BMI088_DMA_TRANS_NONE;
        BMI088_restore_pending_for_transaction(transaction);
        BMI088_ExitCritical(primask);
    }
}

static void BMI088_restore_pending_for_transaction(bmi088_dma_transaction_t transaction)
{
    if (transaction == BMI088_DMA_TRANS_GYRO)
    {
        bmi088_gyro_pending = 1U;
    }
    else if (transaction == BMI088_DMA_TRANS_ACCEL)
    {
        bmi088_accel_pending = 1U;
    }
    else if (transaction == BMI088_DMA_TRANS_TEMP)
    {
        bmi088_temp_pending = 1U;
    }
}

static HAL_StatusTypeDef BMI088_start_transaction(bmi088_dma_transaction_t transaction)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    uint16_t transfer_size = 0U;

    (void)memset(bmi088_tx_buffer, BMI088_DUMMY_BYTE, sizeof(bmi088_tx_buffer));
    (void)memset(bmi088_rx_buffer, 0, sizeof(bmi088_rx_buffer));

    if (transaction == BMI088_DMA_TRANS_GYRO)
    {
        BMI088_prepare_gyro_frame(&transfer_size);
        BMI088_GYRO_NS_L();
    }
    else if (transaction == BMI088_DMA_TRANS_ACCEL)
    {
        BMI088_prepare_accel_frame(&transfer_size);
        BMI088_ACCEL_NS_L();
    }
    else if (transaction == BMI088_DMA_TRANS_TEMP)
    {
        BMI088_prepare_temp_frame(&transfer_size);
        BMI088_ACCEL_NS_L();
    }
    else
    {
        return HAL_ERROR;
    }

    status = BMI088_spi_transmit_receive_dma(bmi088_tx_buffer, bmi088_rx_buffer, transfer_size);
    if (status != HAL_OK)
    {
        if (transaction == BMI088_DMA_TRANS_GYRO)
        {
            BMI088_GYRO_NS_H();
        }
        else
        {
            BMI088_ACCEL_NS_H();
        }
    }

    return status;
}

static void BMI088_prepare_gyro_frame(uint16_t *size)
{
    bmi088_tx_buffer[0] = BMI088_GYRO_X_L | 0x80U;

    if (size != 0)
    {
        *size = 7U;
    }
}

static void BMI088_prepare_accel_frame(uint16_t *size)
{
    bmi088_tx_buffer[0] = BMI088_ACCEL_XOUT_L | 0x80U;
    bmi088_tx_buffer[1] = BMI088_DUMMY_BYTE;

    if (size != 0)
    {
        *size = 8U;
    }
}

static void BMI088_prepare_temp_frame(uint16_t *size)
{
    bmi088_tx_buffer[0] = BMI088_TEMP_M | 0x80U;
    bmi088_tx_buffer[1] = BMI088_DUMMY_BYTE;

    if (size != 0)
    {
        *size = 4U;
    }
}

static void BMI088_parse_gyro_frame(uint32_t timestamp_us)
{
    int16_t raw_value = 0;

    raw_value = (int16_t)(((uint16_t)bmi088_rx_buffer[2] << 8) | bmi088_rx_buffer[1]);
    bmi088_raw_cache.gyro[0] = raw_value;
    bmi088_real_cache.gyro[0] = raw_value * BMI088_GYRO_SEN;

    raw_value = (int16_t)(((uint16_t)bmi088_rx_buffer[4] << 8) | bmi088_rx_buffer[3]);
    bmi088_raw_cache.gyro[1] = raw_value;
    bmi088_real_cache.gyro[1] = raw_value * BMI088_GYRO_SEN;

    raw_value = (int16_t)(((uint16_t)bmi088_rx_buffer[6] << 8) | bmi088_rx_buffer[5]);
    bmi088_raw_cache.gyro[2] = raw_value;
    bmi088_real_cache.gyro[2] = raw_value * BMI088_GYRO_SEN;

    bmi088_raw_cache.status |= (1U << BMI088_GYRO_DATA_READY_BIT);
    bmi088_real_cache.status |= (1U << BMI088_GYRO_DATA_READY_BIT);
    bmi088_raw_cache.gyro_timestamp_us = timestamp_us;
    bmi088_real_cache.gyro_timestamp_us = timestamp_us;
    bmi088_real_cache.time = (float)timestamp_us * 1.0e-6f;

    BMI088_update_attitude(timestamp_us);
    bmi088_new_gyro_sample = 1U;
}

static void BMI088_parse_accel_frame(uint32_t timestamp_us)
{
    int16_t raw_value = 0;

    raw_value = (int16_t)(((uint16_t)bmi088_rx_buffer[3] << 8) | bmi088_rx_buffer[2]);
    bmi088_raw_cache.accel[0] = raw_value;
    bmi088_real_cache.accel[0] = raw_value * BMI088_ACCEL_SEN;

    raw_value = (int16_t)(((uint16_t)bmi088_rx_buffer[5] << 8) | bmi088_rx_buffer[4]);
    bmi088_raw_cache.accel[1] = raw_value;
    bmi088_real_cache.accel[1] = raw_value * BMI088_ACCEL_SEN;

    raw_value = (int16_t)(((uint16_t)bmi088_rx_buffer[7] << 8) | bmi088_rx_buffer[6]);
    bmi088_raw_cache.accel[2] = raw_value;
    bmi088_real_cache.accel[2] = raw_value * BMI088_ACCEL_SEN;

    bmi088_raw_cache.status |= (1U << BMI088_ACCEL_DATA_READY_BIT);
    bmi088_real_cache.status |= (1U << BMI088_ACCEL_DATA_READY_BIT);
    bmi088_raw_cache.accel_timestamp_us = timestamp_us;
    bmi088_real_cache.accel_timestamp_us = timestamp_us;

    bmi088_accel_sample_count++;
    if (bmi088_accel_sample_count >= BMI088_TEMP_READ_DIV)
    {
        bmi088_accel_sample_count = 0U;
        bmi088_temp_pending = 1U;
    }
}

static void BMI088_parse_temp_frame(uint32_t timestamp_us)
{
    int16_t raw_temp = 0;

    raw_temp = (int16_t)(((uint16_t)bmi088_rx_buffer[2] << 3) | (bmi088_rx_buffer[3] >> 5));
    if (raw_temp > 1023)
    {
        raw_temp -= 2048;
    }

    bmi088_raw_cache.temp = raw_temp;
    bmi088_real_cache.temp = raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    bmi088_raw_cache.status |= (1U << BMI088_ACCEL_TEMP_DATA_READY_BIT);
    bmi088_real_cache.status |= (1U << BMI088_ACCEL_TEMP_DATA_READY_BIT);
    bmi088_raw_cache.temp_timestamp_us = timestamp_us;
    bmi088_real_cache.temp_timestamp_us = timestamp_us;
}

static void BMI088_update_attitude(uint32_t timestamp_us)
{
    float dt = 0.001f;
    float accel_norm = 0.0f;
    float gyro_meas[3] = {0.0f, 0.0f, 0.0f};
    float gyro_corr[3] = {0.0f, 0.0f, 0.0f};
    float quat_pred[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float quat_acc[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float roll_acc = 0.0f;
    float pitch_acc = 0.0f;
    float yaw_pred = 0.0f;
    int index = 0;

    if (bmi088_last_gyro_update_us != 0U)
    {
        dt = (float)(timestamp_us - bmi088_last_gyro_update_us) * 1.0e-6f;
        if ((dt <= 0.0f) || (dt > 0.02f))
        {
            dt = 0.001f;
        }
    }

    bmi088_last_gyro_update_us = timestamp_us;

    for (index = 0; index < 3; index++)
    {
        gyro_meas[index] = bmi088_real_cache.gyro[index];
    }

    accel_norm = BMI088_vector_norm3((const float *)bmi088_real_cache.accel);
    BMI088_update_static_and_bias(gyro_meas, (const float *)bmi088_real_cache.accel);

    for (index = 0; index < 3; index++)
    {
        gyro_corr[index] = gyro_meas[index] - bmi088_gyro_bias[index];
    }

    if (bmi088_attitude_calibrated == 0U)
    {
        if (bmi088_is_static != 0U)
        {
            for (index = 0; index < 3; index++)
            {
                bmi088_gyro_bias_sum[index] += gyro_meas[index];
            }

            bmi088_bias_calib_count++;
            if (bmi088_bias_calib_count >= BMI088_BIAS_CALIB_SAMPLES)
            {
                for (index = 0; index < 3; index++)
                {
                    bmi088_gyro_bias[index] = bmi088_gyro_bias_sum[index] / (float)bmi088_bias_calib_count;
                    gyro_corr[index] = gyro_meas[index] - bmi088_gyro_bias[index];
                }

                if (((bmi088_real_cache.status & (1U << BMI088_ACCEL_DATA_READY_BIT)) != 0U) &&
                    (accel_norm > 1.0e-6f))
                {
                    BMI088_quat_from_accel(quat_acc, (const float *)bmi088_real_cache.accel);
                    for (index = 0; index < 4; index++)
                    {
                        bmi088_quat_now[index] = quat_acc[index];
                        bmi088_quat_ref[index] = quat_acc[index];
                    }
                }
                else
                {
                    BMI088_quat_identity(bmi088_quat_now);
                    BMI088_quat_identity(bmi088_quat_ref);
                }

                bmi088_attitude_calibrated = 1U;
            }
        }
        else
        {
            bmi088_bias_calib_count = 0U;
            for (index = 0; index < 3; index++)
            {
                bmi088_gyro_bias_sum[index] = 0.0f;
            }
        }

        BMI088_refresh_attitude_cache(timestamp_us, dt);
        return;
    }

    for (index = 0; index < 4; index++)
    {
        quat_pred[index] = bmi088_quat_now[index];
    }

    BMI088_quat_integrate(quat_pred, gyro_corr, dt);

    if (((bmi088_real_cache.status & (1U << BMI088_ACCEL_DATA_READY_BIT)) != 0U) &&
        (accel_norm > 1.0e-6f))
    {
        BMI088_roll_pitch_from_accel((const float *)bmi088_real_cache.accel, &roll_acc, &pitch_acc);
        BMI088_euler_from_quat(quat_pred, 0, 0, &yaw_pred);
        BMI088_quat_from_euler(quat_acc, roll_acc, pitch_acc, yaw_pred);
        BMI088_quat_slerp(quat_pred, quat_pred, quat_acc, BMI088_AHRS_CORRECTION_GAIN * dt);
    }

    for (index = 0; index < 4; index++)
    {
        bmi088_quat_now[index] = quat_pred[index];
    }

    BMI088_quat_normalize((float *)bmi088_quat_now);
    BMI088_refresh_attitude_cache(timestamp_us, dt);
}

static void BMI088_quat_identity(volatile float quat[4])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;
}

static void BMI088_quat_copy(float dst[4], const volatile float src[4])
{
    int index = 0;

    for (index = 0; index < 4; index++)
    {
        dst[index] = src[index];
    }
}

static void BMI088_quat_multiply(float out[4], const float a[4], const float b[4])
{
    float q0 = (a[0] * b[0]) - (a[1] * b[1]) - (a[2] * b[2]) - (a[3] * b[3]);
    float q1 = (a[0] * b[1]) + (a[1] * b[0]) + (a[2] * b[3]) - (a[3] * b[2]);
    float q2 = (a[0] * b[2]) - (a[1] * b[3]) + (a[2] * b[0]) + (a[3] * b[1]);
    float q3 = (a[0] * b[3]) + (a[1] * b[2]) - (a[2] * b[1]) + (a[3] * b[0]);

    out[0] = q0;
    out[1] = q1;
    out[2] = q2;
    out[3] = q3;
}

static void BMI088_quat_conjugate(float out[4], const float quat[4])
{
    out[0] = quat[0];
    out[1] = -quat[1];
    out[2] = -quat[2];
    out[3] = -quat[3];
}

static void BMI088_quat_normalize(float quat[4])
{
    float norm = sqrtf((quat[0] * quat[0]) + (quat[1] * quat[1]) + (quat[2] * quat[2]) + (quat[3] * quat[3]));

    if (norm <= 1.0e-6f)
    {
        quat[0] = 1.0f;
        quat[1] = 0.0f;
        quat[2] = 0.0f;
        quat[3] = 0.0f;
        return;
    }

    quat[0] /= norm;
    quat[1] /= norm;
    quat[2] /= norm;
    quat[3] /= norm;
}

static void BMI088_quat_integrate(float quat[4], const float gyro[3], float dt)
{
    float omega_norm = BMI088_vector_norm3(gyro);
    float angle = omega_norm * dt;
    float dq[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float quat_out[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    if (angle > 1.0e-9f)
    {
        float half_angle = 0.5f * angle;
        float sin_half = sinf(half_angle);
        float scale = sin_half / omega_norm;

        dq[0] = cosf(half_angle);
        dq[1] = gyro[0] * scale;
        dq[2] = gyro[1] * scale;
        dq[3] = gyro[2] * scale;
    }

    BMI088_quat_multiply(quat_out, quat, dq);
    quat[0] = quat_out[0];
    quat[1] = quat_out[1];
    quat[2] = quat_out[2];
    quat[3] = quat_out[3];
    BMI088_quat_normalize(quat);
}

static void BMI088_quat_from_accel(float quat[4], const float accel[3])
{
    float roll = 0.0f;
    float pitch = 0.0f;

    BMI088_roll_pitch_from_accel(accel, &roll, &pitch);
    BMI088_quat_from_euler(quat, roll, pitch, 0.0f);
}

static void BMI088_roll_pitch_from_accel(const float accel[3], float *roll, float *pitch)
{
    float ax = accel[0];
    float ay = accel[1];
    float az = accel[2];
    float accel_norm = sqrtf((ax * ax) + (ay * ay) + (az * az));

    if (accel_norm <= 1.0e-6f)
    {
        if (roll != 0)
        {
            *roll = 0.0f;
        }

        if (pitch != 0)
        {
            *pitch = 0.0f;
        }
        return;
    }

    ax /= accel_norm;
    ay /= accel_norm;
    az /= accel_norm;

    if (roll != 0)
    {
        *roll = atan2f(ay, az);
    }

    if (pitch != 0)
    {
        *pitch = atan2f(-ax, sqrtf((ay * ay) + (az * az)));
    }
}

static void BMI088_quat_from_euler(float quat[4], float roll, float pitch, float yaw)
{
    float cr = cosf(0.5f * roll);
    float sr = sinf(0.5f * roll);
    float cp = cosf(0.5f * pitch);
    float sp = sinf(0.5f * pitch);
    float cy = cosf(0.5f * yaw);
    float sy = sinf(0.5f * yaw);

    quat[0] = (cr * cp * cy) + (sr * sp * sy);
    quat[1] = (sr * cp * cy) - (cr * sp * sy);
    quat[2] = (cr * sp * cy) + (sr * cp * sy);
    quat[3] = (cr * cp * sy) - (sr * sp * cy);
    BMI088_quat_normalize(quat);
}

static void BMI088_quat_slerp(float out[4], const float from[4], const float to[4], float t)
{
    float q_to[4] = {to[0], to[1], to[2], to[3]};
    float dot = (from[0] * q_to[0]) + (from[1] * q_to[1]) + (from[2] * q_to[2]) + (from[3] * q_to[3]);
    float scale_from = 0.0f;
    float scale_to = 0.0f;
    float theta = 0.0f;
    float sin_theta = 0.0f;
    int index = 0;

    if (t <= 0.0f)
    {
        for (index = 0; index < 4; index++)
        {
            out[index] = from[index];
        }
        return;
    }
    if (t >= 1.0f)
    {
        for (index = 0; index < 4; index++)
        {
            out[index] = q_to[index];
        }
        return;
    }

    if (dot < 0.0f)
    {
        dot = -dot;
        q_to[0] = -q_to[0];
        q_to[1] = -q_to[1];
        q_to[2] = -q_to[2];
        q_to[3] = -q_to[3];
    }

    if (dot > 0.9995f)
    {
        for (index = 0; index < 4; index++)
        {
            out[index] = from[index] + t * (q_to[index] - from[index]);
        }
        BMI088_quat_normalize(out);
        return;
    }

    theta = acosf(dot);
    sin_theta = sinf(theta);
    scale_from = sinf((1.0f - t) * theta) / sin_theta;
    scale_to = sinf(t * theta) / sin_theta;

    for (index = 0; index < 4; index++)
    {
        out[index] = (scale_from * from[index]) + (scale_to * q_to[index]);
    }
    BMI088_quat_normalize(out);
}

static void BMI088_relative_quat_from_state(float quat_rel[4])
{
    float quat_ref[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float quat_ref_inv[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float quat_now[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    BMI088_quat_copy(quat_ref, bmi088_quat_ref);
    BMI088_quat_copy(quat_now, bmi088_quat_now);
    BMI088_quat_conjugate(quat_ref_inv, quat_ref);
    BMI088_quat_multiply(quat_rel, quat_ref_inv, quat_now);
    BMI088_quat_normalize(quat_rel);
}

static void BMI088_euler_from_quat(const float quat[4], float *roll, float *pitch, float *yaw)
{
    float sinr_cosp = 2.0f * ((quat[0] * quat[1]) + (quat[2] * quat[3]));
    float cosr_cosp = 1.0f - 2.0f * ((quat[1] * quat[1]) + (quat[2] * quat[2]));
    float sinp = 2.0f * ((quat[0] * quat[2]) - (quat[3] * quat[1]));
    float siny_cosp = 2.0f * ((quat[0] * quat[3]) + (quat[1] * quat[2]));
    float cosy_cosp = 1.0f - 2.0f * ((quat[2] * quat[2]) + (quat[3] * quat[3]));

    if (roll != 0)
    {
        *roll = atan2f(sinr_cosp, cosr_cosp);
    }

    if (pitch != 0)
    {
        if (sinp >= 1.0f)
        {
            *pitch = 1.57079632679f;
        }
        else if (sinp <= -1.0f)
        {
            *pitch = -1.57079632679f;
        }
        else
        {
            *pitch = asinf(sinp);
        }
    }

    if (yaw != 0)
    {
        *yaw = atan2f(siny_cosp, cosy_cosp);
    }
}

static float BMI088_vector_norm3(const float vec[3])
{
    return sqrtf((vec[0] * vec[0]) + (vec[1] * vec[1]) + (vec[2] * vec[2]));
}

static void BMI088_update_static_and_bias(const float gyro_meas[3], const float accel[3])
{
    float gyro_norm = BMI088_vector_norm3(gyro_meas);
    float accel_norm = BMI088_vector_norm3(accel);
    int index = 0;

    if ((gyro_norm < BMI088_STATIC_GYRO_NORM_THRESHOLD) &&
        (fabsf(accel_norm - BMI088_GRAVITY_NORM) < BMI088_STATIC_ACCEL_NORM_THRESHOLD))
    {
        bmi088_is_static = 1U;
    }
    else
    {
        bmi088_is_static = 0U;
    }

    if ((bmi088_attitude_calibrated != 0U) && (bmi088_is_static != 0U))
    {
        for (index = 0; index < 3; index++)
        {
            bmi088_gyro_bias[index] += BMI088_BIAS_TRACK_ALPHA * (gyro_meas[index] - bmi088_gyro_bias[index]);
        }
    }
}

static void BMI088_refresh_attitude_cache(uint32_t timestamp_us, float dt)
{
    float quat_rel[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    int index = 0;

    BMI088_relative_quat_from_state(quat_rel);
    BMI088_euler_from_quat(quat_rel, &roll, &pitch, &yaw);

    bmi088_attitude_cache.roll = roll;
    bmi088_attitude_cache.pitch = pitch;
    bmi088_attitude_cache.yaw = yaw;

    for (index = 0; index < 4; index++)
    {
        bmi088_attitude_cache.quat[index] = quat_rel[index];
    }
    for (index = 0; index < 3; index++)
    {
        bmi088_attitude_cache.gyro_bias[index] = bmi088_gyro_bias[index];
    }

    bmi088_attitude_cache.dt = dt;
    bmi088_attitude_cache.timestamp_us = timestamp_us;
    bmi088_attitude_cache.calibrated = bmi088_attitude_calibrated;
    bmi088_attitude_cache.is_static = bmi088_is_static;
}


