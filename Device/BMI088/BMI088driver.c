#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"
#include <math.h>




static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

// 欧拉角计算相关全局变量
static bmi088_euler_data_t euler_angle = {0, 0, 0};
static float last_time = 0;
static float gyro_offset[3] = {0, 0, 0};  // 陀螺仪零漂补偿
static const float COMPLEMENTARY_FILTER_ALPHA = 0.95f;  // 降低alpha值，减少延迟（原来0.98）
static const float PI = 3.14159265358979f;
static const uint16_t GYRO_CALIBRATION_SAMPLES = 100;  // 零点校准采样次数
static uint8_t is_calibrated = 0;  // 校准标志

// Kalman filter state for angle estimation (angle + bias)
typedef struct {
    float angle;   // estimated angle
    float bias;    // estimated gyro bias
    float P[2][2]; // error covariance
    float Q_angle; // process noise variance for the angle
    float Q_bias;  // process noise variance for the gyro bias
    float R_measure; // measurement noise variance
} Kalman_t;

static Kalman_t kalman_pitch;

// 默认噪声参数，可根据实际调整
static const float KALMAN_Q_ANGLE = 0.001f;
static const float KALMAN_Q_BIAS = 0.003f;
static const float KALMAN_R_MEASURE = 0.03f;

static void BMI088_kalman_init(Kalman_t *k)
{
    k->angle = 0.0f;
    k->bias = 0.0f;
    k->P[0][0] = 0.0f; k->P[0][1] = 0.0f;
    k->P[1][0] = 0.0f; k->P[1][1] = 0.0f;
    k->Q_angle = KALMAN_Q_ANGLE;
    k->Q_bias = KALMAN_Q_BIAS;
    k->R_measure = KALMAN_R_MEASURE;
}

// 卡尔曼滤波器更新：以陀螺仪角速度（deg/s）和加速度角度测量（deg）更新
static float BMI088_kalman_update(Kalman_t *k, float newRate, float newAngle, float dt)
{
    // Predict
    // State transition: angle += (newRate - bias) * dt
    float rate = newRate - k->bias;
    k->angle += dt * rate;

    // Update error covariance: P = A*P*A' + Q
    k->P[0][0] += dt * (dt*k->P[1][1] - k->P[0][1] - k->P[1][0] + k->Q_angle);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += k->Q_bias * dt;

    // Compute Kalman gain
    float S = k->P[0][0] + k->R_measure; // Estimate error
    float K0 = k->P[0][0] / S;
    float K1 = k->P[1][0] / S;

    // Update estimate with measurement
    float y = newAngle - k->angle;
    k->angle += K0 * y;
    k->bias += K1 * y;

    // Update error covariance: P = (I - K*H) * P
    float P00 = k->P[0][0];
    float P01 = k->P[0][1];
    float P10 = k->P[1][0];
    float P11 = k->P[1][1];

    k->P[0][0] = P00 - K0 * P00;
    k->P[0][1] = P01 - K0 * P01;
    k->P[1][0] = P10 - K1 * P00;
    k->P[1][1] = P11 - K1 * P01;

    return k->angle;
}
/**
 * @brief 角度归一化到 -180° 到 +180° 范围
 * @param angle 输入角度
 * @return 归一化后的角度
 */
static float BMI088_normalize_angle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief 陀螺仪零点校准（需要在静止状态下调用）
 * @retval void
 */
static void BMI088_gyro_calibration(void)
{
    float gyro_sum[3] = {0, 0, 0};
    uint8_t buf[8];
    int16_t bmi088_raw_temp;
    
    // 采样多次求平均
    for(uint16_t i = 0; i < GYRO_CALIBRATION_SAMPLES; i++)
    {
        BMI088_GYRO_NS_L();
        BMI088_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
        BMI088_GYRO_NS_H();
        if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
        {
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            gyro_sum[0] += bmi088_raw_temp * BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            gyro_sum[1] += bmi088_raw_temp * BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
            gyro_sum[2] += bmi088_raw_temp * BMI088_GYRO_SEN;
        }
        BMI088_delay_ms(10);  // 采样间隔10ms
    }
    
    // 计算平均值作为零点偏移
    gyro_offset[0] = gyro_sum[0] / GYRO_CALIBRATION_SAMPLES;
    gyro_offset[1] = gyro_sum[1] / GYRO_CALIBRATION_SAMPLES;
    gyro_offset[2] = gyro_sum[2] / GYRO_CALIBRATION_SAMPLES;
    
    is_calibrated = 1;
}

/**
 * @brief 初始化欧拉角计算模块
 * @retval void
 */
void BMI088_euler_init(void)
{
    euler_angle.pitch = 0;
    euler_angle.roll = 0;
    euler_angle.yaw = 0;
    last_time = 0;
    is_calibrated = 0;
    
    // 进行陀螺仪零点校准（确保设备静止）
    BMI088_gyro_calibration();
    // 初始化卡尔曼滤波器
    BMI088_kalman_init(&kalman_pitch);
}

/**
 * @brief 从加速度计数据计算roll和pitch（初始姿态）
 * @param accel 加速度计数据
 * @param pitch 俯仰角指针
 * @param roll 翻滚角指针
 */
static void BMI088_accel_to_angle(float accel[3], float *pitch, float *roll)
{
    // pitch: 绕Y轴旋转，由加速度X和Z分量确定
    *pitch = atan2f(accel[0], sqrtf(accel[1] * accel[1] + accel[2] * accel[2])) * 180.0f / PI;
    
    // roll: 绕X轴旋转，由加速度Y和Z分量确定  
    *roll = atan2f(accel[1], accel[2]) * 180.0f / PI;
}

/**
 * @brief 欧拉角更新（基于陀螺仪积分和互补滤波）
 * @param gyro 陀螺仪数据 (deg/s)
 * @param accel 加速度计数据 (m/s^2)
 * @param dt 时间间隔 (s)
 */
static void BMI088_update_euler(float gyro[3], float accel[3], float dt)
{
    float head_accel[3], head_gyro[3];
    float accel_pitch;

    /* axis remap: sensor -> head frame
       Board mounted: USB up, BMI088 X-up(along board), Y-left
       head_X(forward) = -sensor_Z, head_Y(left) = sensor_Y, head_Z(up) = sensor_X
       gyro: same rotation applies */
    head_accel[0] = -accel[2];   /* forward */
    head_accel[1] =  accel[1];   /* left */
    head_accel[2] =  accel[0];   /* up */

    head_gyro[0] = -gyro[2];     /* around forward */
    head_gyro[1] =  gyro[1];     /* around left (nod axis) */
    head_gyro[2] =  gyro[0];     /* around vertical (turn axis) */

    if(is_calibrated)
    {
        /* Pitch (nod): Kalman filter - accel provides absolute ref, gyro provides rate */
        accel_pitch = atan2f(head_accel[0],
                             sqrtf(head_accel[1] * head_accel[1] +
                                   head_accel[2] * head_accel[2]))
                      * 180.0f / PI;
        euler_angle.pitch = BMI088_kalman_update(&kalman_pitch,
                                                  head_gyro[1] - gyro_offset[1],
                                                  accel_pitch, dt);

        /* Roll (head turn): pure gyro integration around vertical axis.
           No absolute reference from accelerometer for vertical rotation.
           Reset to zero during calibration, drift acceptable in short sessions. */
        euler_angle.roll += (head_gyro[2] - gyro_offset[0]) * dt;
    }
    else
    {
        euler_angle.pitch += head_gyro[1] * dt;
        euler_angle.roll  += head_gyro[2] * dt;
    }

    euler_angle.roll = BMI088_normalize_angle(euler_angle.roll);
    euler_angle.yaw  = atan2f(head_accel[1], fabsf(head_accel[2])) * 180.0f / PI;  /* lateral tilt (ear-to-shoulder), 0 at vertical regardless of USB up/down */
}

#if defined(BMI088_USE_SPI)
/**
************************************************************************
* @brief:      	BMI088_accel_write_single_reg(reg, data)
* @param:       reg - 寄存器地址
*               data - 写入的数据
* @retval:     	void
* @details:    	通过BMI088加速度计的SPI总线写入单个寄存器的宏定义
************************************************************************
**/
#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }
/**
************************************************************************
* @brief:      	BMI088_accel_read_single_reg(reg, data)
* @param:       reg - 寄存器地址
*               data - 读取的寄存器数据
* @retval:     	void
* @details:    	通过BMI088加速度计的SPI总线读取单个寄存器的宏定义
************************************************************************
**/
#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_read_write_byte((reg) | 0x80);   \
        BMI088_read_write_byte(0x55);           \
        (data) = BMI088_read_write_byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }
/**
************************************************************************
* @brief:      	BMI088_accel_read_muli_reg(reg, data, len)
* @param:       reg - 起始寄存器地址
*               data - 存储读取数据的缓冲区
*               len - 要读取的字节数
* @retval:     	void
* @details:    	通过BMI088加速度计的SPI总线连续读取多个寄存器的宏定义
************************************************************************
**/
#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_read_write_byte((reg) | 0x80);      \
        BMI088_read_muli_reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                       \
    }
/**
************************************************************************
* @brief:      	BMI088_gyro_write_single_reg(reg, data)
* @param:       reg - 寄存器地址
*               data - 写入的数据
* @retval:     	void
* @details:    	通过BMI088陀螺仪的SPI总线写入单个寄存器的宏定义
************************************************************************
**/
#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }
/**
************************************************************************
* @brief:      	BMI088_gyro_read_single_reg(reg, data)
* @param:       reg - 寄存器地址
*               data - 读取的寄存器数据
* @retval:     	void
* @details:    	通过BMI088陀螺仪的SPI总线读取单个寄存器的宏定义
************************************************************************
**/
#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }
/**
************************************************************************
* @brief:      	BMI088_gyro_read_muli_reg(reg, data, len)
* @param:       reg - 起始寄存器地址
*               data - 存储读取数据的缓冲区
*               len - 要读取的字节数
* @retval:     	void
* @details:    	通过BMI088陀螺仪的SPI总线连续读取多个寄存器的宏定义
************************************************************************
**/
#define BMI088_gyro_read_muli_reg(reg, data, len)   \
    {                                               \
        BMI088_GYRO_NS_L();                         \
        BMI088_read_muli_reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                         \
    }

static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
//static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len );
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

uint8_t bmi088_accel_init(void);
uint8_t bmi088_gyro_init(void);

#elif defined(BMI088_USE_IIC)


#endif
/**
************************************************************************
* @brief:      	write_BMI088_accel_reg_data_error_init(void)
* @param:       void
* @retval:     	void
* @details:    	BMI088加速度传感器寄存器数据写入错误处理初始化
************************************************************************
**/
static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};
/**
************************************************************************
* @brief:      	write_BMI088_gyro_reg_data_error_init(void)
* @param:       void
* @retval:     	void
* @details:    	BMI088陀螺仪传感器寄存器数据写入错误处理初始化
************************************************************************
**/
static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};
/**
************************************************************************
* @brief:      	BMI088_init(void)
* @param:       void
* @retval:     	uint8_t - 错误代码
* @details:    	BMI088传感器初始化函数，包括GPIO和SPI初始化，以及加速度和陀螺仪的初始化
************************************************************************
**/
uint8_t BMI088_init(void)
{
    uint8_t error = BMI088_NO_ERROR;
    // GPIO and SPI  Init .
    BMI088_GPIO_init();
    BMI088_com_init();

    error |= bmi088_accel_init();
    error |= bmi088_gyro_init();

    return error;
}
/**
************************************************************************
* @brief:      	bmi088_accel_init(void)
* @param:       void
* @retval:     	uint8_t - 错误代码
* @details:    	BMI088加速度传感器初始化函数，包括通信检查、软件复位、配置寄存器写入及检查
************************************************************************
**/
uint8_t bmi088_accel_init(void)
{
    uint8_t res = 0;
    uint8_t write_reg_num = 0;

    //check commiunication
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //accel software reset
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal after reset
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
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
/**
************************************************************************
* @brief:      	bmi088_gyro_init(void)
* @param:       void
* @retval:     	uint8_t - 错误代码
* @details:    	BMI088陀螺仪传感器初始化函数，包括通信检查、软件复位、配置寄存器写入及检查
************************************************************************
**/
uint8_t bmi088_gyro_init(void)
{
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    //check commiunication
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
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
/**
************************************************************************
* @brief:      	BMI088_read(float gyro[3], float accel[3], float *temperate)
* @param:       gyro - 陀螺仪数据数组 (x, y, z)
* @param:       accel - 加速度计数据数组 (x, y, z)
* @param:       temperate - 温度数据指针
* @retval:     	void
* @details:    	读取BMI088传感器数据，包括加速度、陀螺仪和温度
************************************************************************
**/
void BMI088_read(float gyro[3], float accel[3], float *temperate)
{
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;

    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;

    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
        gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
    }
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }

    *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

/**
 * @brief 读取欧拉角（pitch、roll、yaw）和温度
 * @param euler 欧拉角结构体指针
 * @param temp 温度指针
 * @retval void
 */
void BMI088_read_euler(bmi088_euler_data_t *euler, float *temp)
{
    uint8_t buf[8] = {0};
    int16_t bmi088_raw_temp;
    float gyro[3], accel[3];
    float current_time = 0;
    static uint32_t sample_count = 0;
    
    // 读取加速度计数据
    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    
    // 读取陀螺仪数据
    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
        gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
    }
    
    // 读取温度
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }
    *temp = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    
    // 计算时间间隔（假设固定采样率10ms）
    sample_count++;
    float dt = 0.01f;  // 10ms采样周期，可根据实际调整
    
    // 更新欧拉角
    BMI088_update_euler(gyro, accel, dt);
    
    // 返回计算得到的欧拉角
    euler->pitch = euler_angle.pitch;
    euler->roll = euler_angle.roll;
    euler->yaw = euler_angle.yaw;
}

#if defined(BMI088_USE_SPI)
/**
************************************************************************
* @brief:      	BMI088_write_single_reg(uint8_t reg, uint8_t data)
* @param:       reg - 寄存器地址
* @param:       data - 写入的数据
* @retval:     	void
* @details:    	向BMI088传感器写入单个寄存器的数据
************************************************************************
**/
static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}
/**
************************************************************************
* @brief:      	BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
* @param:       reg - 寄存器地址
* @param:       return_data - 读取的寄存器数据
* @retval:     	void
* @details:    	从BMI088传感器读取单个寄存器的数据
************************************************************************
**/
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);
    *return_data = BMI088_read_write_byte(0x55);
}

//static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len )
//{
//    BMI088_read_write_byte( reg );
//    while( len != 0 )
//    {

//        BMI088_read_write_byte( *buf );
//        buf ++;
//        len --;
//    }

//}
/**
************************************************************************
* @brief:      	BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
* @param:       reg - 起始寄存器地址
*               buf - 存储读取数据的缓冲区
*               len - 要读取的字节数
* @retval:     	void
* @details:    	从BMI088传感器连续读取多个寄存器的数据
************************************************************************
**/
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(reg | 0x80);

    while (len != 0)
    {

        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
}
#elif defined(BMI088_USE_IIC)

#endif
