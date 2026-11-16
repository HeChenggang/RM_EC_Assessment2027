/* BMI088 six-axis IMU on SPI1 of the RoboMaster C board.
 *
 * The chip contains two dies with separate chip selects and, annoyingly, two
 * different SPI frame layouts: the accelerometer inserts one dummy byte between
 * the address byte and the data, the gyroscope does not. Both are handled below
 * through BMI088_Target.dummy_bytes.
 *
 * Reading is interrupt driven. The data-ready pins only set a flag; the SPI
 * transfer runs in BMI088_Service from the main loop, so the EXTI handlers stay
 * short enough for the 1 kHz control tick that shares the bus.
 */
#include "bmi088.h"
#include "main.h"

#define BMI088_ACCEL_CHIP_ID 0x1EU
#define BMI088_GYRO_CHIP_ID 0x0FU

#define BMI088_SPI_TIMEOUT_MS 5U
#define BMI088_ACCEL_RESET_DELAY_MS 2U /* Datasheet: wait >= 1 ms after reset. */
#define BMI088_GYRO_RESET_DELAY_MS 30U /* Datasheet: wait 30 ms after reset. */
#define BMI088_MAX_READ 6U             /* Largest burst: three axes of int16. */

/* Accelerometer register map. */
#define BMI088_ACC_CHIP_ID_REG 0x00U
#define BMI088_ACC_DATA_REG 0x12U /* X_LSB..Z_MSB, auto-incrementing burst. */
#define BMI088_ACC_TEMP_MSB_REG 0x22U
#define BMI088_ACC_CONF_REG 0x40U
#define BMI088_ACC_RANGE_REG 0x41U
#define BMI088_ACC_INT1_IO_CTRL_REG 0x53U
#define BMI088_ACC_INT2_IO_CTRL_REG 0x54U
#define BMI088_ACC_INT_MAP_DATA_REG 0x58U
#define BMI088_ACC_PWR_CONF_REG 0x7CU
#define BMI088_ACC_PWR_CTRL_REG 0x7DU
#define BMI088_ACC_SOFTRESET_REG 0x7EU
#define BMI088_SOFTRESET_CMD 0xB6U
/* Bit 7 of ACC_CONF is marked "must be set" in the datasheet; 0x0A is the
 * normal-bandwidth code and 0x8 selects 800 Hz. */
#define BMI088_ACC_CONF_800HZ 0xA8U
#define BMI088_ACC_PWR_CTRL_ON 0x04U
#define BMI088_ACC_PWR_CONF_ACTIVE 0x00U
#define BMI088_ACC_RANGE_6G_CODE 0x01U
/* Output enable (0x08) + active-high level (0x02), push-pull. */
#define BMI088_ACC_INT_PUSH_PULL_ACTIVE_HIGH 0x0AU
#define BMI088_ACC_INT1_DRDY 0x04U

/* Gyroscope register map. */
#define BMI088_GYRO_CHIP_ID_REG 0x00U
#define BMI088_GYRO_RATE_REG 0x02U /* X_LSB..Z_MSB, auto-incrementing burst. */
#define BMI088_GYRO_RANGE_REG 0x0FU
#define BMI088_GYRO_BANDWIDTH_REG 0x10U
#define BMI088_GYRO_LPM1_REG 0x11U
#define BMI088_GYRO_SOFTRESET_REG 0x14U
#define BMI088_GYRO_INT_CTRL_REG 0x15U
#define BMI088_GYRO_INT3_INT4_IO_CONF_REG 0x16U
#define BMI088_GYRO_INT3_INT4_IO_MAP_REG 0x18U
#define BMI088_GYRO_RANGE_2000_CODE 0x00U
#define BMI088_GYRO_BANDWIDTH_1000HZ_230HZ 0x01U
#define BMI088_GYRO_NORMAL_MODE 0x00U
#define BMI088_GYRO_DRDY_ON 0x80U
#define BMI088_GYRO_INT3_PUSH_PULL_ACTIVE_HIGH 0x01U
#define BMI088_GYRO_DRDY_TO_INT3 0x01U

#define BMI088_FLAG_ACCEL_READY 0x01U
#define BMI088_FLAG_GYRO_READY 0x02U

#define BMI088_ACCEL_TO_MPS2 (BMI088_ACCEL_RANGE_G * 9.80665f / 32768.0f)
#define BMI088_GYRO_TO_DPS (BMI088_GYRO_RANGE_DPS / 32768.0f)

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t dummy_bytes;
} BMI088_Target;

static const BMI088_Target accel_target = {
    IMU_ACCEL_CS_GPIO_Port, IMU_ACCEL_CS_Pin, 1U};
static const BMI088_Target gyro_target = {
    IMU_GYRO_CS_GPIO_Port, IMU_GYRO_CS_Pin, 0U};

static SPI_HandleTypeDef *imu_spi;
static bool imu_ready;
static volatile uint8_t pending_flags;
static BMI088_Sample sample;
static BMI088_GyroCalibration calibration;
static float calibration_sum[3];
static float calibration_min[3];
static float calibration_max[3];

static HAL_StatusTypeDef read_regs(const BMI088_Target *target, uint8_t reg,
                                   uint8_t *out, uint16_t length)
{
    uint8_t tx[2U + BMI088_MAX_READ] = {0};
    uint8_t rx[2U + BMI088_MAX_READ] = {0};
    if (out == NULL || length == 0U || length > BMI088_MAX_READ) {
        return HAL_ERROR;
    }
    const uint16_t size = (uint16_t)(1U + target->dummy_bytes + length);
    tx[0] = (uint8_t)(reg | 0x80U);

    HAL_GPIO_WritePin(target->port, target->pin, GPIO_PIN_RESET);
    const HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive(imu_spi, tx, rx, size, BMI088_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(target->port, target->pin, GPIO_PIN_SET);
    if (status != HAL_OK) {
        return status;
    }
    for (uint16_t i = 0; i < length; ++i) {
        out[i] = rx[1U + target->dummy_bytes + i];
    }
    return HAL_OK;
}

static HAL_StatusTypeDef write_reg(const BMI088_Target *target, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {(uint8_t)(reg & 0x7FU), value};
    uint8_t rx[2] = {0};

    HAL_GPIO_WritePin(target->port, target->pin, GPIO_PIN_RESET);
    const HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive(imu_spi, tx, rx, 2U, BMI088_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(target->port, target->pin, GPIO_PIN_SET);
    return status;
}

static int16_t combine(const uint8_t *raw, unsigned axis)
{
    const uint16_t low = raw[2U * axis];
    const uint16_t high = (uint16_t)((uint16_t)raw[2U * axis + 1U] << 8);
    return (int16_t)(low | high);
}

static void read_accelerometer(void)
{
    uint8_t raw[BMI088_MAX_READ];
    if (read_regs(&accel_target, BMI088_ACC_DATA_REG, raw, BMI088_MAX_READ) != HAL_OK) {
        ++sample.spi_errors;
        return;
    }
    for (unsigned axis = 0; axis < 3U; ++axis) {
        sample.accel_mps2[axis] = (float)combine(raw, axis) * BMI088_ACCEL_TO_MPS2;
    }
    uint8_t temperature[2];
    if (read_regs(&accel_target, BMI088_ACC_TEMP_MSB_REG, temperature, 2U) == HAL_OK &&
        temperature[0] != 0x80U) {
        /* 11-bit two's complement, 0.125 degree steps, 23 degC offset. */
        int16_t value = (int16_t)(((uint16_t)temperature[0] << 3) | (temperature[1] >> 5));
        if (value > 1023) {
            value = (int16_t)(value - 2048);
        }
        sample.temperature_c = (float)value * 0.125f + 23.0f;
    } else {
        ++sample.spi_errors;
    }
    ++sample.accel_updates;
    sample.last_accel_ms = HAL_GetTick();
}

static void finish_calibration(void)
{
    float bias[3];
    float spread = 0.0f;

    for (unsigned axis = 0; axis < 3U; ++axis) {
        bias[axis] = calibration_sum[axis] / (float)calibration.collected;
        const float axis_spread = calibration_max[axis] - calibration_min[axis];
        if (axis_spread > spread) {
            spread = axis_spread;
        }
    }
    calibration.running = false;
    calibration.spread_dps = spread;
    if (spread > BMI088_CALIBRATION_MAX_SPREAD_DPS) {
        /* Motion inside the window: keep whatever bias was valid before, report
         * the failure and let the caller decide whether to retry. */
        calibration.failed = true;
        return;
    }
    for (unsigned axis = 0; axis < 3U; ++axis) {
        calibration.bias_dps[axis] = bias[axis];
    }
    calibration.calibrated = true;
    calibration.failed = false;
}

static void accumulate_calibration(const float dps[3])
{
    if (!calibration.running) {
        return;
    }
    for (unsigned axis = 0; axis < 3U; ++axis) {
        if (calibration.collected == 0U) {
            calibration_min[axis] = dps[axis];
            calibration_max[axis] = dps[axis];
        } else {
            if (dps[axis] < calibration_min[axis]) {
                calibration_min[axis] = dps[axis];
            }
            if (dps[axis] > calibration_max[axis]) {
                calibration_max[axis] = dps[axis];
            }
        }
        calibration_sum[axis] += dps[axis];
    }
    ++calibration.collected;
    if (calibration.collected >= calibration.target) {
        finish_calibration();
    }
}

static bool read_gyroscope(void)
{
    uint8_t raw[BMI088_MAX_READ];
    if (read_regs(&gyro_target, BMI088_GYRO_RATE_REG, raw, BMI088_MAX_READ) != HAL_OK) {
        ++sample.spi_errors;
        return false;
    }
    float dps[3];
    for (unsigned axis = 0; axis < 3U; ++axis) {
        dps[axis] = (float)combine(raw, axis) * BMI088_GYRO_TO_DPS;
    }
    /* Calibration always sees raw values, so restarting it on an already
     * calibrated board does not fold the old bias into the new one. */
    accumulate_calibration(dps);
    for (unsigned axis = 0; axis < 3U; ++axis) {
        sample.gyro_dps[axis] = dps[axis] - (calibration.calibrated ? calibration.bias_dps[axis] : 0.0f);
    }
    ++sample.gyro_updates;
    sample.last_gyro_ms = HAL_GetTick();
    return true;
}

HAL_StatusTypeDef BMI088_Init(SPI_HandleTypeDef *hspi)
{
    if (hspi == NULL || hspi->Instance != SPI1) {
        return HAL_ERROR;
    }

    imu_spi = hspi;
    imu_ready = false;
    pending_flags = 0U;
    sample = (BMI088_Sample){0};
    calibration = (BMI088_GyroCalibration){0};
    HAL_GPIO_WritePin(IMU_ACCEL_CS_GPIO_Port, IMU_ACCEL_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IMU_GYRO_CS_GPIO_Port, IMU_GYRO_CS_Pin, GPIO_PIN_SET);

    /* The first transaction selects SPI on the rising edge of CS. */
    HAL_Delay(1U);
    uint8_t chip_id = 0U;
    if (read_regs(&accel_target, BMI088_ACC_CHIP_ID_REG, &chip_id, 1U) != HAL_OK) {
        return HAL_ERROR;
    }
    if (write_reg(&accel_target, BMI088_ACC_SOFTRESET_REG, BMI088_SOFTRESET_CMD) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(BMI088_ACCEL_RESET_DELAY_MS);

    /* Soft reset selects I2C again. The first read only restores SPI. */
    if (read_regs(&accel_target, BMI088_ACC_CHIP_ID_REG, &chip_id, 1U) != HAL_OK) {
        return HAL_ERROR;
    }
    if (read_regs(&accel_target, BMI088_ACC_CHIP_ID_REG, &chip_id, 1U) != HAL_OK ||
        chip_id != BMI088_ACCEL_CHIP_ID) {
        return HAL_ERROR;
    }

    static const struct {
        uint8_t reg;
        uint8_t value;
    } accel_config[] = {
        {BMI088_ACC_PWR_CTRL_REG, BMI088_ACC_PWR_CTRL_ON},
        {BMI088_ACC_PWR_CONF_REG, BMI088_ACC_PWR_CONF_ACTIVE},
        {BMI088_ACC_CONF_REG, BMI088_ACC_CONF_800HZ},
        {BMI088_ACC_RANGE_REG, BMI088_ACC_RANGE_6G_CODE},
        {BMI088_ACC_INT1_IO_CTRL_REG, BMI088_ACC_INT_PUSH_PULL_ACTIVE_HIGH},
        {BMI088_ACC_INT2_IO_CTRL_REG, BMI088_ACC_INT_PUSH_PULL_ACTIVE_HIGH},
        {BMI088_ACC_INT_MAP_DATA_REG, BMI088_ACC_INT1_DRDY},
    };
    for (unsigned i = 0; i < sizeof accel_config / sizeof accel_config[0]; ++i) {
        if (write_reg(&accel_target, accel_config[i].reg, accel_config[i].value) != HAL_OK) {
            return HAL_ERROR;
        }
        /* Covers SPI write settling in suspend and the wake-up delay. */
        HAL_Delay(1U);
    }

    if (write_reg(&gyro_target, BMI088_GYRO_SOFTRESET_REG, BMI088_SOFTRESET_CMD) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(BMI088_GYRO_RESET_DELAY_MS);

    if (read_regs(&gyro_target, BMI088_GYRO_CHIP_ID_REG, &chip_id, 1U) != HAL_OK ||
        chip_id != BMI088_GYRO_CHIP_ID) {
        return HAL_ERROR;
    }

    static const struct {
        uint8_t reg;
        uint8_t value;
    } gyro_config[] = {
        {BMI088_GYRO_RANGE_REG, BMI088_GYRO_RANGE_2000_CODE},
        {BMI088_GYRO_BANDWIDTH_REG, BMI088_GYRO_BANDWIDTH_1000HZ_230HZ},
        {BMI088_GYRO_LPM1_REG, BMI088_GYRO_NORMAL_MODE},
        {BMI088_GYRO_INT3_INT4_IO_CONF_REG, BMI088_GYRO_INT3_PUSH_PULL_ACTIVE_HIGH},
        {BMI088_GYRO_INT3_INT4_IO_MAP_REG, BMI088_GYRO_DRDY_TO_INT3},
        {BMI088_GYRO_INT_CTRL_REG, BMI088_GYRO_DRDY_ON},
    };
    for (unsigned i = 0; i < sizeof gyro_config / sizeof gyro_config[0]; ++i) {
        if (write_reg(&gyro_target, gyro_config[i].reg, gyro_config[i].value) != HAL_OK) {
            return HAL_ERROR;
        }
        HAL_Delay(1U);
    }

    imu_ready = true;
    return HAL_OK;
}

void BMI088_ExtiCallback(uint16_t gpio_pin)
{
    if (!imu_ready) {
        return;
    }
    if (gpio_pin == IMU_ACCEL_INT_Pin) {
        pending_flags |= BMI088_FLAG_ACCEL_READY;
    } else if (gpio_pin == IMU_GYRO_INT_Pin) {
        pending_flags |= BMI088_FLAG_GYRO_READY;
    }
}

bool BMI088_Service(void)
{
    if (!imu_ready) {
        return false;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint8_t flags = pending_flags;
    pending_flags = 0U;
    __set_PRIMASK(primask);

    if ((flags & BMI088_FLAG_ACCEL_READY) != 0U) {
        read_accelerometer();
    }
    if ((flags & BMI088_FLAG_GYRO_READY) != 0U) {
        return read_gyroscope();
    }
    return false;
}

bool BMI088_GetSample(BMI088_Sample *out)
{
    if (out == NULL) {
        return false;
    }
    *out = sample;
    return imu_ready && sample.gyro_updates != 0U &&
           (uint32_t)(HAL_GetTick() - sample.last_gyro_ms) <= BMI088_TIMEOUT_MS;
}

bool BMI088_IsOnline(uint32_t timeout_ms)
{
    if (!imu_ready || sample.gyro_updates == 0U || timeout_ms == 0U) {
        return false;
    }
    return (uint32_t)(HAL_GetTick() - sample.last_gyro_ms) <= timeout_ms;
}

bool BMI088_StartGyroCalibration(uint16_t samples)
{
    if (!imu_ready || samples < 10U) {
        return false;
    }
    calibration.target = samples;
    calibration.collected = 0U;
    calibration.running = true;
    calibration.failed = false;
    calibration.spread_dps = 0.0f;
    for (unsigned axis = 0; axis < 3U; ++axis) {
        calibration_sum[axis] = 0.0f;
        calibration_min[axis] = 0.0f;
        calibration_max[axis] = 0.0f;
    }
    return true;
}

void BMI088_GetGyroCalibration(BMI088_GyroCalibration *out)
{
    if (out == NULL) {
        return;
    }
    *out = calibration;
}
