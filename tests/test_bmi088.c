/* HAL test doubles: compiled for Cortex-M4 and run by run_motor_tests.py.
 *
 * The SPI double behaves like the two BMI088 dies behind one bus: which chip
 * answers is decided by the chip select that the driver left asserted, the
 * accelerometer consumes an extra dummy byte on reads, and registers
 * auto-increment so burst reads work. Nothing here proves electrical timing;
 * it proves the driver's frame layout and register sequence.
 */
#include "bmi088.h"
#include "main.h"
#include <stddef.h>

#define CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)
#define CHECK_NEAR(actual, expected, tolerance)                                          \
    do {                                                                                 \
        const float difference = (actual) - (expected);                                   \
        if (!(difference < (tolerance) && difference > -(tolerance))) return __LINE__;    \
    } while (0)

static uint8_t accel_regs[0x80];
static uint8_t gyro_regs[0x80];
static bool accel_selected, gyro_selected;
static HAL_StatusTypeDef spi_result = HAL_OK;
static uint32_t tick = 1000U;

uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t delay) { (void)delay; }

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    if (port == IMU_ACCEL_CS_GPIO_Port && pin == IMU_ACCEL_CS_Pin) {
        accel_selected = state == GPIO_PIN_RESET;
    } else if (port == IMU_GYRO_CS_GPIO_Port && pin == IMU_GYRO_CS_Pin) {
        gyro_selected = state == GPIO_PIN_RESET;
    }
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, const uint8_t *tx, uint8_t *rx,
                                          uint16_t size, uint32_t timeout)
{
    (void)hspi;
    (void)timeout;
    if (spi_result != HAL_OK) {
        return spi_result;
    }
    if (accel_selected == gyro_selected || size < 2U) {
        return HAL_ERROR; /* Exactly one die must be selected. */
    }
    uint8_t *regs = accel_selected ? accel_regs : gyro_regs;
    const unsigned skip = accel_selected ? 2U : 1U; /* Address byte (+ accel dummy). */

    if ((tx[0] & 0x80U) != 0U) {
        const unsigned address = tx[0] & 0x7FU;
        for (uint16_t i = 0; i < size; ++i) {
            rx[i] = i < skip ? 0xAAU : regs[(address + i - skip) & 0x7FU];
        }
    } else {
        regs[tx[0] & 0x7FU] = tx[1];
        for (uint16_t i = 0; i < size; ++i) {
            rx[i] = 0U;
        }
    }
    return HAL_OK;
}

static void put16(uint8_t *regs, unsigned address, int16_t value)
{
    regs[address] = (uint8_t)((uint16_t)value & 0xFFU);
    regs[address + 1U] = (uint8_t)((uint16_t)value >> 8);
}

static void set_gyro_dps(float x, float y, float z)
{
    const float scale = 32768.0f / 2000.0f;
    put16(gyro_regs, 0x02U, (int16_t)(x * scale));
    put16(gyro_regs, 0x04U, (int16_t)(y * scale));
    put16(gyro_regs, 0x06U, (int16_t)(z * scale));
}

static void feed_gyro(float x, float y, float z)
{
    set_gyro_dps(x, y, z);
    BMI088_ExtiCallback(IMU_GYRO_INT_Pin);
    (void)BMI088_Service();
}

int main(void)
{
    static SPI_HandleTypeDef spi, other;
    BMI088_Sample sample = {0};
    BMI088_GyroCalibration calibration = {0};

    spi.Instance = SPI1;
    other.Instance = SPI2;
    accel_regs[0x00] = 0x1EU;
    gyro_regs[0x00] = 0x0FU;

    /* Argument and state rejection. */
    CHECK(BMI088_Init(NULL) == HAL_ERROR);
    CHECK(BMI088_Init(&other) == HAL_ERROR);
    CHECK(!BMI088_Service());
    CHECK(!BMI088_GetSample(&sample));
    CHECK(!BMI088_GetSample(NULL));
    CHECK(!BMI088_IsOnline(100));
    CHECK(!BMI088_StartGyroCalibration(1000));
    BMI088_GetGyroCalibration(NULL);

    /* A wrong chip id must fail the init instead of configuring nothing. */
    accel_regs[0x00] = 0x1FU;
    CHECK(BMI088_Init(&spi) == HAL_ERROR);
    accel_regs[0x00] = 0x1EU;
    gyro_regs[0x00] = 0x0EU;
    CHECK(BMI088_Init(&spi) == HAL_ERROR);
    gyro_regs[0x00] = 0x0FU;
    CHECK(BMI088_Init(&spi) == HAL_OK);
    CHECK(BMI088_Init(&spi) == HAL_OK); /* Retrying after a failure is allowed. */

    /* Register sequence: values are the ones the datasheet requires for the
     * 800 Hz / 6 g / active-high data-ready configuration. */
    CHECK(accel_regs[0x7EU] == 0xB6U && gyro_regs[0x14U] == 0xB6U);
    CHECK(accel_regs[0x7DU] == 0x04U);
    CHECK(accel_regs[0x7CU] == 0x00U);
    CHECK(accel_regs[0x40U] == 0xA8U);
    CHECK(accel_regs[0x41U] == 0x01U);
    CHECK(accel_regs[0x53U] == 0x0AU);
    CHECK(accel_regs[0x54U] == 0x0AU);
    CHECK(accel_regs[0x58U] == 0x04U);
    CHECK(gyro_regs[0x0FU] == 0x00U);
    CHECK(gyro_regs[0x10U] == 0x01U);
    CHECK(gyro_regs[0x11U] == 0x00U);
    CHECK(gyro_regs[0x15U] == 0x80U);
    CHECK(gyro_regs[0x16U] == 0x00U);
    CHECK(gyro_regs[0x18U] == 0x01U);

    /* Accelerometer conversion: 16384 LSB is half of the 6 g range. */
    put16(accel_regs, 0x12U, 16384);
    put16(accel_regs, 0x14U, 0);
    put16(accel_regs, 0x16U, -16384);
    BMI088_ExtiCallback(IMU_ACCEL_INT_Pin);
    CHECK(!BMI088_Service()); /* No gyro sample yet, so control must not run. */
    CHECK(!BMI088_GetSample(&sample));
    CHECK_NEAR(sample.accel_mps2[0], 3.0f * 9.80665f, 0.05f);
    CHECK_NEAR(sample.accel_mps2[2], -3.0f * 9.80665f, 0.05f);
    CHECK(sample.accel_updates == 1U);

    /* Temperature: 11-bit two's complement, 0.125 degree steps, 23 degC offset.
     * 0x0150 >> 5 == 10 -> 1.25 degC above the offset. */
    BMI088_ExtiCallback(4U);
    CHECK(!BMI088_Service()); /* Unknown pin: nothing pending, no transfer. */
    accel_regs[0x22U] = 0x01U;
    accel_regs[0x23U] = 0x50U;
    BMI088_ExtiCallback(IMU_ACCEL_INT_Pin);
    CHECK(!BMI088_Service());
    CHECK(!BMI088_GetSample(&sample));
    CHECK_NEAR(sample.temperature_c, 24.25f, 0.02f);

    /* Gyroscope conversion, 2000 dps full scale. */
    set_gyro_dps(200.0f, -100.0f, 50.0f);
    BMI088_ExtiCallback(IMU_GYRO_INT_Pin);
    CHECK(BMI088_Service());
    CHECK(BMI088_GetSample(&sample));
    CHECK_NEAR(sample.gyro_dps[0], 200.0f, 0.1f);
    CHECK_NEAR(sample.gyro_dps[1], -100.0f, 0.1f);
    CHECK_NEAR(sample.gyro_dps[2], 50.0f, 0.1f);
    CHECK(BMI088_IsOnline(BMI088_TIMEOUT_MS));

    /* Zero-rate bias: a stationary window is averaged and then subtracted. */
    CHECK(!BMI088_StartGyroCalibration(9));
    CHECK(BMI088_StartGyroCalibration(100));
    for (unsigned i = 0; i < 100U; ++i) {
        feed_gyro(10.0f, -5.0f, 2.0f);
    }
    BMI088_GetGyroCalibration(&calibration);
    CHECK(calibration.calibrated && !calibration.failed && !calibration.running);
    CHECK(calibration.collected == 100U && calibration.target == 100U);
    CHECK_NEAR(calibration.bias_dps[0], 10.0f, 0.1f);
    CHECK_NEAR(calibration.bias_dps[1], -5.0f, 0.1f);
    CHECK(calibration.spread_dps < BMI088_CALIBRATION_MAX_SPREAD_DPS);

    feed_gyro(10.0f, -5.0f, 2.0f);
    CHECK(BMI088_GetSample(&sample));
    CHECK_NEAR(sample.gyro_dps[0], 0.0f, 0.1f);
    CHECK_NEAR(sample.gyro_dps[1], 0.0f, 0.1f);

    /* A moving window is rejected and the previous bias survives. */
    CHECK(BMI088_StartGyroCalibration(20));
    for (unsigned i = 0; i < 20U; ++i) {
        feed_gyro((i % 2U) == 0U ? 0.0f : 100.0f, 0.0f, 0.0f);
    }
    BMI088_GetGyroCalibration(&calibration);
    CHECK(calibration.failed && calibration.calibrated);
    CHECK(calibration.spread_dps > 90.0f);
    CHECK_NEAR(calibration.bias_dps[0], 10.0f, 0.1f);

    /* Bus errors are counted, do not publish a sample and keep the driver usable. */
    const uint32_t errors = sample.spi_errors;
    spi_result = HAL_ERROR;
    BMI088_ExtiCallback(IMU_GYRO_INT_Pin);
    CHECK(!BMI088_Service());
    spi_result = HAL_OK;
    CHECK(BMI088_GetSample(&sample) && sample.spi_errors == errors + 1U);

    /* Freshness: the sample goes stale on its own and reports as such. */
    tick += BMI088_TIMEOUT_MS;
    CHECK(BMI088_IsOnline(BMI088_TIMEOUT_MS));
    tick += 1U;
    CHECK(!BMI088_IsOnline(BMI088_TIMEOUT_MS));
    CHECK(!BMI088_IsOnline(0));
    CHECK(!BMI088_GetSample(&sample));
    CHECK(sample.gyro_updates > 0U); /* Stale data is still visible to the debugger. */
    return 0;
}
