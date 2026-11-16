#ifndef BMI088_H
#define BMI088_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Full-scale settings written by BMI088_Init; the conversions below assume them. */
#define BMI088_ACCEL_RANGE_G 6.0f
#define BMI088_GYRO_RANGE_DPS 2000.0f

/* Gyro samples averaged for the zero-rate bias. At 1 kHz this is one second. */
#define BMI088_CALIBRATION_SAMPLES 1000U
/* Motion rejection for the calibration window. Two statistics are checked
 * because they fail in different ways:
 *
 *  - drift (BMI088_CALIBRATION_MAX_DRIFT_DPS) is the real gate: the difference
 *    between the mean of the first and of the second half of the window. For a
 *    stationary board (0.014 dps/sqrt(Hz), 230 Hz filter -> sigma ~0.27 dps)
 *    each half mean has sigma ~0.27/sqrt(500), so the difference is ~0.02 dps.
 *    One isolated bad sample only moves it by glitch/(N/2). A constant rotation
 *    of r dps moves it by r, which is exactly what a peak-to-peak test misses.
 *
 *  - spread (peak-to-peak) is an extreme-value statistic: it is set by the
 *    single worst sample out of N, and white noise alone reaches
 *    2*sigma*sqrt(2*ln(N)) ~ 2 dps for 1000 samples. A real board measured
 *    4.5 dps while standing still, so peak-to-peak cannot be the motion limit;
 *    it stays as a corruption guard at 20 dps, ~10x the stationary value, and
 *    only trips on a bus error or bit-flip storm. */
#define BMI088_CALIBRATION_MAX_SPREAD_DPS 20.0f
#define BMI088_CALIBRATION_MAX_DRIFT_DPS 1.0f
/* Data-ready silence after which the sample counts as stale. */
#define BMI088_TIMEOUT_MS 20U

/* Sensor axes, not chassis axes: the installation rotation belongs to the
 * attitude code, together with the mounting matrix of the actual board. */
typedef struct {
    float accel_mps2[3]; /* Gravity is NOT removed. */
    float gyro_dps[3];   /* Zero-rate bias removed once a calibration succeeded. */
    float temperature_c;
    uint32_t accel_updates;
    uint32_t gyro_updates;
    uint32_t spi_errors;
    uint32_t last_accel_ms;
    uint32_t last_gyro_ms;
} BMI088_Sample;

typedef struct {
    bool calibrated; /* Bias is valid and applied to every new gyro sample. */
    bool failed;     /* Last window was rejected; the old bias is kept. */
    bool running;
    uint16_t collected;
    uint16_t target;
    float bias_dps[3];
    float spread_dps; /* Peak-to-peak of the window, worst axis: diagnostic. */
    float drift_dps;  /* Mean(second half) - mean(first half), worst axis: the
                       * motion statistic the accept/reject decision uses. */
    uint32_t window_ms; /* Diagnostic: wall time the window really took. Much
                         * more than target/ODR means samples were dropped
                         * because the main loop runs slower than data ready. */
} BMI088_GyroCalibration;

/* Configures both dies over SPI1 and verifies their chip IDs. Blocking for about
 * 35 ms of datasheet delays, so it must not be called where that stall matters.
 * Safe to call again to retry after a failed attempt; the sample is cleared. */
HAL_StatusTypeDef BMI088_Init(SPI_HandleTypeDef *hspi);

/* Forward from HAL_GPIO_EXTI_Callback. Only records which die is ready; the SPI
 * transfer happens later in BMI088_Service, never inside the interrupt. */
void BMI088_ExtiCallback(uint16_t gpio_pin);

/* Call from the main loop. Reads whatever the data-ready interrupts flagged and
 * returns true when a NEW gyro sample was stored, i.e. when control may run.
 * Reading the accelerometer rotates the axes of accel_mps2 into sample form, so
 * a caller that ignores the return value still gets fresh accelerometer data. */
bool BMI088_Service(void);

/* Copies the latest snapshot. out is always filled with the last known values
 * and diagnostics; the return value tells whether they are fresh and usable. */
bool BMI088_GetSample(BMI088_Sample *out);
bool BMI088_IsOnline(uint32_t timeout_ms);

/* Arms the zero-rate bias window. The chassis must be stationary and the
 * watchdog must keep being fed elsewhere while samples accumulate; this call
 * does not block. Fails until BMI088_Init succeeded or for samples < 10. */
bool BMI088_StartGyroCalibration(uint16_t samples);
/* Snapshot of the calibration state, for the debugger and the safety logic. */
void BMI088_GetGyroCalibration(BMI088_GyroCalibration *out);

#ifdef __cplusplus
}
#endif
#endif
