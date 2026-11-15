#ifndef ATTITUDE_H
#define ATTITUDE_H
#include <stdbool.h>
typedef struct {
    float sensor_to_body[3][3]; /* orthonormal, det +1; explicitly configured */
    float gravity, accel_tolerance, correction_gain, max_dt, max_gyro;
} AttitudeConfig;
typedef struct {
    AttitudeConfig config;
    float q[4]; /* w,x,y,z; body -> world, world z up */
    bool configured, initialized;
} Attitude;
typedef struct {
    float roll, pitch, yaw; /* ZYX Euler radians, right handed */
    float euler_rate[3]; /* roll,pitch,yaw rad/s; reject near pitch singularity */
    bool accel_used;
} AttitudeOutput;
bool Attitude_Init(Attitude *state, const AttitudeConfig *config);
void Attitude_Reset(Attitude *state);
/* Input acceleration m/s^2, bias-corrected gyro rad/s (BMI088 gives dps:
 * caller must convert). First stationary sample determines roll/pitch, yaw=0.
 * Out-of-band acceleration: gyro-only propagation after initialization.
 * Invalid samples/dt invalidate orientation and require fresh initialization.
 * No magnetometer: yaw is relative and will drift. No gyro-bias estimation here. */
bool Attitude_Update(Attitude *state, const float accel[3], const float gyro[3],
                     float dt, AttitudeOutput *out);
#endif
