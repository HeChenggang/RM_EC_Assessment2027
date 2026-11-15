#include "attitude.h"
#include "motion_math.h"
#include <math.h>
#define CHECK(c) do { if (!(c)) return __LINE__; } while (0)
#define NEAR(a,b,e) (fabsf((a)-(b)) < (e))
int main(void)
{
    AttitudeConfig config = {.sensor_to_body = {{1,0,0},{0,1,0},{0,0,1}},
        .gravity = 9.80665f, .accel_tolerance = 0.2f, .correction_gain = 2,
        .max_dt = 0.02f, .max_gyro = 40};
    Attitude s; AttitudeOutput out;
    float a[3] = {0,0,9.80665f}, w[3] = {0};
    CHECK(Attitude_Init(&s, &config));
    for (unsigned i = 0; i < 100; ++i) CHECK(Attitude_Update(&s, a, w, 0.01f, &out));
    CHECK(NEAR(out.roll, 0, 1e-5f) && NEAR(out.pitch, 0, 1e-5f) && NEAR(out.yaw, 0, 1e-5f));
    CHECK(out.accel_used);
    w[2] = MOTION_PI/2;
    for (unsigned i = 0; i < 100; ++i) CHECK(Attitude_Update(&s, a, w, 0.01f, &out));
    CHECK(NEAR(out.yaw, MOTION_PI/2, 0.001f) && NEAR(out.euler_rate[2], w[2], 1e-5f));
    float norm = 0; for (unsigned i = 0; i < 4; ++i) norm += s.q[i]*s.q[i];
    CHECK(NEAR(norm, 1, 1e-5f));
    for (unsigned i = 0; i < 400; ++i) CHECK(Attitude_Update(&s, a, w, 0.01f, &out));
    CHECK(NEAR(out.yaw, MOTION_PI/2, 0.002f));
    a[2] = 30;
    CHECK(Attitude_Update(&s, a, w, 0.01f, &out) && !out.accel_used);
    CHECK(!Attitude_Update(&s, a, w, 0, &out) && !s.initialized);
    CHECK(!Attitude_Update(&s, a, w, 0.01f, &out)); /* cannot initialize from acceleration burst */

    /* Known static roll/pitch, up-vector = R^T * world-up. */
    const float roll = 0.3f, pitch = -0.4f;
    a[0] = -sinf(pitch)*config.gravity;
    a[1] = sinf(roll)*cosf(pitch)*config.gravity;
    a[2] = cosf(roll)*cosf(pitch)*config.gravity;
    w[2] = 0;
    CHECK(Attitude_Update(&s, a, w, 0.01f, &out));
    CHECK(NEAR(out.roll, roll, 1e-5f) && NEAR(out.pitch, pitch, 1e-5f) && NEAR(out.yaw, 0, 1e-5f));
    /* Tilt correction must converge back to level when static gravity changes. */
    a[0] = a[1] = 0; a[2] = config.gravity;
    for (unsigned i = 0; i < 500; ++i) CHECK(Attitude_Update(&s, a, w, 0.01f, &out));
    CHECK(fabsf(out.roll) < 0.001f && fabsf(out.pitch) < 0.001f);
    a[0] = NAN;
    CHECK(!Attitude_Update(&s, a, w, 0.01f, &out) && !s.initialized);
    a[0] = 0; w[0] = INFINITY;
    CHECK(!Attitude_Update(&s, a, w, 0.01f, &out));
    w[0] = 41;
    CHECK(!Attitude_Update(&s, a, w, 0.01f, &out));
    w[0] = 0;
    CHECK(!Attitude_Update(&s, a, w, 0.03f, &out));

    /* Sensor Y maps to body Z under a proper +90 degree X rotation. */
    config.sensor_to_body[1][1] = 0; config.sensor_to_body[1][2] = -1;
    config.sensor_to_body[2][1] = 1; config.sensor_to_body[2][2] = 0;
    CHECK(Attitude_Init(&s, &config));
    a[1] = config.gravity; a[2] = 0; w[1] = 1;
    CHECK(Attitude_Update(&s, a, w, 0.01f, &out));
    CHECK(NEAR(out.roll, 0, 1e-5f) && NEAR(out.pitch, 0, 1e-5f) && NEAR(out.yaw, 0.01f, 1e-5f));
    CHECK(NEAR(out.euler_rate[2], 1, 1e-5f));
    config.sensor_to_body[0][0] = -1; /* reflection cannot be a mounting rotation */
    CHECK(!Attitude_Init(&s, &config));
    config.sensor_to_body[0][0] = NAN;
    CHECK(!Attitude_Init(&s, &config));
    config.sensor_to_body[0][0] = 1;
    CHECK(Attitude_Init(&s, &config));
    a[0] = -config.gravity; a[1] = a[2] = 0; w[1] = 0;
    CHECK(!Attitude_Update(&s, a, w, 0.01f, &out)); /* Euler pitch singularity */
    return 0;
}
