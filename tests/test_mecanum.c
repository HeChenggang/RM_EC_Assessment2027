#include "mecanum.h"
#include <float.h>
#include <math.h>
#include <stddef.h>

#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
#define NEAR(a,b) (fabsf((a)-(b)) < 0.002f)

int main(void)
{
    Mecanum_Config c = MECANUM_DEFAULT_CONFIG;
    Mecanum_Velocity v = {0}, measured;
    Mecanum_Wheels w;
    CHECK(NEAR(c.track_m, 0.45f) && NEAR(c.wheelbase_m, 0.55f));
    CHECK(Mecanum_Inverse(&c, &v, &w) && w.scale == 1);
    for (unsigned i = 0; i < 4; ++i) CHECK(w.motor_rpm[i] == 0);
    /* One wheel revolution/sec -> 60 wheel rpm -> 840 rotor rpm. */
    v.vx = 6.28318530718f * 0.07786f;
    CHECK(Mecanum_Inverse(&c, &v, &w));
    for (unsigned i = 0; i < 4; ++i) CHECK(NEAR(w.motor_rpm[i], 840));
    v.vy = v.vx; v.vx = 0;
    CHECK(Mecanum_Inverse(&c, &v, &w));
    CHECK(NEAR(w.motor_rpm[0], -840) && NEAR(w.motor_rpm[1], 840));
    CHECK(NEAR(w.motor_rpm[2], -840) && NEAR(w.motor_rpm[3], 840));
    v.wz = v.vy / 0.5f; v.vy = 0;
    CHECK(Mecanum_Inverse(&c, &v, &w));
    CHECK(NEAR(w.motor_rpm[0], -840) && NEAR(w.motor_rpm[1], -840));
    CHECK(NEAR(w.motor_rpm[2], 840) && NEAR(w.motor_rpm[3], 840));
    v = (Mecanum_Velocity){1, 1, 0};
    CHECK(Mecanum_Inverse(&c, &v, &w));
    CHECK(w.motor_rpm[0] == 0 && w.motor_rpm[2] == 0);
    c.motor_direction[2] = c.motor_direction[3] = -1;
    for (int i = -5; i <= 5; ++i) {
        v = (Mecanum_Velocity){i * 0.2f, i * -0.13f, i * 0.31f};
        CHECK(Mecanum_Inverse(&c, &v, &w));
        CHECK(Mecanum_Forward(&c, w.motor_rpm, &measured));
        CHECK(NEAR(measured.vx, v.vx) && NEAR(measured.vy, v.vy) && NEAR(measured.wz, v.wz));
    }
    c.max_motor_rpm = 1000;
    CHECK(Mecanum_Inverse(&c, &v, &w) && w.scale < 1 && w.scale > 0);
    for (unsigned i = 0; i < 4; ++i) CHECK(fabsf(w.motor_rpm[i]) <= 1000.001f);
    CHECK(Mecanum_Forward(&c, w.motor_rpm, &measured));
    CHECK(NEAR(measured.vx, v.vx*w.scale) && NEAR(measured.vy, v.vy*w.scale));
    CHECK(NEAR(measured.wz, v.wz*w.scale));
    c.radius_m = 0;
    CHECK(!Mecanum_Inverse(&c, &v, &w) && w.scale == 0);
    c = MECANUM_DEFAULT_CONFIG; c.motor_direction[1] = 0;
    CHECK(!Mecanum_Inverse(&c, &v, &w));
    c = MECANUM_DEFAULT_CONFIG; c.max_motor_rpm = -1;
    CHECK(!Mecanum_Inverse(&c, &v, &w));
    c = MECANUM_DEFAULT_CONFIG; v.vx = NAN;
    CHECK(!Mecanum_Inverse(&c, &v, &w));
    v.vx = FLT_MAX;
    CHECK(!Mecanum_Inverse(&c, &v, &w));
    for (unsigned i = 0; i < 4; ++i) CHECK(w.motor_rpm[i] == 0);
    CHECK(!Mecanum_Inverse(NULL, &v, &w));
    CHECK(!Mecanum_Inverse(&c, NULL, &w));
    CHECK(!Mecanum_Inverse(&c, &v, NULL));
    w.motor_rpm[2] = INFINITY;
    CHECK(!Mecanum_Forward(&c, w.motor_rpm, &measured) && measured.vx == 0);
    CHECK(!Mecanum_Forward(&c, NULL, &measured));
    CHECK(!Mecanum_Forward(&c, w.motor_rpm, NULL));
    return 0;
}
