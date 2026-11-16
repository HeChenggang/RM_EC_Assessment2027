#include "pid.h"
#include <float.h>
#include <math.h>
#include <stddef.h>

#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
#define NEAR(a, b) (fabsf((a) - (b)) < 0.001f)

int main(void)
{
    PID_Controller pid = {0}, other = {0};
    PID_Config c = { .kp = 2, .ki = 0, .kd = 0, .output_limit = 10,
        .integral_limit = 4, .derivative_tau = 0, .max_dt = 1 };
    float out = 99;
    CHECK(!PID_Update(&pid, 1, 0, 0.01f, &out) && out == 0);
    CHECK(!PID_Init(NULL, &c));
    CHECK(PID_Init(&pid, &c));
    CHECK(PID_Update(&pid, 3, 1, 0.01f, &out) && out == 4);
    CHECK(PID_Update(&pid, 100, 0, 0.01f, &out) && out == 10);
    CHECK(PID_Update(&pid, -100, 0, 0.01f, &out) && out == -10);

    c.kp = 0; c.ki = 2;
    CHECK(PID_Init(&pid, &c) && PID_Init(&other, &c));
    for (unsigned i = 0; i < 10; ++i) CHECK(PID_Update(&pid, 1, 0, 0.1f, &out));
    CHECK(NEAR(out, 2));
    CHECK(PID_Update(&other, 1, 0, 1, &out) && NEAR(out, 2));
    CHECK(NEAR(pid.i, other.i)); /* dt scaling and independent instances */
    for (unsigned i = 0; i < 40; ++i) CHECK(PID_Update(&pid, 1, 0, 0.1f, &out));
    CHECK(NEAR(pid.i, 4));
    PID_Reset(&pid);
    CHECK(pid.configured && pid.i == 0 && pid.output == 0 && !pid.has_previous_measurement);

    c.kp = 2; c.ki = 1; c.integral_limit = 100;
    CHECK(PID_Init(&pid, &c));
    for (unsigned i = 0; i < 50; ++i) CHECK(PID_Update(&pid, 100, 0, 0.1f, &out));
    CHECK(pid.i == 0 && out == 10); /* no accumulated windup */
    CHECK(PID_Update(&pid, 0, 0, 0.1f, &out) && out == 0);
    CHECK(PID_Update(&pid, 1, 0, 1, &out) && pid.i == 1);
    CHECK(PID_Update(&pid, -1, 0, 0.1f, &out) && NEAR(pid.i, 0.9f));

    c.kp = 0; c.ki = 0; c.kd = 2; c.output_limit = 100;
    CHECK(PID_Init(&pid, &c));
    CHECK(PID_Update(&pid, 0, 100, 0.1f, &out) && out == 0); /* first sample */
    CHECK(PID_Update(&pid, 50, 100, 0.1f, &out) && out == 0); /* no setpoint kick */
    CHECK(PID_Update(&pid, 50, 101, 0.1f, &out) && NEAR(out, -20));
    c.derivative_tau = 0.1f;
    CHECK(PID_Init(&pid, &c));
    CHECK(PID_Update(&pid, 0, 0, 0.1f, &out));
    CHECK(PID_Update(&pid, 0, 1, 0.1f, &out) && NEAR(out, -10));
    CHECK(PID_Update(&pid, 0, 1, 0.1f, &out) && NEAR(out, -5));

    const float bad_dt[] = {0, -1, 2, NAN, INFINITY};
    for (unsigned i = 0; i < 5; ++i) {
        CHECK(!PID_Update(&pid, 1, 0, bad_dt[i], &out));
        CHECK(out == 0 && pid.i == 0 && !pid.has_previous_measurement);
    }
    CHECK(!PID_Update(&pid, NAN, 0, 0.1f, &out));
    CHECK(!PID_Update(&pid, 0, INFINITY, 0.1f, &out));
    CHECK(!PID_Update(&pid, 0, 0, 0.1f, NULL));
    CHECK(!PID_Update(NULL, 0, 0, 0.1f, &out) && out == 0);
    c.kp = FLT_MAX;
    CHECK(PID_Init(&pid, &c));
    CHECK(!PID_Update(&pid, 2, 0, 0.1f, &out) && out == 0);
    c.kp = -1;
    CHECK(!PID_Init(&pid, &c) && !pid.configured);
    CHECK(!PID_Init(&pid, NULL));
    c.kp = 0; c.output_limit = 0;
    CHECK(!PID_Init(&pid, &c));
    c.output_limit = 10; c.ki = NAN;
    CHECK(!PID_Init(&pid, &c));

    /* Added holding torque must share saturation and prevent windup. */
    c = (PID_Config){.kp=2,.ki=1,.output_limit=10,.integral_limit=10,.max_dt=1};
    CHECK(PID_Init(&pid, &c));
    for (unsigned i=0; i<50; ++i)
        CHECK(PID_UpdateWithFeedforward(&pid, 1, 0, 9, 0.1f, &out));
    CHECK(out==10 && pid.i==0);
    CHECK(PID_UpdateWithFeedforward(&pid, -1, 0, -9, 0.1f, &out));
    CHECK(out==-10 && pid.i==0);
    CHECK(PID_UpdateWithFeedforward(&pid, 1, 0, 0, 1, &out) && pid.i==1);
    CHECK(PID_UpdateWithFeedforward(&pid, -1, 0, 20, 0.1f, &out));
    CHECK(out==10 && NEAR(pid.i,0.9f)); /* allow unwind while saturated */
    CHECK(!PID_UpdateWithFeedforward(&pid, 0, 0, NAN, 0.1f, &out));
    CHECK(out==0 && pid.i==0);

    /* A simple first-order simulated plant: PI should track a unit step. */
    c = (PID_Config){.kp = 2, .ki = 4, .output_limit = 2, .integral_limit = 2,
        .max_dt = 0.02f};
    CHECK(PID_Init(&pid, &c));
    float measurement = 0;
    for (unsigned i = 0; i < 500; ++i) {
        CHECK(PID_Update(&pid, 1, measurement, 0.01f, &out));
        CHECK(out >= -2 && out <= 2);
        measurement += 0.02f * (out - measurement);
    }
    CHECK(fabsf(measurement - 1) < 0.01f);
    CHECK(PID_Init(&pid, &pid.config) && pid.i == 0);
    return 0;
}
