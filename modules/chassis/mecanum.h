#ifndef CHASSIS_MECANUM_H
#define CHASSIS_MECANUM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Standard 45-degree X mecanum layout; order matches CAN motor IDs 1..4. */
enum { MECANUM_FL = 0, MECANUM_RL, MECANUM_RR, MECANUM_FR, MECANUM_WHEEL_COUNT };
typedef struct {
    float vx; /* m/s, forward positive, chassis center */
    float vy; /* m/s, left positive */
    float wz; /* rad/s, counter-clockwise viewed from above */
} Mecanum_Velocity;

typedef struct {
    float track_m;         /* Full left-right wheel center distance. */
    float wheelbase_m;     /* Full front-rear wheel center distance. */
    float radius_m;
    float reduction;       /* Motor rotor rpm / wheel rpm. */
    int8_t motor_direction[4]; /* Each +1 or -1; verify on the real robot. */
    float max_motor_rpm;   /* Absolute rotor speed limit; 0 disables. */
} Mecanum_Config;

typedef struct {
    float motor_rpm[4];    /* Signed rotor targets, for speed PID, NOT CAN current. */
    float scale;           /* 0..1; applied to all wheels, 1 if not limited. */
} Mecanum_Wheels;

/* Geometry confirmed by user; radius/reduction from assessment document.
 * Directions +1 are mathematical defaults, NOT hardware calibration. */
extern const Mecanum_Config MECANUM_DEFAULT_CONFIG;

/* Fit a velocity to the wheel speed limit before inverse kinematics. Scale
 * translation uniformly and reduce rotation only if pure rotation is too fast. */
bool Mecanum_PrioritizeRotation(const Mecanum_Config *config, Mecanum_Velocity *velocity);
/* No HAL/state/allocation. Invalid input or overflow returns false and zeros
 * output. Use separate input/output objects (no aliasing).
 * Caller handles feedback freshness, arming and PID/current control. */
bool Mecanum_Inverse(const Mecanum_Config *config, const Mecanum_Velocity *velocity,
                     Mecanum_Wheels *out);
/* Forward kinematics from measured signed rotor rpm, no speed clipping.
 * Ideal no-slip estimate; not fused odometry or an absolute pose. */
bool Mecanum_Forward(const Mecanum_Config *config, const float motor_rpm[4],
                     Mecanum_Velocity *out);

#ifdef __cplusplus
}
#endif
#endif
