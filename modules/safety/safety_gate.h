#ifndef SAFETY_GATE_H
#define SAFETY_GATE_H
#include "command.h"
typedef enum { SAFETY_CALIBRATING, SAFETY_DISARMED, SAFETY_ACTIVE, SAFETY_FAULT } SafetyState;
typedef struct { SafetyState state; bool ready_to_arm; } SafetyGate;
typedef struct {
    bool link_fresh, sensors_fresh, motors_healthy;
    bool calibrated, calibration_failed, hardware_configured;
} SafetyInputs;
/* Zero-init is CALIBRATING. hardware_configured is an explicit integration
 * interlock; software sample configs must never set it in production.
 * After ANY stop/fault, require a healthy neutral DISARM frame, followed by a
 * neutral ARM frame. Held ARM at startup/recovery cannot resume motion. */
bool Safety_Step(SafetyGate *gate, const RobotCommand *cmd, const SafetyInputs *input);
#endif
