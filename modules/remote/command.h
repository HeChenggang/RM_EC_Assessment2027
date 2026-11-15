#ifndef ROBOT_COMMAND_H
#define ROBOT_COMMAND_H
#include <stdbool.h>
#include <stdint.h>

typedef enum { MODE_SEPARATE = 0, MODE_FOLLOW = 1, MODE_SPIN = 2 } DriveMode;
typedef struct {
    float forward, left, yaw, pitch; /* normalized [-1, 1] */
    DriveMode mode;
    bool remote_valid, arm, estop;
} RobotCommand;

/* Values match DR16's raw switch encoding, but physical switch placement and
 * channel signs must be checked. Explicit configuration, no hardware defaults. */
typedef struct {
    int16_t channel[5];
    uint8_t mode_switch, arm_switch;
    bool fresh;
} RemoteInput;
typedef struct {
    uint8_t channel[4]; /* forward, left, yaw, pitch */
    int8_t direction[4];
    float deadzone; /* [0, 1), rescaled continuously outside the deadzone */
    uint8_t separate_switch, follow_switch, spin_switch;
    uint8_t stop_switch, disarm_switch, enable_switch;
} RemoteMap;

bool Command_Valid(const RobotCommand *cmd);
bool Command_Neutral(const RobotCommand *cmd, float threshold);
bool Remote_Map(const RemoteMap *map, const RemoteInput *raw, RobotCommand *out);
#endif
