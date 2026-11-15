#ifndef BOARD_LINK_H
#define BOARD_LINK_H
#include "command.h"

/* Project-local v1 proposal; not an existing robot/vendor protocol.
 * Three standard CAN data frames per message, each DLC=8.
 * Reserve 0x300..302 command and 0x310..312 status after bus ID audit. */
typedef enum { LINK_COMMAND = 0, LINK_STATUS = 1 } LinkKind;
typedef struct { uint16_t id; uint8_t dlc; bool extended, remote; uint8_t data[8]; } LinkFrame;
typedef struct {
    float yaw_joint, pitch_joint; /* rad, wrapped yaw; status is not a control target */
    bool calibrated, motors_healthy, sensors_fresh, fault;
} LinkStatus;
typedef struct { RobotCommand command; LinkStatus status; } LinkMessage;
typedef struct {
    LinkKind kind;
    uint32_t assembly_timeout_ms, freshness_ms, assembly_started_ms, last_rx_ms;
    uint8_t parts[18], pending_seq, mask, last_seq;
    bool configured, received;
    LinkMessage message;
} LinkReceiver;

bool Link_EncodeCommand(const RobotCommand *cmd, uint8_t sequence, LinkFrame out[3]);
bool Link_EncodeStatus(const LinkStatus *status, uint8_t sequence, LinkFrame out[3]);
bool Link_Init(LinkReceiver *rx, LinkKind kind, uint32_t assembly_timeout_ms, uint32_t freshness_ms);
/* True only for a complete NEW CRC-valid message. Duplicate, old, incomplete
 * and malformed messages never refresh last_rx_ms. After startup/timeout a
 * command must be disarmed before the sequence baseline can be acquired.
 * For IWDG, require true AND message.command.remote_valid; never feed on status.
 * A CRC detects corruption, not malicious senders or deliberate replay. */
bool Link_Receive(LinkReceiver *rx, const LinkFrame *frame, uint32_t now_ms);
bool Link_IsFresh(const LinkReceiver *rx, uint32_t now_ms);
bool Link_GetCommand(const LinkReceiver *rx, uint32_t now_ms, RobotCommand *out);
#endif
