#ifndef BOARD_LINK_SETUP_H
#define BOARD_LINK_SETUP_H

/* Shared timing for the 0x300..0x302 command and 0x310..0x312 status messages.
 * The sender emits one status frame per main-loop pass, leaving at least one
 * CAN1 mailbox for the 200 Hz yaw torque frame.
 * The receiver accepts three fragments within 50 ms and treats 100 ms without
 * a complete new CRC-valid message as stale. */
#define BOARD_LINK_STATUS_PERIOD_MS 20U
#define BOARD_LINK_ASSEMBLY_MS 50U
#define BOARD_LINK_FRESH_MS 100U

#endif
