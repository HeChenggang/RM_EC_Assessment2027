#ifndef GIMBAL_BOARD_H
#define GIMBAL_BOARD_H

/* Assessment electrical topology, page 3. DM IDs use the document's naming:
 * RX_ID = TX_ID + 0x10. Wire protocol/mode must be confirmed before use. */
#define GIMBAL_YAW_CAN_BUS       1U
#define GIMBAL_YAW_TX_ID         0x03U
#define GIMBAL_YAW_RX_ID         0x13U
#define GIMBAL_PITCH_CAN_BUS     2U
#define GIMBAL_PITCH_TX_ID       0x09U
#define GIMBAL_PITCH_RX_ID       0x19U
#define GIMBAL_BOARD_LINK_CAN_BUS 1U

/* No motor enable/control is implemented by the board bring-up firmware. */

#endif
