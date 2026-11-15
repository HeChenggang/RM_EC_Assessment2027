#include "board_link.h"
#include "motion_math.h"
#include <math.h>
#include <stddef.h>
#define CHECK(c) do { if (!(c)) return __LINE__; } while (0)
static bool deliver(LinkReceiver *rx, LinkFrame frames[3], uint32_t now)
{
    (void)Link_Receive(rx, &frames[2], now);
    (void)Link_Receive(rx, &frames[0], now+1);
    return Link_Receive(rx, &frames[1], now+2);
}
int main(void)
{
    LinkReceiver rx; LinkFrame frames[3]; RobotCommand out;
    RobotCommand cmd = {.remote_valid = true, .mode = MODE_FOLLOW, .forward = 1,
        .left = -1, .yaw = 0.1234f, .pitch = -0.25f};
    CHECK(!Link_Init(NULL, LINK_COMMAND, 10, 100));
    CHECK(!Link_Init(&rx, LINK_COMMAND, 100, 100));
    CHECK(!Link_Init(&rx, (LinkKind)2, 10, 100));
    CHECK(Link_Init(&rx, LINK_COMMAND, 10, 100));
    CHECK(!Link_GetCommand(&rx, 0, &out) && out.estop && !out.remote_valid);
    CHECK(Link_EncodeCommand(&cmd, 254, frames));
    CHECK(frames[0].id == 0x300 && frames[1].id == 0x301 && frames[2].id == 0x302);
    CHECK(frames[0].data[0] == 1 && frames[0].data[1] == 254);
    CHECK(frames[0].data[2] == 1 && frames[0].data[3] == 1);
    CHECK(frames[0].data[4] == 0x10 && frames[0].data[5] == 0x27); /* +10000 */
    CHECK(frames[0].data[6] == 0xf0 && frames[0].data[7] == 0xd8); /* -10000 */
    /* Independent golden CRC: Python binascii.crc_hqx on version/kind/seq/payload. */
    CHECK(frames[2].data[6] == 0xb4 && frames[2].data[7] == 0x88);
    CHECK(deliver(&rx, frames, 0xfffffff0U));
    CHECK(Link_GetCommand(&rx, 1, &out));
    CHECK(out.mode == MODE_FOLLOW && out.forward == 1 && out.left == -1);
    CHECK(fabsf(out.yaw-0.1234f) < 1e-6f && out.pitch == -0.25f);
    CHECK(!deliver(&rx, frames, 2)); /* duplicate */
    CHECK(rx.last_rx_ms == 0xfffffff2U);
    CHECK(Link_EncodeCommand(&cmd, 253, frames) && !deliver(&rx, frames, 5)); /* old */
    cmd.arm = true;
    CHECK(Link_EncodeCommand(&cmd, 255, frames) && deliver(&rx, frames, 10));
    CHECK(Link_EncodeCommand(&cmd, 0, frames) && deliver(&rx, frames, 20));
    CHECK(Link_IsFresh(&rx, 121) && !Link_IsFresh(&rx, 122));
    CHECK(!Link_GetCommand(&rx, 122, &out) && out.estop && !out.arm);
    CHECK(Link_EncodeCommand(&cmd, 1, frames) && !deliver(&rx, frames, 130)); /* resync ARM rejected */
    cmd.arm = false;
    CHECK(Link_EncodeCommand(&cmd, 1, frames) && deliver(&rx, frames, 140));

    /* Every single-bit corruption in payload or CRC must fail without changing
     * the previous message timestamp. Header version/sequence get own tests. */
    for (unsigned byte = 0; byte < 18; ++byte) {
        for (unsigned bit = 0; bit < 8; ++bit) {
            CHECK(Link_EncodeCommand(&cmd, 2, frames));
            frames[byte/6].data[2+byte%6] ^= (uint8_t)(1U<<bit);
            CHECK(!deliver(&rx, frames, 150) && rx.last_rx_ms == 142);
        }
    }
    CHECK(Link_EncodeCommand(&cmd, 2, frames));
    frames[0].extended = true; CHECK(!deliver(&rx, frames, 151));
    CHECK(Link_Init(&rx, LINK_COMMAND, 10, 100));
    CHECK(Link_EncodeCommand(&cmd, 2, frames));
    frames[0].remote = true; CHECK(!deliver(&rx, frames, 1));
    CHECK(Link_Init(&rx, LINK_COMMAND, 10, 100));
    frames[0].remote = false; frames[0].dlc = 7; CHECK(!deliver(&rx, frames, 1));
    CHECK(Link_Init(&rx, LINK_COMMAND, 10, 100));
    CHECK(Link_EncodeCommand(&cmd, 2, frames));
    frames[0].data[0] = 2; CHECK(!deliver(&rx, frames, 1));
    CHECK(Link_Init(&rx, LINK_COMMAND, 10, 100));
    CHECK(Link_EncodeCommand(&cmd, 2, frames));
    CHECK(!Link_Receive(&rx, &frames[0], 1));
    CHECK(!Link_Receive(&rx, &frames[1], 11)); /* assembly expired */
    CHECK(!Link_Receive(&rx, &frames[2], 12));
    CHECK(!rx.received);
    CHECK(Link_Init(&rx, LINK_COMMAND, 10, 100));
    CHECK(!Link_Receive(&rx, &frames[0], 1));
    CHECK(!Link_Receive(&rx, &frames[0], 2)); /* duplicate fragment */
    CHECK(!Link_Receive(&rx, &frames[1], 3));
    CHECK(Link_Receive(&rx, &frames[2], 4));
    CHECK(Link_EncodeCommand(&cmd, 3, frames));
    CHECK(!Link_Receive(&rx, &frames[0], 5));
    CHECK(Link_EncodeCommand(&cmd, 4, frames));
    CHECK(!Link_Receive(&rx, &frames[1], 6));
    CHECK(!Link_Receive(&rx, &frames[2], 7));
    CHECK(Link_Receive(&rx, &frames[0], 8)); /* newer message replaces partial */
    CHECK(rx.last_seq == 4);
    cmd.forward = NAN; CHECK(!Link_EncodeCommand(&cmd, 5, frames));
    cmd.forward = 1.01f; CHECK(!Link_EncodeCommand(&cmd, 5, frames));
    CHECK(!Link_EncodeCommand(NULL, 5, frames));

    LinkStatus status = {.yaw_joint = -MOTION_PI, .pitch_joint = MOTION_PI/2,
        .calibrated = true, .motors_healthy = true, .sensors_fresh = true, .fault = false};
    CHECK(Link_Init(&rx, LINK_STATUS, 10, 100));
    CHECK(Link_EncodeStatus(&status, 17, frames) && frames[0].id == 0x310);
    CHECK(deliver(&rx, frames, 1));
    CHECK(fabsf(rx.message.status.yaw_joint + MOTION_PI) < 1e-6f);
    CHECK(fabsf(rx.message.status.pitch_joint - MOTION_PI/2) < 1e-6f);
    CHECK(rx.message.status.calibrated && rx.message.status.motors_healthy && !rx.message.status.fault);
    CHECK(!Link_GetCommand(&rx, 5, &out));
    status.yaw_joint = INFINITY;
    CHECK(!Link_EncodeStatus(&status, 18, frames));
    return 0;
}
