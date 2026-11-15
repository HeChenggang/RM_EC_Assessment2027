#include "board_link.h"
#include "motion_math.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static uint16_t crc_byte(uint16_t crc, uint8_t byte)
{
    crc ^= (uint16_t)byte << 8;
    for (unsigned i = 0; i < 8; ++i) crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000U) ? 0x1021U : 0));
    return crc;
}
static uint16_t checksum(LinkKind kind, uint8_t sequence, const uint8_t data[18])
{
    uint16_t crc = crc_byte(crc_byte(crc_byte(0xffff, 1), (uint8_t)kind), sequence);
    for (unsigned i = 0; i < 16; ++i) crc = crc_byte(crc, data[i]);
    return crc;
}
static void put16(uint8_t *p, uint16_t n) { p[0] = (uint8_t)n; p[1] = (uint8_t)(n >> 8); }
static int32_t signed16(const uint8_t *p)
{
    const uint32_t n = p[0] | ((uint32_t)p[1] << 8);
    return n & 0x8000U ? (int32_t)n - 65536 : (int32_t)n;
}
static void put32(uint8_t *p, int32_t n)
{
    const uint32_t u = (uint32_t)n;
    for (unsigned i = 0; i < 4; ++i) p[i] = (uint8_t)(u >> (8*i));
}
static int32_t signed32(const uint8_t *p)
{
    const uint32_t n = p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24);
    return n <= 0x7fffffffU ? (int32_t)n : -1 - (int32_t)(~n);
}
static void frames(LinkKind kind, uint8_t seq, uint8_t data[18], LinkFrame out[3])
{
    put16(data+16, checksum(kind, seq, data));
    for (unsigned i = 0; i < 3; ++i) {
        out[i] = (LinkFrame){.id = (uint16_t)(0x300 + 0x10*kind + i), .dlc = 8};
        out[i].data[0] = 1; out[i].data[1] = seq;
        memcpy(out[i].data+2, data+6*i, 6);
    }
}
bool Link_EncodeCommand(const RobotCommand *c, uint8_t seq, LinkFrame out[3])
{
    if (out == NULL) return false;
    memset(out, 0, 3*sizeof(*out));
    if (!Command_Valid(c)) return false;
    uint8_t data[18] = {0};
    data[0] = (uint8_t)(c->remote_valid | (c->arm<<1) | (c->estop<<2));
    data[1] = (uint8_t)c->mode;
    const float axes[4] = {c->forward, c->left, c->yaw, c->pitch};
    for (unsigned i = 0; i < 4; ++i) put16(data+2+2*i, (uint16_t)(int16_t)roundf(axes[i]*10000));
    frames(LINK_COMMAND, seq, data, out);
    return true;
}
static bool status_valid(const LinkStatus *s)
{
    return s != NULL && isfinite(s->yaw_joint) && fabsf(s->yaw_joint) <= MOTION_PI &&
        isfinite(s->pitch_joint) && fabsf(s->pitch_joint) <= MOTION_PI/2;
}
bool Link_EncodeStatus(const LinkStatus *s, uint8_t seq, LinkFrame out[3])
{
    if (out == NULL) return false;
    memset(out, 0, 3*sizeof(*out));
    if (!status_valid(s)) return false;
    uint8_t data[18] = {0};
    data[0] = (uint8_t)(s->calibrated | (s->motors_healthy<<1) | (s->sensors_fresh<<2) | (s->fault<<3));
    put32(data+2, (int32_t)roundf(s->yaw_joint*1000000));
    put32(data+6, (int32_t)roundf(s->pitch_joint*1000000));
    frames(LINK_STATUS, seq, data, out);
    return true;
}
bool Link_Init(LinkReceiver *r, LinkKind kind, uint32_t assembly, uint32_t fresh)
{
    if (r == NULL) return false;
    *r = (LinkReceiver){0};
    if ((kind != LINK_COMMAND && kind != LINK_STATUS) || assembly == 0 || fresh <= assembly || fresh > 0x7fffffffU)
        return false;
    *r = (LinkReceiver){.kind = kind, .assembly_timeout_ms = assembly, .freshness_ms = fresh, .configured = true};
    return true;
}
bool Link_IsFresh(const LinkReceiver *r, uint32_t now)
{
    return r != NULL && r->configured && r->received && (uint32_t)(now-r->last_rx_ms) < r->freshness_ms;
}
static bool newer(uint8_t n, uint8_t previous)
{
    const uint8_t delta = (uint8_t)(n-previous);
    return delta > 0 && delta < 128;
}
static bool decode(LinkKind kind, const uint8_t data[18], LinkMessage *out)
{
    for (unsigned i = 10; i < 16; ++i) if (data[i] != 0) return false;
    if (kind == LINK_COMMAND) {
        if ((data[0] & ~7U) != 0) return false;
        out->command = (RobotCommand){.mode = (DriveMode)data[1], .remote_valid = (data[0]&1)!=0,
            .arm = (data[0]&2)!=0, .estop = (data[0]&4)!=0,
            .forward = signed16(data+2)/10000.0f, .left = signed16(data+4)/10000.0f,
            .yaw = signed16(data+6)/10000.0f, .pitch = signed16(data+8)/10000.0f};
        return Command_Valid(&out->command);
    }
    if ((data[0] & ~15U) != 0 || data[1] != 0) return false;
    out->status = (LinkStatus){.yaw_joint = signed32(data+2)/1000000.0f,
        .pitch_joint = signed32(data+6)/1000000.0f, .calibrated = (data[0]&1)!=0,
        .motors_healthy = (data[0]&2)!=0, .sensors_fresh = (data[0]&4)!=0, .fault = (data[0]&8)!=0};
    /* Allow half a quantization unit at the closed endpoints. */
    return isfinite(out->status.yaw_joint) && fabsf(out->status.yaw_joint) <= MOTION_PI+1e-6f &&
        isfinite(out->status.pitch_joint) && fabsf(out->status.pitch_joint) <= MOTION_PI/2+1e-6f;
}
bool Link_Receive(LinkReceiver *r, const LinkFrame *f, uint32_t now)
{
    if (r == NULL || !r->configured || f == NULL) return false;
    const uint16_t base = (uint16_t)(0x300 + 0x10*r->kind);
    if (f->extended || f->remote || f->dlc != 8 || f->id < base || f->id > base+2 || f->data[0] != 1)
        return false;
    const uint8_t seq = f->data[1], index = (uint8_t)(f->id-base);
    const bool synced = Link_IsFresh(r, now);
    if (synced && !newer(seq, r->last_seq)) return false;
    if (r->mask && (uint32_t)(now-r->assembly_started_ms) >= r->assembly_timeout_ms) r->mask = 0;
    if (r->mask && seq != r->pending_seq) {
        if (!newer(seq, r->pending_seq)) return false;
        r->mask = 0;
    }
    if (!r->mask) { r->pending_seq = seq; r->assembly_started_ms = now; }
    const uint8_t bit = (uint8_t)(1U<<index);
    if (r->mask & bit) return false;
    memcpy(r->parts+6*index, f->data+2, 6);
    r->mask |= bit;
    if (r->mask != 7) return false;
    r->mask = 0;
    const uint16_t crc = (uint16_t)(r->parts[16] | ((uint16_t)r->parts[17]<<8));
    LinkMessage message = {0};
    if (crc != checksum(r->kind, seq, r->parts) || !decode(r->kind, r->parts, &message) ||
        (!synced && r->kind == LINK_COMMAND && message.command.arm)) return false;
    r->message = message; r->received = true; r->last_seq = seq; r->last_rx_ms = now;
    return true;
}
bool Link_GetCommand(const LinkReceiver *r, uint32_t now, RobotCommand *out)
{
    if (out == NULL) return false;
    *out = (RobotCommand){.estop = true};
    if (!Link_IsFresh(r, now) || r->kind != LINK_COMMAND) return false;
    *out = r->message.command;
    return true;
}
