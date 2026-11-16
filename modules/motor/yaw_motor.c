#include "yaw_motor.h"
#include "gimbal_board.h"
#include <math.h>
#include <stddef.h>
static CAN_HandleTypeDef *bus;
static YawMotor_Ranges limits;
static bool configured;
static volatile YawMotor_Feedback feedback;
static bool positive(float x) { return isfinite(x) && x > 0 && x < 1e6f; }
HAL_StatusTypeDef YawMotor_Init(CAN_HandleTypeDef *can, const YawMotor_Ranges *ranges)
{
    if (bus || !can || can->Instance != CAN1) return HAL_ERROR;
    if (ranges && (!positive(ranges->pmax) || !positive(ranges->vmax) || !positive(ranges->tmax))) return HAL_ERROR;
    if (ranges) limits = *ranges;
    configured = ranges != NULL;
    bus = can;
    feedback = (YawMotor_Feedback){0};
    return HAL_OK;
}
static float decode(unsigned value, unsigned maximum, float range)
{ return ((float)value / maximum * 2.0f - 1.0f) * range; }
bool YawMotor_Receive(const CAN_RxHeaderTypeDef *h, const uint8_t d[8])
{
    if (!bus || !h || !d || h->IDE != CAN_ID_STD || h->RTR != CAN_RTR_DATA ||
        h->DLC != 8 || h->StdId != GIMBAL_YAW_RX_ID || (d[0] & 15) != GIMBAL_YAW_TX_ID) return false;
    YawMotor_Feedback f = {0};
    f.position_raw = (d[1] << 8) | d[2];
    f.velocity_raw = (d[3] << 4) | (d[4] >> 4);
    f.torque_raw = ((d[4] & 15) << 8) | d[5];
    f.state = d[0] >> 4; f.mos_temperature = d[6]; f.coil_temperature = d[7];
    f.received = true; f.scaled = configured;
    if (configured) {
        f.pos = decode(f.position_raw, 65535, limits.pmax);
        f.vel = decode(f.velocity_raw, 4095, limits.vmax);
        f.torque = decode(f.torque_raw, 4095, limits.tmax);
    }
    f.last_rx_ms = HAL_GetTick(); f.rx_count = feedback.rx_count + 1;
    feedback = f;
    return true;
}
bool YawMotor_GetFeedback(YawMotor_Feedback *out)
{
    if (!out) return false;
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    *out = feedback;
    __set_PRIMASK(mask);
    return out->received;
}
bool YawMotor_IsOnline(uint32_t timeout_ms)
{
    YawMotor_Feedback f;
    return YawMotor_GetFeedback(&f) && (uint32_t)(HAL_GetTick() - f.last_rx_ms) < timeout_ms;
}
static HAL_StatusTypeDef send_id(uint16_t id, const uint8_t data[8])
{
    if (!bus || HAL_CAN_GetState(bus) != HAL_CAN_STATE_LISTENING) return HAL_ERROR;
    if (HAL_CAN_GetTxMailboxesFreeLevel(bus) != 3) return HAL_BUSY;
    CAN_TxHeaderTypeDef h = {.StdId = id, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 8};
    uint32_t mailbox;
    return HAL_CAN_AddTxMessage(bus, &h, data, &mailbox);
}
static HAL_StatusTypeDef send(const uint8_t data[8])
{ return send_id(GIMBAL_YAW_TX_ID, data); }
HAL_StatusTypeDef YawMotor_RequestFeedback(void)
{
    const uint8_t data[8] = {GIMBAL_YAW_TX_ID,0,0xCC,0,0,0,0,0};
    return send_id(0x7FF, data);
}
HAL_StatusTypeDef YawMotor_Enable(void)
{
    const uint8_t d[8] = {255,255,255,255,255,255,255,252};
    return configured ? send(d) : HAL_ERROR;
}
HAL_StatusTypeDef YawMotor_Disable(void)
{
    const uint8_t d[8] = {255,255,255,255,255,255,255,253};
    return send(d);
}
static bool within(float x, float lo, float hi) { return isfinite(x) && x >= lo && x <= hi; }
static unsigned encode(float x, float range, unsigned max)
{ return (unsigned)((x / range + 1.0f) * 0.5f * max); }
HAL_StatusTypeDef YawMotor_SendMIT(float p, float v, float kp, float kd, float t)
{
    if (!configured || !within(p,-limits.pmax,limits.pmax) || !within(v,-limits.vmax,limits.vmax) ||
        !within(t,-limits.tmax,limits.tmax) || !within(kp,0,500) || !within(kd,0,5)) return HAL_ERROR;
    unsigned pi=encode(p,limits.pmax,65535), vi=encode(v,limits.vmax,4095), ti=encode(t,limits.tmax,4095);
    unsigned kpi=(unsigned)(kp/500*4095), kdi=(unsigned)(kd/5*4095);
    uint8_t d[8] = {pi>>8,pi,vi>>4,((vi&15)<<4)|(kpi>>8),kpi,kdi>>4,((kdi&15)<<4)|(ti>>8),ti};
    bool special=true;
    for (unsigned i=0;i<7;++i) special = special && d[i]==255;
    if (special && d[7]>=252) return HAL_ERROR;
    return send(d);
}

/* Register-query framing follows cmjang/DM_Control_Python DM_CAN.py.
 * Parameter packets must be dispatched before normal motor feedback. */
#include <string.h>
static volatile YawMotor_Params params;
static const uint8_t param_rid[4] = {10, 21, 22, 23};
static volatile uint8_t param_index, param_attempts;
static volatile bool param_pending;
static uint32_t param_sent_ms;
void YawMotor_GetParams(YawMotor_Params *out)
{
    if (!out) return;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    *out=params; __set_PRIMASK(mask);
}
bool YawMotor_ReceiveParam(const CAN_RxHeaderTypeDef *h, const uint8_t d[8])
{
    if (!h || !d || h->IDE!=CAN_ID_STD || h->RTR!=CAN_RTR_DATA ||
        h->DLC!=8 || h->StdId!=GIMBAL_YAW_RX_ID ||
        d[0]!=GIMBAL_YAW_TX_ID || d[1]!=0 || d[2]!=0x33) return false;
    if (!param_pending || param_index>=4 || d[3]!=param_rid[param_index]) return true;
    if (param_index==0) {
        uint32_t mode; memcpy(&mode,d+4,4);
        if (mode<1 || mode>4) return true;
        params.control_mode=mode;
    } else {
        float value; memcpy(&value,d+4,4);
        if (!positive(value)) return true;
        if (param_index==1) params.pmax=value;
        if (param_index==2) params.vmax=value;
        if (param_index==3) params.tmax=value;
    }
    params.valid_mask |= (uint8_t)(1U<<param_index);
    param_pending=false; param_attempts=0; ++param_index;
    params.finished=param_index==4;
    return true;
}
void YawMotor_ParamService(bool disarmed)
{
    if (!bus || !disarmed) return;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    uint32_t now=HAL_GetTick();
    if (params.finished) goto done;
    if (param_pending) {
        if ((uint32_t)(now-param_sent_ms)<100U) goto done;
        param_pending=false; ++params.timeout_count;
    }
    if (param_attempts>=3) {
        param_attempts=0; ++param_index;
        if (param_index==4) { params.finished=true; goto done; }
    }
    if (HAL_CAN_GetState(bus)!=HAL_CAN_STATE_LISTENING ||
        HAL_CAN_GetTxMailboxesFreeLevel(bus)!=3) goto done;
    CAN_TxHeaderTypeDef h={.StdId=0x7ff,.IDE=CAN_ID_STD,.RTR=CAN_RTR_DATA,.DLC=8};
    uint8_t d[8]={GIMBAL_YAW_TX_ID,0,0x33,param_rid[param_index],0,0,0,0};
    uint32_t mailbox;
    HAL_StatusTypeDef result=HAL_CAN_AddTxMessage(bus,&h,d,&mailbox);
    ++param_attempts; param_sent_ms=now; param_pending=true;
    if (result!=HAL_OK) ++params.tx_error_count;
done:
    __set_PRIMASK(mask);
}
