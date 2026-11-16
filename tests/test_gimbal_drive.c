#include "gimbal_drive.h"
#include "gimbal_setup.h"
#include <math.h>
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)

static CAN_HandleTypeDef buses[2];
static unsigned free_tx[2]={3,3}, aborts, enables[2], disables[2], controls[2];
static float torque[2];
static HAL_StatusTypeDef enable_result[2], control_result[2];
static HAL_StatusTypeDef disable_result[2];
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *h)
{ return free_tx[h == &buses[1]]; }
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *h, uint32_t mailboxes)
{ (void)mailboxes; ++aborts; free_tx[h == &buses[1]]=3; return HAL_OK; }
HAL_StatusTypeDef YawMotor_Enable(void) { ++enables[0]; return enable_result[0]; }
HAL_StatusTypeDef PitchMotor_Enable(void) { ++enables[1]; return enable_result[1]; }
HAL_StatusTypeDef YawMotor_Disable(void) { ++disables[0]; torque[0]=0; return disable_result[0]; }
HAL_StatusTypeDef PitchMotor_Disable(void) { ++disables[1]; torque[1]=0; return disable_result[1]; }
static HAL_StatusTypeDef control(unsigned i, float p,float v,float kp,float kd,float t)
{
    if (p || v || kp || kd) return HAL_ERROR;
    ++controls[i]; torque[i]=t; return control_result[i];
}
HAL_StatusTypeDef YawMotor_SendMIT(float p,float v,float kp,float kd,float t)
{ return control(0,p,v,kp,kd,t); }
HAL_StatusTypeDef PitchMotor_SendMIT(float p,float v,float kp,float kd,float t)
{ return control(1,p,v,kp,kd,t); }

static GimbalDrive s;
static RobotCommand cmd;
static YawMotor_Feedback y;
static PitchMotor_Feedback p;
static uint32_t now;
static GimbalDriveConfig config(void)
{
    GimbalDriveConfig c=GimbalSetup_Config();
    /* Synthetic calibrated test rig, not defaults for the physical robot. */
    c.hardware_confirmed=true;
    c.pitch_only=false;
    c.control.pitch_min=-0.4f; c.control.pitch_max=0.4f;
    c.pitch_zero=0.5f;
    c.pitch_direction=1;
    return c;
}
static void feedback(unsigned state)
{
    y=(YawMotor_Feedback){.received=true,.scaled=true,.state=state,
        .last_rx_ms=now,.rx_count=y.rx_count+1};
    p=(PitchMotor_Feedback){.received=true,.last_rx_ms=now,.rx_count=p.rx_count+1,
        .motor={.pos=0.5f,.state=state}};
}
static void step(unsigned ms, bool link)
{ now+=ms; GimbalDrive_Step(&s,&cmd,link,&y,&p,now); }
static void init(void)
{
    GimbalDriveConfig c=config();
    now=1000;
    (void)GimbalDrive_Init(&s,&c,&buses[0],&buses[1],now);
    cmd=(RobotCommand){.remote_valid=true};
    y=(YawMotor_Feedback){0}; p=(PitchMotor_Feedback){0};
    free_tx[0]=free_tx[1]=3;
    enable_result[0]=enable_result[1]=control_result[0]=control_result[1]=HAL_OK;
    disable_result[0]=disable_result[1]=HAL_OK;
}
static void arm(void)
{
    cmd.arm=false; step(20,true);
    cmd.arm=true; step(5,true); /* preload */
    step(5,true); /* enable */
    feedback(1); step(5,true); /* acknowledge and track */
}
int main(void)
{
    GimbalDriveConfig c=GimbalSetup_Config();
    c.hardware_confirmed=false;
    CHECK(!GimbalDrive_Init(&s,&c,&buses[0],&buses[1],0));
    cmd=(RobotCommand){.arm=true,.remote_valid=true};
    step(20,true); CHECK(s.fault==GIMBAL_CONFIG && enables[0]==0 && controls[0]==0);
    init(); cmd.arm=true; step(5,true);
    CHECK(s.state==GIMBAL_STOPPED && enables[0]==0); /* held arm at boot */
    cmd.arm=false; cmd.yaw=0.5f; step(20,true);
    cmd.arm=true; cmd.yaw=0; step(5,true);
    CHECK(s.state==GIMBAL_STOPPED); /* disarm must also be neutral */
    arm(); CHECK(s.state==GIMBAL_ACTIVE && s.fault==GIMBAL_OK);
    CHECK(fabsf(s.output.target[1])<0.0001f); /* calibrated offset */
    cmd.yaw=1; cmd.pitch=1; feedback(1); step(5,true);
    CHECK(torque[0]>0 && torque[1]>0 && torque[0]<=0.3f && torque[1]<=1);
    free_tx[0]=free_tx[1]=0;
    unsigned old_aborts=aborts, old_disable=disables[0];
    cmd.estop=true; step(1,true);
    CHECK(s.state==GIMBAL_STOPPED && torque[0]==0 && torque[1]==0);
    CHECK(aborts==old_aborts+2 && disables[0]>old_disable);
    old_disable=disables[0]; step(20,true); CHECK(disables[0]>old_disable);
    cmd.estop=false; cmd.yaw=cmd.pitch=0; step(5,true);
    CHECK(s.state==GIMBAL_STOPPED); /* cannot auto restart */
    arm(); CHECK(s.state==GIMBAL_ACTIVE);
    step(5,false); CHECK(s.state==GIMBAL_STOPPED && s.fault==GIMBAL_LINK);
    step(5,true); CHECK(s.state==GIMBAL_STOPPED);
    arm(); CHECK(s.state==GIMBAL_ACTIVE);
    y.last_rx_ms=now-100; step(5,true);
    CHECK(s.state==GIMBAL_STOPPED && s.fault==GIMBAL_FEEDBACK);
    arm(); feedback(1); p.motor.state=8; step(5,true);
    CHECK(s.state==GIMBAL_STOPPED && s.fault==GIMBAL_FEEDBACK);
    arm(); feedback(1); p.motor.pos=1.0f; step(5,true);
    CHECK(s.state==GIMBAL_STOPPED && s.fault==GIMBAL_LIMIT);
    arm(); control_result[1]=HAL_BUSY; feedback(1); step(5,true);
    CHECK(s.state==GIMBAL_STOPPED && s.fault==GIMBAL_TX && torque[0]==0);
    init(); enable_result[1]=HAL_ERROR;
    cmd.arm=false; step(20,true); cmd.arm=true; step(5,true); step(5,true);
    CHECK(s.state==GIMBAL_STOPPED && s.fault==GIMBAL_TX);
    init(); cmd.arm=false; step(20,true); cmd.arm=true; step(5,true); step(5,true);
    CHECK(s.state==GIMBAL_ENABLING);
    for (unsigned i=0;i<61;i++) step(5,true);
    CHECK(s.state==GIMBAL_STOPPED && s.fault==GIMBAL_ENABLE_TIMEOUT);
    CHECK(torque[0]==0 && torque[1]==0);
    init(); feedback(1); /* old enabled feedback cannot acknowledge new enable */
    cmd.arm=false; step(20,true); cmd.arm=true; step(5,true); step(5,true); step(5,true);
    CHECK(s.state==GIMBAL_ENABLING && torque[0]==0 && torque[1]==0);
    ++y.rx_count; y.last_rx_ms=now; step(5,true);
    CHECK(s.state==GIMBAL_ENABLING); /* both motors must acknowledge */
    feedback(1); step(5,true); CHECK(s.state==GIMBAL_ACTIVE);
    disable_result[0]=HAL_BUSY; cmd.estop=true; step(1,true);
    CHECK(s.state==GIMBAL_STOPPED && !s.stop_sent);
    old_disable=disables[0]; disable_result[0]=HAL_OK; step(20,true);
    CHECK(s.stop_sent && disables[0]>old_disable);
    /* One pending Pitch frame must not consume a valid arm request. */
    init(); step(20,true);
    free_tx[1]=2; cmd.arm=true;
    unsigned old_control=controls[1], old_enable=enables[1];
    step(1,true); step(10,true);
    CHECK(s.state==GIMBAL_WAIT_TX && controls[1]==old_control && enables[1]==old_enable);
    free_tx[1]=3; step(1,true);
    CHECK(s.state==GIMBAL_PREPARING && controls[1]==old_control+1 && torque[1]==0);
    step(5,true); feedback(1); step(5,true);
    CHECK(s.state==GIMBAL_ACTIVE && enables[1]==old_enable+1);
    cmd.yaw=1; feedback(1); step(5,true);
    CHECK(torque[0]>0);
    /* Busy timeout, emergency stop, link loss and stick movement cancel the pending request. */
    for (unsigned reason=0; reason<4; ++reason) {
        init(); step(20,true);
        free_tx[1]=2; cmd.arm=true; step(1,true);
        CHECK(s.state==GIMBAL_WAIT_TX);
        old_enable=enables[1];
        cmd.estop=reason==1;
        cmd.yaw=reason==3 ? 1 : 0;
        step(reason==0 ? 20 : 1,reason!=2);
        CHECK(s.state==GIMBAL_STOPPED && s.fault==(reason==0 ? GIMBAL_TX : GIMBAL_LINK));
        CHECK(s.last_fault==s.fault && torque[0]==0 && torque[1]==0);
        free_tx[1]=3; cmd.estop=false; cmd.yaw=0; step(5,true);
        CHECK(s.state==GIMBAL_STOPPED && enables[1]==old_enable);
    }
    init(); arm(); step(21,true);
    CHECK(s.state==GIMBAL_STOPPED && s.fault==GIMBAL_TIMING);
    init(); now=0xfffffff0U; s.last_step=now; s.last_stop=now-20;
    arm(); CHECK(s.state==GIMBAL_ACTIVE); /* tick wrap */
    init(); c=config(); c.pitch_direction=-1;
    CHECK(GimbalDrive_Init(&s,&c,&buses[0],&buses[1],now));
    arm(); cmd.pitch=1; feedback(1); step(5,true); CHECK(torque[1]<0);
    /* Actual measured pitch calibration: zero and both endpoint directions. */
    c=GimbalSetup_Config();
    CHECK(fabsf(c.control.pitch_min-(-0.244f))<0.00001f);
    CHECK(fabsf(c.control.pitch_max-0.959f)<0.00001f);
    GimbalControl calibrated;
    CHECK(Gimbal_Init(&calibrated,&c.control));
    GimbalFeedback f={.joint_fresh=true};
    GimbalOutput out;
    cmd=(RobotCommand){.remote_valid=true,.arm=true,.pitch=1};
    f.joint[1]=(GIMBAL_PITCH_ZERO-c.pitch_zero)*c.pitch_direction;
    CHECK(Gimbal_Step(&calibrated,&cmd,&f,true,0.005f,&out));
    CHECK(out.torque[1]*c.pitch_direction<0); /* up requests decreasing raw angle */
    Gimbal_Reset(&calibrated);
    f.joint[1]=(GIMBAL_PITCH_UPPER_RAW-c.pitch_zero)*c.pitch_direction;
    CHECK(Gimbal_Step(&calibrated,&cmd,&f,true,0.005f,&out));
    CHECK(out.torque[1]==0 && out.target[1]<=c.control.pitch_max);
    Gimbal_Reset(&calibrated); cmd.pitch=-1;
    f.joint[1]=(GIMBAL_PITCH_LOWER_RAW-c.pitch_zero)*c.pitch_direction;
    CHECK(Gimbal_Step(&calibrated,&cmd,&f,true,0.005f,&out));
    CHECK(out.torque[1]==0 && out.target[1]>=c.control.pitch_min);
    f.joint[1]-=0.001f;
    CHECK(!Gimbal_Step(&calibrated,&cmd,&f,true,0.005f,&out));
    init(); c=GimbalSetup_Config();
    CHECK(!c.pitch_only && c.hardware_confirmed);
    CHECK(c.yaw_direction==1 && c.control.rate_pid[0].output_limit==0.3f);
    CHECK(c.control.rate_pid[1].output_limit==DM4340_T_MAX);
    CHECK(fabsf(c.control.rate_pid[1].integral_limit-0.9f*DM4340_T_MAX)<0.01f);
    /* Preserve coverage of the optional Pitch-only commissioning mode. */
    c.pitch_only=true;
    /* Keep the legacy 2 N m wall in this scenario: the stall-window and
     * holding-torque timings below are calibrated around it. The production
     * cap is covered by the dedicated case at the end of main. */
    c.control.rate_pid[1].output_limit=2.0f;
    c.control.rate_pid[1].integral_limit=1.8f;
    CHECK(GimbalDrive_Init(&s,&c,&buses[0],&buses[1],now));
    unsigned yaw_enable_before=enables[0], yaw_control_before=controls[0];
    cmd.arm=false; step(20,true); cmd.arm=true; step(5,true); step(5,true);
    y=(YawMotor_Feedback){0};
    p=(PitchMotor_Feedback){.received=true,.last_rx_ms=now,.rx_count=1,
        .motor={.pos=GIMBAL_PITCH_ZERO,.state=1}};
    step(5,true); CHECK(s.state==GIMBAL_ACTIVE);
    cmd.pitch=1; cmd.yaw=1; p.last_rx_ms=now; ++p.rx_count; step(5,true);
    CHECK(torque[1]<0 && s.output.torque[0]==0);
    CHECK(enables[0]==yaw_enable_before && controls[0]==yaw_control_before);
    /* Integral can reach the legacy 2 N m cap before the bounded stall window expires. */
    for (unsigned i=0;i<1000 && torque[1]>-1.95f;++i) {
        p.motor.pos=GIMBAL_PITCH_ZERO; p.motor.vel=0;
        p.last_rx_ms=now; ++p.rx_count; step(5,true);
    }
    CHECK(s.state==GIMBAL_ACTIVE && torque[1]<=-1.95f && torque[1]>=-2.0f);
    /* Progress under high load refreshes the window instead of false-tripping. */
    for (unsigned i=0;i<240;++i) {
        p.motor.pos-=0.001f; p.last_rx_ms=now; ++p.rx_count; step(5,true);
        CHECK(s.state==GIMBAL_ACTIVE && fabsf(torque[1])<=2.0f);
    }
    /* Freeze a loaded axis, including a millisecond-counter wrap. */
    now=0xffffff00U; s.last_step=now; s.pitch_stall_tracking=false;
    for (unsigned i=0;i<220 && s.state==GIMBAL_ACTIVE;++i) {
        p.last_rx_ms=now; ++p.rx_count; step(5,true);
    }
    CHECK(s.state==GIMBAL_STOPPED && s.fault==GIMBAL_STALL);
    CHECK(torque[1]==0 && s.control.rate[1].i==0);
    step(5,true); CHECK(s.state==GIMBAL_STOPPED); /* held arm never resumes */
    cmd=(RobotCommand){.remote_valid=true}; step(20,true);
    CHECK(s.last_fault==GIMBAL_STALL); /* survives return to disarm */
    cmd.arm=true; step(5,true); step(5,true);
    p.motor.pos=GIMBAL_PITCH_ZERO; p.last_rx_ms=now; ++p.rx_count; step(5,true);
    CHECK(s.state==GIMBAL_ACTIVE && s.last_fault==GIMBAL_OK);
    /* Gravity holding torque with negligible position error is not a stall. */
    s.control.rate[1].i=1.8f;
    for (unsigned i=0;i<220;++i) {
        p.last_rx_ms=now; ++p.rx_count; step(5,true);
    }
    CHECK(s.state==GIMBAL_ACTIVE && !s.pitch_stall_tracking);
    p.motor.pos=0.059107065f;
    p.last_rx_ms=now; ++p.rx_count; step(5,true);
    CHECK(s.state==GIMBAL_ACTIVE && s.fault==GIMBAL_OK);
    p.motor.pos=GIMBAL_PITCH_LOWER_RAW+0.00001f;
    p.last_rx_ms=now; ++p.rx_count; cmd.pitch=-1; step(5,true);
    CHECK(s.state==GIMBAL_ACTIVE && torque[1]<=0); /* no outward/down torque */
    p.motor.pos=GIMBAL_PITCH_LOWER_RAW+0.002f;
    p.last_rx_ms=now; ++p.rx_count; step(5,true);
    CHECK(s.state==GIMBAL_STOPPED && s.fault==GIMBAL_LIMIT);
    cmd.pitch=0; cmd.yaw=0; cmd.arm=false; step(20,true);
    cmd.arm=true; step(5,true); step(5,true);
    p.motor.pos=GIMBAL_PITCH_ZERO; p.last_rx_ms=now; ++p.rx_count; step(5,true);
    CHECK(s.state==GIMBAL_ACTIVE);
    p.last_rx_ms=now-100; step(5,true);
    CHECK(s.state==GIMBAL_STOPPED && s.fault==GIMBAL_FEEDBACK);
    /* P0-2 skeleton: a WORLD yaw axis tracks the attitude sample, not the joint
     * angle, and disarms as soon as that sample goes stale. */
    init(); c=config(); c.control.space[0]=GIMBAL_WORLD;
    CHECK(GimbalDrive_Init(&s,&c,&buses[0],&buses[1],now));
    cmd.arm=false; step(20,true);
    GimbalDrive_SetWorld(&s,0.4f,0.0f,now);
    cmd.arm=true; step(5,true); step(5,true);
    y=(YawMotor_Feedback){.received=true,.scaled=true,.state=1,
        .last_rx_ms=now,.rx_count=1,.pos=0.2f};
    p=(PitchMotor_Feedback){.received=true,.last_rx_ms=now,.rx_count=1,
        .motor={.pos=0.5f,.state=1}};
    GimbalDrive_SetWorld(&s,0.4f,0.0f,now);
    step(5,true);
    CHECK(s.state==GIMBAL_ACTIVE && s.fault==GIMBAL_OK);
    cmd.yaw=1; GimbalDrive_SetWorld(&s,0.4f,0.0f,now);
    feedback(1); GimbalDrive_SetWorld(&s,0.4f,0.0f,now); step(5,true);
    CHECK(torque[0]>0); /* joystick moves the world heading, joint stays free */
    step(20,true); CHECK(s.state==GIMBAL_ACTIVE);
    step(20,true); /* world sample now older than WORLD_MS */
    CHECK(s.state==GIMBAL_STOPPED && s.fault==GIMBAL_FEEDBACK);
    c.pitch_stall_ms=0;
    CHECK(!GimbalDrive_Init(&s,&c,&buses[0],&buses[1],now));
    /* Production config: the removed Pitch torque cap lets the rate loop
     * command well beyond the old 2 N m commissioning limit. A heavy axis is
     * simulated by freezing the joint; sustained command builds integral past
     * -2 N m while the commanded torque stays inside the DM4340 MIT range. */
    init(); c=GimbalSetup_Config();
    CHECK(GimbalDrive_Init(&s,&c,&buses[0],&buses[1],now));
    cmd=(RobotCommand){.remote_valid=true};
    cmd.arm=false; step(20,true);
    cmd.arm=true; step(5,true); step(5,true); /* preload + enable */
    y=(YawMotor_Feedback){.received=true,.scaled=true,.state=1,
        .last_rx_ms=now,.rx_count=1};
    p=(PitchMotor_Feedback){.received=true,.last_rx_ms=now,.rx_count=1,
        .motor={.pos=GIMBAL_PITCH_ZERO,.state=1}};
    step(5,true); CHECK(s.state==GIMBAL_ACTIVE);
    cmd.pitch=1;
    float peak=0;
    for (unsigned i=0;i<1200;++i) {
        y.last_rx_ms=now; ++y.rx_count;
        p.last_rx_ms=now; ++p.rx_count;
        step(5,true);
        CHECK(s.state==GIMBAL_ACTIVE);
        if (torque[1]<peak) peak=torque[1];
    }
    CHECK(peak<-2.0f && peak>=-DM4340_T_MAX);
    c=GimbalSetup_Config(); c.pitch_stall_travel=NAN;
    CHECK(!GimbalDrive_Init(&s,&c,&buses[0],&buses[1],now));
    /* Neutral stick preserves the target under either external displacement.
     * Real negative Pitch direction must produce restoring raw torque. */
    for (int direction=-1; direction<=1; direction+=2) {
        init(); c=GimbalSetup_Config();
        CHECK(GimbalDrive_Init(&s,&c,&buses[0],&buses[1],now));
        cmd.arm=false; step(20,true);
        cmd.arm=true; step(5,true); step(5,true);
        feedback(1); p.motor.pos=GIMBAL_PITCH_ZERO; step(5,true);
        CHECK(s.state==GIMBAL_ACTIVE);
        const float target=s.output.target[1];
        p.motor.pos=GIMBAL_PITCH_ZERO+direction*0.1f;
        p.last_rx_ms=now; y.last_rx_ms=now; step(5,true);
        CHECK(s.state==GIMBAL_ACTIVE && fabsf(s.output.target[1]-target)<1e-6f);
        CHECK(direction*torque[1]<-2.0f); /* immediate restoring torque */
        float first=fabsf(torque[1]);
        for (unsigned i=0;i<100;++i) {
            p.last_rx_ms=now; y.last_rx_ms=now; step(5,true);
            CHECK(s.state==GIMBAL_ACTIVE && fabsf(s.output.target[1]-target)<1e-6f);
        }
        CHECK(fabsf(torque[1])>first); /* sustained load builds compensation */
        p.motor.pos=GIMBAL_PITCH_ZERO;
        p.last_rx_ms=now; y.last_rx_ms=now; step(5,true);
        CHECK(direction*torque[1]<0); /* retain learned holding load at target */
        cmd.estop=true; step(1,true);
        CHECK(torque[1]==0 && s.control.rate[1].i==0);
    }
    c=config(); c.control.pitch_hold_kp=-1;
    CHECK(!GimbalDrive_Init(&s,&c,&buses[0],&buses[1],now));
    c.control.pitch_hold_kp=NAN;
    CHECK(!GimbalDrive_Init(&s,&c,&buses[0],&buses[1],now));
    /* Follow mode is unavailable without confirmed installation. */
    init(); arm(); cmd.mode=MODE_FOLLOW; feedback(1);
    GimbalDrive_SetWorld(&s,1.0f,0,now);
    step(5,true);
    CHECK(s.state==GIMBAL_STOPPED && s.fault==GIMBAL_CONFIG);
    init(); c=config(); c.follow_hardware_confirmed=true;
    CHECK(GimbalDrive_Init(&s,&c,&buses[0],&buses[1],now));
    arm(); cmd.mode=MODE_FOLLOW; feedback(1);
    GimbalDrive_SetWorld(&s,1.0f,0,now);
    step(5,true);
    CHECK(s.state==GIMBAL_ACTIVE && fabsf(s.output.target[0]-1.0f)<0.001f);
    feedback(1); step(15,true);
    feedback(1); step(10,true);
    CHECK(s.state==GIMBAL_STOPPED && s.fault==GIMBAL_FEEDBACK);
    GimbalDrive_SetWorld(&s,1.0f,0,now); feedback(1); step(5,true);
    CHECK(s.state==GIMBAL_STOPPED); /* held ARM cannot recover */
    /* Conventional spin is unavailable without confirmed installation. */
    init(); arm(); cmd.mode=MODE_SPIN; feedback(1);
    GimbalDrive_SetWorld(&s,1.0f,0,now);
    step(5,true);
    CHECK(s.state==GIMBAL_STOPPED && s.fault==GIMBAL_CONFIG);
    init(); c=config(); c.spin_hardware_confirmed=true;
    CHECK(GimbalDrive_Init(&s,&c,&buses[0],&buses[1],now));
    arm(); cmd.mode=MODE_SPIN; feedback(1);
    GimbalDrive_SetWorld(&s,1.0f,0,now);
    step(5,true);
    CHECK(s.state==GIMBAL_ACTIVE && fabsf(s.output.target[0]-1.0f)<0.001f);
    feedback(1); step(15,true);
    feedback(1); step(10,true);
    CHECK(s.state==GIMBAL_STOPPED && s.fault==GIMBAL_FEEDBACK);
    GimbalDrive_SetWorld(&s,1.0f,0,now); feedback(1); step(5,true);
    CHECK(s.state==GIMBAL_STOPPED); /* held ARM cannot recover */
    return 0;
}
