/* HAL test doubles: compiled for Cortex-M4 and run by run_motor_tests.py. */
#include "yaw_motor.h"
#include <math.h>
#include <stddef.h>

#define CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

static uint32_t tick, free_mailboxes = 3, pending;
static CAN_FilterTypeDef configured_filter;
static CAN_TxHeaderTypeDef sent_header;
static uint8_t sent_data[8];
static uint32_t send_count;
static HAL_StatusTypeDef config_result = HAL_OK;
static HAL_StatusTypeDef tx_result = HAL_OK;
static CAN_RxHeaderTypeDef rx_header;
static uint8_t rx_data[8];

uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *h, const CAN_FilterTypeDef *f)
{ (void)h; configured_filter = *f; return config_result; }
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *h)
{ h->State = HAL_CAN_STATE_LISTENING; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *h)
{ h->State = HAL_CAN_STATE_READY; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h, uint32_t mask)
{ (void)h; return mask == CAN_IT_RX_FIFO1_MSG_PENDING ? HAL_OK : HAL_ERROR; }
uint32_t HAL_CAN_GetRxFifoFillLevel(const CAN_HandleTypeDef *h, uint32_t fifo)
{ (void)h; return fifo == CAN_RX_FIFO1 ? pending : 0; }
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h, uint32_t fifo,
                                     CAN_RxHeaderTypeDef *header, uint8_t data[])
{
    (void)h; (void)fifo;
    *header = rx_header;
    for (unsigned i = 0; i < 8; ++i) data[i] = rx_data[i];
    --pending;
    return HAL_OK;
}
HAL_CAN_StateTypeDef HAL_CAN_GetState(const CAN_HandleTypeDef *h) { return h->State; }
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *h)
{ (void)h; return free_mailboxes; }
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h, const CAN_TxHeaderTypeDef *header,
                                     const uint8_t data[], uint32_t *mailbox)
{
    (void)h;
    sent_header = *header;
    for (unsigned i = 0; i < header->DLC; ++i) sent_data[i] = data[i];
    *mailbox = CAN_TX_MAILBOX0;
    ++send_count;
    return tx_result;
}

int main(void) {
 CAN_HandleTypeDef can={.Instance=CAN1,.State=HAL_CAN_STATE_LISTENING};
 YawMotor_Ranges r={3.14159298f,21,7.5f}; /* hardware readback */
 CHECK(YawMotor_Enable()==HAL_ERROR);
 YawMotor_Ranges bad={NAN,30,8};
 CHECK(YawMotor_Init(&can,&bad)==HAL_ERROR);
 CHECK(YawMotor_Init(&can,&r)==HAL_OK && send_count==0);
 CHECK(YawMotor_RequestFeedback()==HAL_OK);
 CHECK(sent_header.StdId==0x7ff && sent_header.DLC==8 && sent_data[0]==3 && sent_data[2]==0xcc);
 CHECK(sent_data[1]==0 && sent_data[3]==0 && sent_data[7]==0);
 CHECK(YawMotor_Enable()==HAL_OK && sent_header.StdId==3 && sent_data[7]==252);
 CHECK(YawMotor_SendMIT(0,0,0,0,0)==HAL_OK);
 CHECK(sent_data[0]==127 && sent_data[1]==255 && sent_data[7]==255);
 CHECK(YawMotor_SendMIT(0,0,0,0,7.51f)==HAL_ERROR);
 CHECK(YawMotor_SendMIT(0,21.01f,0,0,0)==HAL_ERROR);
 CHECK(YawMotor_SendMIT(3.142f,0,0,0,0)==HAL_ERROR);
 CHECK(YawMotor_SendMIT(-r.pmax,-21,0,0,-7.5f)==HAL_OK);
 for(unsigned i=0;i<8;++i) CHECK(sent_data[i]==0);
 CHECK(YawMotor_SendMIT(NAN,0,0,0,0)==HAL_ERROR);
 CHECK(YawMotor_SendMIT(r.pmax,21,500,5,7.5f)==HAL_ERROR);
 /* One free mailbox is enough for one frame; only a full queue is BUSY. */
 free_mailboxes=2; CHECK(YawMotor_Disable()==HAL_OK);
 free_mailboxes=0; CHECK(YawMotor_Disable()==HAL_BUSY); free_mailboxes=3;
 CAN_RxHeaderTypeDef h={.StdId=0x13,.DLC=8,.IDE=CAN_ID_STD,.RTR=CAN_RTR_DATA};
 uint8_t d[8]={0x13,255,255,0,15,255,42,53};
 tick=0xfffffff0;
 CHECK(YawMotor_Receive(&h,d));
 YawMotor_Feedback f;
 CHECK(YawMotor_GetFeedback(&f) && f.scaled && f.pos==r.pmax && f.vel==-21 && f.torque==7.5f);
 CHECK(f.state==1 && f.rx_count==1 && f.mos_temperature==42);
 tick=0x10; CHECK(YawMotor_IsOnline(33) && !YawMotor_IsOnline(32));
 h.StdId=0x19; CHECK(!YawMotor_Receive(&h,d));
 h.StdId=0x13; d[0]=0x19; CHECK(!YawMotor_Receive(&h,d));
 __disable_irq(); CHECK(YawMotor_GetFeedback(&f) && __get_PRIMASK()==1); __enable_irq();
 YawMotor_Params params;
 uint32_t before=send_count;
 YawMotor_ParamService(false); CHECK(send_count==before);
 YawMotor_ParamService(true);
 CHECK(sent_header.StdId==0x7ff && sent_data[0]==3 && sent_data[2]==0x33 && sent_data[3]==10);
 uint8_t reply[8]={3,0,0x33,10,1,0,0,0};
 CHECK(YawMotor_ReceiveParam(&h,reply));
 YawMotor_GetParams(&params); CHECK(params.control_mode==1 && params.valid_mask==1);
 YawMotor_ParamService(true); CHECK(sent_data[3]==21);
 reply[3]=22; CHECK(YawMotor_ReceiveParam(&h,reply));
 YawMotor_GetParams(&params); CHECK(params.valid_mask==1);
 reply[3]=21; reply[4]=0; reply[5]=0; reply[6]=0x40; reply[7]=0x41; /* 12 */
 CHECK(YawMotor_ReceiveParam(&h,reply));
 YawMotor_GetParams(&params); CHECK(params.pmax==12 && params.valid_mask==3);
 YawMotor_ParamService(true);
 for(unsigned i=0;i<6;++i) {tick+=100; YawMotor_ParamService(true);}
 YawMotor_GetParams(&params); CHECK(params.finished && params.timeout_count==6 && params.valid_mask==3);
 before=send_count; YawMotor_ParamService(true); CHECK(send_count==before);
 return 0;
}
