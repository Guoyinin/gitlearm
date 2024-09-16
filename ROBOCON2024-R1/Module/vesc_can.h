#ifndef __VESC_CAN_H__
#define  __VESC_CAN_H__


#include "dataytype.h"
#include "buffer.h"
#include "bldc_interface.h"
#include <String.h>
#include "struct_typedef.h"
#include "pid.h"
#include "can.h"

#define RX_BUFFER_SIZE 1024
#define CAN_STATUS_MSGS_TO_STORE 10 // The number of status Messages to store
#define FILTER_BUF_LEN		5
#define VESC_MOTOR_SPEED_PID_KP 2.0f
#define VESC_MOTOR_SPEED_PID_KI 0.0f
#define VESC_MOTOR_SPEED_PID_KD 1.0f
#define VESC_MOTOR_SPEED_PID_MAX_OUT 30000.0f
#define VESC_MOTOR_SPEED_PID_MAX_IOUT 20000.0f
#define VESC_MOTOR_SPEED_PID_DEADBAND 0.0f


void vesc_can_begin();
void vesc_can_set_duty(uint8_t controller_id, float duty);
void comm_can_set_current(uint8_t controller_id, float current);
void comm_can_set_current_brake(uint8_t controller_id, float current);
void comm_can_set_rpm(uint8_t controller_id, float rpm);
void comm_can_set_pos(uint8_t controller_id, float pos);
bool sendPacket(uint8_t id,uint8_t packet[],int32_t len);
void can_process_packet(unsigned char *data, unsigned int len);

extern unsigned short crc16(unsigned char *buf, unsigned int len);
void can_process_packet(unsigned char *data, unsigned int len);
int vesc_can_read();



typedef enum {
   CAN_PACKET_SET_DUTY = 0,
   CAN_PACKET_SET_CURRENT,
   CAN_PACKET_SET_CURRENT_BRAKE,
   CAN_PACKET_SET_RPM,
   CAN_PACKET_SET_POS,
   CAN_PACKET_FILL_RX_BUFFER,
   CAN_PACKET_FILL_RX_BUFFER_LONG,
   CAN_PACKET_PROCESS_RX_BUFFER,
   CAN_PACKET_PROCESS_SHORT_BUFFER,
   CAN_PACKET_STATUS
} CAN_PACKET_ID;

typedef struct {
  int id;
  float rpm;
  float current;
  float duty;
} can_status_msg;

typedef struct CAN_message_t {
  uint32_t id; 
  uint8_t ext; 
  uint8_t len; 
  uint16_t timeout; 
  uint8_t buf[8];
	
} CAN_message_t;

typedef struct CAN_filter_t {
  uint8_t rtr;
  uint8_t ext;
  uint32_t id;
} CAN_filter_t;
typedef enum
{
  // 底盘
  CAN_vesc_M1_ID = 0x001,
  CAN_vesc_M2_ID = 0x002,
  CAN_vesc_M3_ID = 0x003,


} can_vesc_id_e;

typedef struct
{
  int32_t  speed_rpm;
  
  int16_t  Duty_Cycle;
  int16_t  Total_current;
  int16_t   PID_POS;
  int16_t   current_in;
  int16_t   Motor_Temp;
  int16_t   FET_Temp;
//  int16_t given_current;

//  uint8_t  	hall;
//  uint16_t 	angle;				//abs angle range:[0,8191]
//  uint16_t 	last_angle;	//abs angle range:[0,8191]
//  uint16_t	offset_angle;
//  int32_t		round_cnt;
//  int32_t		total_angle;

//  uint8_t		buf_idx;
//  uint16_t	angle_buf[FILTER_BUF_LEN];
//  uint16_t	fited_angle;
//  uint32_t	msg_cnt;
    
}vesc_motor_measure;
typedef struct
{  
 
  can_vesc_id_e id;
//  can_bus can;
  fp32 speed_rpm_set;
  fp32 current;
  fp32 angle_set; //设置旋转角度；转子角度8192*减速比36*转轴旋转圈数 x/2pi
  int16_t give_current;
	
// const vesc_motor_measure_t *measure;
 const vesc_motor_measure *measure_vesc;
  // fp32 accel;

  PID_TypeDef velocity_pid;
  PID_TypeDef position_pid;

} vesc_motor_t;
void vesc_motor_vel_control_set(vesc_motor_t *motor);
void vesc_spm(vesc_motor_t *motor);
//void vesc_int(vesc_motor_t *motor, can_bus can, can_vesc_id_e id);
void vesc_get_date1(vesc_motor_measure *ptr, uint8_t *Data);
void vesc_get_date2(vesc_motor_measure *ptr, uint8_t *Data);
#endif
