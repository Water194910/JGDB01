#ifndef __GM6020_CAN_H
#define __GM6020_CAN_H

#include <stdint.h>
#include "can.h"
#include <string.h>
#include "main.h"


#define Voltage 0
#define Current 1
#define LIMIT_MIN_MAX(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define MOTOR_MAX_RATED_SPEED       132     // 额定安全转速（无负载时）
#define MOTOR_MAX_ELECTRICAL_SPEED  320     // 电气极限转速（带负载）
#define MOTOR_DEFAULT_SPEED_LIMIT   200     // 推荐默认限速

#define MOTOR_MAX_RATED_SPEED_DU      132*6     // 额定安全转速（无负载时）
#define MOTOR_MAX_ELECTRICAL_SPEED_DU  320*6     // 电气极限转速（带负载）
#define MOTOR_DEFAULT_SPEED_LIMIT_DU   200*6     // 推荐默认限速


#define MOTOR_Voltage_MAX           25000
#define MOTOR_Voltage_MIN           -25000
#define MOTOR_Current_MAX           16384
#define MOTOR_Current_MIN           -16384
#define RPM_TO_RADS 0.104719755f
#define RPM_TO_DEGS 6.0f
#define PI          3.1415926f
#define TWO_PI      6.2831853f
#define ENCODER_RESOLUTION 8192.0f
#define RAD_PER_TICK (6.2831853f / 8192.0f)  // 0.0007669903f
#define DEG_PER_TICK (360.0f / ENCODER_RESOLUTION)
#define MOTOR1_OFFSET 4095  //机械臂


typedef struct
{
	int16_t  rotor_last_angle;
    int16_t  rotor_angle;
	float  rotor_rad_angle;
	float  rotor_du_angle;
    int16_t  rotor_speed;
	float  rotor_rad_speed;
	float  rotor_du_speed;
    int16_t  torque_current;
    uint8_t  motor_temperature;
    int16_t  last_ecd;
	int32_t  turn_count;
	int32_t  total_ticks;
	float  total_rad;
	float  total_du;
	uint16_t offset_angle;

} Motor_Date;



typedef enum
{
//	CAN_6020Moto_ALL_ID = 0x200,
	CAN_6020Moto1_ID = 0x205,
	CAN_6020Moto2_ID = 0x206,
	CAN_6020Moto3_ID = 0x207,
	CAN_6020Moto4_ID = 0x208,
}CAN_Message_ID;



typedef struct
{
	float Kp, Ki,Kd;
	float actual_val;
	float target_val;
	float err;
	float err_last;
	float err_sum;
}pid;

typedef struct
{
    pid outer;
    pid inner;
	float output;
}pid_Cascade;



typedef pid * pid_t;

extern uint8_t TxData[8];
extern uint8_t RxData[8];
extern volatile Motor_Date motor_date[];
extern CAN_FilterTypeDef sFilterConfig;
extern CAN_TxHeaderTypeDef TxMessage;
extern CAN_RxHeaderTypeDef RxMessage;

float shortest_path_error(float target, float current);
float Pid_Speed(pid_t pid ,float actual_val,float target_val);
float Pid_Position(pid_t pid ,float actual_val,float target_val);
float GM6020_ST_Control(pid_Cascade*pid,float actual_val,float target_val,uint8_t ID);
float GM6020_MT_Control(pid_Cascade*pid,float actual_val,float target_val,uint8_t ID);
void Pid_Init(pid_t pid,float kp,float ki, float kd);
void Configure_Filter(void);
void CAN_Transmit(int16_t data1,int16_t data2,int16_t data3,int16_t data4,uint8_t mode);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan);

#endif
