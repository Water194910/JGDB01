/*主控*/
#include "gm6020_can.h"


CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef TxMessage;
CAN_RxHeaderTypeDef RxMessage;//这些结构体是hal库提供的

uint8_t TxData[8];
uint8_t aData[8];



uint32_t TxMailbox;//作用 "硬件发送缓冲/调度位"

volatile Motor_Date motor_date[4];



void Configure_Filter(void) //配置过滤器
{

    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;        //IdHigh/Low：匹配目标 ID。32位模式下，掩码模式："目标 ID要匹配的基准值"。掩码模式：(接收ID & 掩码) == (配置ID & 掩码)。List 模式："第一个ID"
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;     //MaskHigh/Low：掩码模式："掩码"。List 模式："第二个ID"
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;  //FIFO：匹配后放到哪个队列，最终决定哪个中断回调
    sFilterConfig.FilterBank = 0;              //Bank：选第几个过滤器组，0-13 或 0-27。
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  //Scale：32位（匹配1个ID） 或 16位（可匹配2个ID）。
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  //Mode：Mask（模糊匹配） List（精确匹配）
    sFilterConfig.FilterActivation = ENABLE;     //Activation：ENABLE 使生效
    sFilterConfig.SlaveStartFilterBank = 14;     // 有疑问  F407系列有双 CAN 芯片 表示：过滤器组 0-13 给 CAN1 用，14-27 给 CAN2 用。 当前这里只用了CAN1;


    if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig)!=HAL_OK)    //过滤器配置是否成功写入硬件
    {

        Error_Handler();
    }
    if(HAL_CAN_Start(&hcan1)!=HAL_OK)                           //CAN总线是否成功启动
    {

        Error_Handler();
    }
    //当FIFO0收到新消息时，触发中断
    if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK)   //接收中断是否成功启动
    {

        Error_Handler();
    }
}


void CAN_Transmit(int16_t data1,int16_t data2,int16_t data3,int16_t data4,uint8_t mode) //发送指令
{

	memset(&TxMessage, 0, sizeof(TxMessage));

   switch(mode)
   {
	   case Voltage:
	   {
    TxMessage.StdId=0x1FF;
	   }
	    break;

	    case Current:
	   {
    TxMessage.StdId=0x1FE;
	   }
	    break;

	   default: return;  // 非法 mode 直接返回
   }

    TxMessage.IDE=CAN_ID_STD;  //标识类型：可选 CAN_ID_STD（0）为标准帧。   CAN_ID_EXT（1）为扩展帧。
    TxMessage.RTR=CAN_RTR_DATA;  //远程传输： CAN_RTR_DATA 数据帧（0）。   CAN_RTR_REMOTE 远程帧（1）。
    TxMessage.DLC=8;  //数据长度传输

    //data
    TxData[0] = (data1>>8); //&0xff
    TxData[1] = (data1);    //&0xff
    TxData[2] = (data2>>8);
    TxData[3] = (data2);
    TxData[4] = (data3>>8);
    TxData[5] = (data3);
    TxData[6] = (data4>>8);
    TxData[7] = (data4);


    if(HAL_CAN_AddTxMessage(&hcan1,&TxMessage,TxData,&TxMailbox)!= HAL_OK)
    {

        Error_Handler();
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{

    if(hcan->Instance==CAN1)
    {
        uint8_t MotorID;

        if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxMessage,aData)==HAL_OK)
        {

				switch(RxMessage.StdId)
			 {
			case CAN_6020Moto1_ID: MotorID=0,motor_date[MotorID].offset_angle=4095; break;
			case CAN_6020Moto2_ID: MotorID=1,motor_date[MotorID].offset_angle=4095; break;
			case CAN_6020Moto3_ID: MotorID=2,motor_date[MotorID].offset_angle=4095; break;
			case CAN_6020Moto4_ID: MotorID=3,motor_date[MotorID].offset_angle=4095; break;
				 default: return;
			 }



            motor_date[MotorID].rotor_angle       = ((aData[0] << 8) | aData[1])-motor_date[MotorID].offset_angle;//转子机械角度
			motor_date[MotorID].rotor_rad_angle	  =  motor_date[MotorID].rotor_angle*RAD_PER_TICK; //rad
			motor_date[MotorID].rotor_du_angle	  =  motor_date[MotorID].rotor_angle*DEG_PER_TICK;//度
            motor_date[MotorID].rotor_speed       = ((aData[2] << 8) | aData[3]);
			motor_date[MotorID].rotor_rad_speed   =  motor_date[MotorID].rotor_speed*RPM_TO_RADS;//rad/s
			motor_date[MotorID].rotor_du_speed    =  motor_date[MotorID].rotor_speed*RPM_TO_DEGS;//度/s
            motor_date[MotorID].torque_current    = ((aData[4] << 8) | aData[5]);
            motor_date[MotorID].motor_temperature =   aData[6];
			int16_t delta = motor_date[MotorID].rotor_angle - motor_date[MotorID].rotor_last_angle;  //多圈计数逻辑
			 if(delta > 4096)
            motor_date[MotorID].turn_count--;
            else if(delta < -4096)
            motor_date[MotorID].turn_count++;

			motor_date[MotorID].total_ticks= motor_date[MotorID].turn_count*8192+motor_date[MotorID].rotor_angle;//多圈总角度
			motor_date[MotorID].total_rad  =  motor_date[MotorID].total_ticks*RAD_PER_TICK;
			motor_date[MotorID].total_du   =  motor_date[MotorID].total_ticks*DEG_PER_TICK;

			motor_date[MotorID].rotor_last_angle = motor_date[MotorID].rotor_angle;



            HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
        }
    }
}



void Pid_Init(pid_t pid,float kp,float ki, float kd)
{
	pid->Kp = kp;
    pid->Ki = ki;
	pid->Kd = kd;
	pid->actual_val=pid->err_sum=pid->target_val=0;

}


float Pid_Speed(pid_t pid ,float actual_val,float target_val)
{
	pid->target_val = target_val;
	pid->actual_val = actual_val;//更新实际值
    pid->err = pid->target_val - pid->actual_val;////当前误差=目标值-实际值
    pid->err_sum += pid->err;//误差累计值 = 当前误差累加和


	float err_out = pid->Kp*pid->err
			      + pid->Ki*pid->err_sum
				  + pid->Kd*(pid->err - pid->err_last);
	 pid->err_last = pid->err;

	return err_out= LIMIT_MIN_MAX(err_out,MOTOR_Voltage_MIN,MOTOR_Voltage_MAX);
}


float Pid_Position(pid_t pid ,float actual_val,float target_val)
{
	pid->target_val = target_val;
	pid->actual_val = actual_val;//更新实际值
    pid->err = pid->target_val - pid->actual_val;////当前误差=目标值-实际值


	float err_out = pid->Kp*pid->err;

	return err_out= LIMIT_MIN_MAX(err_out,-MOTOR_MAX_RATED_SPEED_DU ,MOTOR_MAX_RATED_SPEED_DU );

}

// 最短路径误差（角度归一化 ±180度）
float shortest_path_error(float target, float current)
	{
    float error = target - current;  // 原始误差，例如 -340度 或 +20度

    // 如果最远路径>180度，反方向（加360度）
    if (error > 180.0f) {
        error -= 360.0f;
    }
    // 如果最远路径<-180度，反方向（加360度）
    else if (error < -180.0f) {
        error += 360.0f;
    }

    return error;  // 返回范围在 [-180, +180]，即最短路径
}
