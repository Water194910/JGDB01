#ifndef __BNO085_H__
#define __BNO085_H__

#include "main.h"
#include "usart.h"

// SHTP通道定义
#define SHTP_CHANNEL_COMMAND        0
#define SHTP_CHANNEL_EXECUTABLE     1
#define SHTP_CHANNEL_CONTROL        2
#define SHTP_CHANNEL_REPORT         3
#define SHTP_CHANNEL_WAKE_REPORT    4
#define SHTP_CHANNEL_GYRO_ROTATION  5

// SHTP命令ID
#define SHTP_CMD_PRODUCT_ID_REQUEST  0xF9
#define SHTP_CMD_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_CMD_SET_FEATURE         0xFD
#define SHTP_CMD_GET_FEATURE_REQUEST 0xFE
#define SHTP_CMD_GET_FEATURE_RESPONSE 0xFC

// 传感器报告ID
#define REPORT_ID_ROTATION_VECTOR    0x05  // 旋转矢量（姿态）
#define REPORT_ID_GYRO_CALIBRATED    0x02  // 校准陀螺仪（角速度）
#define REPORT_ID_ACCELEROMETER      0x01  // 加速度计

// SHTP头结构
typedef struct {
    uint16_t length;      // 包含头本身的长度
    uint8_t channel;      // 通道号
    uint8_t sequence;     // 序列号
    uint8_t continuation; // 连续标志
} SHTP_Header_t;

// 旋转矢量数据（姿态）
typedef struct {
    uint8_t status;       // 精度状态（0不可靠，1低，2中，3高）
    uint16_t delay;       // 延迟
    float q1, q2, q3, q4; // 四元数
    float yaw, pitch, roll; // 欧拉角（度）
} RotationVector_t;

// 陀螺仪数据（角速度）
typedef struct {
    uint8_t status;       // 精度状态
    uint16_t delay;       // 延迟
    float x, y, z;        // 角速度（rad/s）
} GyroCalibrated_t;

// BNO085数据结构
typedef struct {
    RotationVector_t rotation;
    GyroCalibrated_t gyro;
    uint8_t initialized;
    uint8_t data_ready;
} BNO085_Data_t;

// 函数声明
void BNO085_Init(void);
void BNO085_ProcessData(uint8_t *data, uint16_t len);
void BNO085_PushRxData(uint8_t *data, uint16_t len);
void BNO085_Service(void);
void BNO085_SendProductIDRequest(void);
void BNO085_EnableRotationVector(uint16_t interval_ms);
void BNO085_EnableGyroCalibrated(uint16_t interval_ms);
void BNO085_ConvertToEuler(RotationVector_t *rv);

// 外部变量
extern BNO085_Data_t bno085_data;
extern uint8_t bno085_rx_buf[256];
extern volatile uint16_t bno085_rx_len;
extern volatile uint8_t bno085_frame_ready;

#endif
