#include "bno085.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

// BNO085数据
BNO085_Data_t bno085_data = {0};

// USART2 DMA接收缓冲区
uint8_t bno085_rx_buf[256];
volatile uint16_t bno085_rx_len = 0;
volatile uint8_t bno085_frame_ready = 0;

// SHTP序列号
static uint8_t shtp_sequence[6] = {0}; // 6个通道的序列号

/**
 * @brief 解析SHTP头
 */
static void BNO085_ParseSHTPHeader(uint8_t *data, SHTP_Header_t *header)
{
    header->length = (uint16_t)(data[0] | (data[1] << 8));
    header->continuation = (data[1] >> 7) & 0x01;
    header->channel = data[2];
    header->sequence = data[3];
}

/**
 * @brief 发送SHTP命令
 */
static void BNO085_SendSHTPCommand(uint8_t channel, uint8_t *payload, uint16_t payload_len)
{
    uint8_t packet[32];
    uint16_t total_len = payload_len + 4; // SHTP头4字节

    // 构建SHTP头
    packet[0] = total_len & 0xFF;
    packet[1] = (total_len >> 8) & 0x7F;
    packet[2] = channel;
    packet[3] = shtp_sequence[channel]++;

    // 复制payload
    if (payload_len > 0 && payload_len <= 28) {
        memcpy(&packet[4], payload, payload_len);
    }

    // 通过USART2发送（需要逐字节发送，保证字节间隔）
    for (uint16_t i = 0; i < total_len; i++) {
        HAL_UART_Transmit(&huart2, &packet[i], 1, 10);
        // 字节间隔至少100us（UART-SHTP要求）
        HAL_Delay(1);
    }
}

/**
 * @brief 发送Product ID请求
 */
void BNO085_SendProductIDRequest(void)
{
    uint8_t payload[1] = {SHTP_CMD_PRODUCT_ID_REQUEST};
    BNO085_SendSHTPCommand(SHTP_CHANNEL_CONTROL, payload, 1);
}

/**
 * @brief 使能旋转矢量（姿态）
 * @param interval_ms 报告间隔（毫秒）
 */
void BNO085_EnableRotationVector(uint16_t interval_ms)
{
    uint8_t cmd[17];

    // Set Feature命令
    cmd[0] = SHTP_CMD_SET_FEATURE;           // 0xFD
    cmd[1] = REPORT_ID_ROTATION_VECTOR;      // 0x05
    cmd[2] = 0x00;                           // 功能标志
    cmd[3] = 0x00;                           // 敏感度

    // 报告间隔（微秒，小端）
    uint32_t interval_us = (uint32_t)interval_ms * 1000;
    cmd[4] = interval_us & 0xFF;
    cmd[5] = (interval_us >> 8) & 0xFF;
    cmd[6] = (interval_us >> 16) & 0xFF;
    cmd[7] = (interval_us >> 24) & 0xFF;

    cmd[8] = 0x00;  // 批量状态
    cmd[9] = 0x00;  // 传感器特定配置
    cmd[10] = 0x00;
    cmd[11] = 0x00;
    cmd[12] = 0x00;

    BNO085_SendSHTPCommand(SHTP_CHANNEL_CONTROL, cmd, 13);
}

/**
 * @brief 使能校准陀螺仪（角速度）
 * @param interval_ms 报告间隔（毫秒）
 */
void BNO085_EnableGyroCalibrated(uint16_t interval_ms)
{
    uint8_t cmd[17];

    // Set Feature命令
    cmd[0] = SHTP_CMD_SET_FEATURE;           // 0xFD
    cmd[1] = REPORT_ID_GYRO_CALIBRATED;      // 0x02
    cmd[2] = 0x00;                           // 功能标志
    cmd[3] = 0x00;                           // 敏感度

    // 报告间隔（微秒，小端）
    uint32_t interval_us = (uint32_t)interval_ms * 1000;
    cmd[4] = interval_us & 0xFF;
    cmd[5] = (interval_us >> 8) & 0xFF;
    cmd[6] = (interval_us >> 16) & 0xFF;
    cmd[7] = (interval_us >> 24) & 0xFF;

    cmd[8] = 0x00;  // 批量状态
    cmd[9] = 0x00;  // 传感器特定配置
    cmd[10] = 0x00;
    cmd[11] = 0x00;
    cmd[12] = 0x00;

    BNO085_SendSHTPCommand(SHTP_CHANNEL_CONTROL, cmd, 13);
}

/**
 * @brief 解析旋转矢量报告
 */
static void BNO085_ParseRotationVector(uint8_t *data, uint16_t len)
{
    if (len < 17) return; // 至少需要17字节

    bno085_data.rotation.status = data[1];
    bno085_data.rotation.delay = (uint16_t)(data[2] | (data[3] << 8));

    // 四元数（Q14格式，需要转换为float）
    int16_t q1_raw = (int16_t)(data[4] | (data[5] << 8));
    int16_t q2_raw = (int16_t)(data[6] | (data[7] << 8));
    int16_t q3_raw = (int16_t)(data[8] | (data[9] << 8));
    int16_t q4_raw = (int16_t)(data[10] | (data[11] << 8));

    // Q14格式转换为float（范围-1到1）
    bno085_data.rotation.q1 = (float)q1_raw / 16384.0f;
    bno085_data.rotation.q2 = (float)q2_raw / 16384.0f;
    bno085_data.rotation.q3 = (float)q3_raw / 16384.0f;
    bno085_data.rotation.q4 = (float)q4_raw / 16384.0f;

    // 转换为欧拉角
    BNO085_ConvertToEuler(&bno085_data.rotation);
}

/**
 * @brief 解析校准陀螺仪报告
 */
static void BNO085_ParseGyroCalibrated(uint8_t *data, uint16_t len)
{
    if (len < 16) return; // 至少需要16字节

    bno085_data.gyro.status = data[1];
    bno085_data.gyro.delay = (uint16_t)(data[2] | (data[3] << 8));

    // 角速度（Q9格式，单位rad/s）
    int16_t x_raw = (int16_t)(data[4] | (data[5] << 8));
    int16_t y_raw = (int16_t)(data[6] | (data[7] << 8));
    int16_t z_raw = (int16_t)(data[8] | (data[9] << 8));

    // Q9格式转换为float（范围-256到256 rad/s）
    bno085_data.gyro.x = (float)x_raw / 512.0f;
    bno085_data.gyro.y = (float)y_raw / 512.0f;
    bno085_data.gyro.z = (float)z_raw / 512.0f;
}

/**
 * @brief 处理接收到的SHTP数据
 */
void BNO085_ProcessData(uint8_t *data, uint16_t len)
{
    if (len < 4) return; // 至少需要SHTP头

    SHTP_Header_t header;
    BNO085_ParseSHTPHeader(data, &header);

    // 检查长度是否有效
    if (header.length > len) return;

    uint8_t *payload = &data[4];
    uint16_t payload_len = header.length - 4;

    // 根据通道处理
    switch (header.channel) {
        case SHTP_CHANNEL_CONTROL:
            // 控制通道响应（如Product ID Response）
            if (payload_len > 0 && payload[0] == SHTP_CMD_PRODUCT_ID_RESPONSE) {
                // 收到Product ID响应，说明通信正常
                bno085_data.initialized = 1;
            }
            break;

        case SHTP_CHANNEL_REPORT:
        case SHTP_CHANNEL_GYRO_ROTATION:
            // 传感器报告
            if (payload_len > 0) {
                switch (payload[0]) {
                    case REPORT_ID_ROTATION_VECTOR:
                        BNO085_ParseRotationVector(payload, payload_len);
                        bno085_data.data_ready = 1;
                        break;

                    case REPORT_ID_GYRO_CALIBRATED:
                        BNO085_ParseGyroCalibrated(payload, payload_len);
                        bno085_data.data_ready = 1;
                        break;

                    default:
                        break;
                }
            }
            break;

        default:
            break;
    }
}

/**
 * @brief 将四元数转换为欧拉角（度）
 */
void BNO085_ConvertToEuler(RotationVector_t *rv)
{
    float q1 = rv->q1, q2 = rv->q2, q3 = rv->q3, q4 = rv->q4;

    // Roll (X轴旋转)
    float sinr_cosp = 2.0f * (q1 * q2 + q3 * q4);
    float cosr_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    rv->roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / 3.14159265f;

    // Pitch (Y轴旋转)
    float sinp = 2.0f * (q1 * q3 - q4 * q2);
    if (fabsf(sinp) >= 1.0f) {
        rv->pitch = copysignf(90.0f, sinp); // 万向节锁
    } else {
        rv->pitch = asinf(sinp) * 180.0f / 3.14159265f;
    }

    // Yaw (Z轴旋转)
    float siny_cosp = 2.0f * (q1 * q4 + q2 * q3);
    float cosy_cosp = 1.0f - 2.0f * (q3 * q3 + q4 * q4);
    rv->yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / 3.14159265f;
}

/**
 * @brief BNO085初始化
 */
void BNO085_Init(void)
{
    // 清空数据
    memset(&bno085_data, 0, sizeof(BNO085_Data_t));

    // 等待BNO085启动（H_INTN拉低表示就绪）
    HAL_Delay(100);

    // 发送Product ID请求验证通信
    BNO085_SendProductIDRequest();
    HAL_Delay(50);

    // 使能旋转矢量（10ms间隔 = 100Hz）
    BNO085_EnableRotationVector(10);
    HAL_Delay(10);

    // 使能校准陀螺仪（10ms间隔 = 100Hz）
    BNO085_EnableGyroCalibrated(10);
    HAL_Delay(10);

    // 启动USART2 DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, bno085_rx_buf, 256);
}
