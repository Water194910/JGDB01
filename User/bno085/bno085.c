#include "bno085.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define BNO085_DMA_BUF_SIZE 512
#define BNO085_UART_FRAME_MAX 512

#define UART_FLAG_BYTE      0x7E
#define UART_ESCAPE_BYTE    0x7D
#define UART_ESCAPE_XOR     0x20
#define UART_PROTO_BSQ_BSN  0x00
#define UART_PROTO_SHTP     0x01

#define REPORT_ID_GAME_ROTATION_VECTOR 0x08

#define BNO085_STATE_WAIT_RESET  0
#define BNO085_STATE_READY       1
#define BNO085_STATE_CONFIGURING 2
#define BNO085_STATE_RUNNING     3

// BNO085数据
BNO085_Data_t bno085_data = {0};

// USART2 DMA接收缓冲区
uint8_t bno085_rx_buf[BNO085_DMA_BUF_SIZE];
volatile uint16_t bno085_rx_len = 0;
volatile uint8_t bno085_frame_ready = 0;

static uint8_t uart_frame[BNO085_UART_FRAME_MAX];
static uint16_t uart_frame_len;
static uint8_t uart_in_frame;
static uint8_t uart_escape;
static uint8_t shtp_sequence[6];
static uint16_t bno085_last_read_pos;
static uint32_t bno085_last_bsq_ms;
static uint8_t bno085_reports_enabled;

static uint8_t BNO085_DmaRead(uint16_t pos)
{
    return bno085_rx_buf[pos % BNO085_DMA_BUF_SIZE];
}

static int16_t BNO085_ReadI16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void BNO085_DelayUs(uint32_t us)
{
    uint32_t cycles_per_us;
    uint32_t cycles;
    uint32_t start;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000u;
    cycles = cycles_per_us * us;
    start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) {
        __NOP();
    }
}

static void BNO085_SendByte(uint8_t b)
{
    (void)HAL_UART_Transmit(&huart2, &b, 1, 10);
    BNO085_DelayUs(120);
}

static void BNO085_SendEscaped(uint8_t b)
{
    if (b == UART_FLAG_BYTE || b == UART_ESCAPE_BYTE) {
        BNO085_SendByte(UART_ESCAPE_BYTE);
        BNO085_SendByte((uint8_t)(b ^ UART_ESCAPE_XOR));
    } else {
        BNO085_SendByte(b);
    }
}

static void BNO085_SendBSQ(void)
{
    BNO085_SendByte(UART_FLAG_BYTE);
    BNO085_SendByte(UART_PROTO_BSQ_BSN);
    BNO085_SendByte(UART_FLAG_BYTE);
}

/**
 * @brief 发送UART-SHTP命令
 */
static void BNO085_SendSHTPCommand(uint8_t channel, const uint8_t *payload, uint16_t payload_len)
{
    uint8_t packet[32];
    uint16_t total_len = payload_len + 4U;

    if (channel >= sizeof(shtp_sequence) || total_len > sizeof(packet)) {
        return;
    }

    packet[0] = (uint8_t)(total_len & 0xFFU);
    packet[1] = (uint8_t)((total_len >> 8) & 0x7FU);
    packet[2] = channel;
    packet[3] = shtp_sequence[channel]++;

    if (payload_len > 0U && payload != NULL) {
        memcpy(&packet[4], payload, payload_len);
    }

    BNO085_SendByte(UART_FLAG_BYTE);
    BNO085_SendByte(UART_PROTO_SHTP);
    for (uint16_t i = 0; i < total_len; i++) {
        BNO085_SendEscaped(packet[i]);
    }
    BNO085_SendByte(UART_FLAG_BYTE);
}

/**
 * @brief 发送Product ID请求
 */
void BNO085_SendProductIDRequest(void)
{
    uint8_t payload[1] = {SHTP_CMD_PRODUCT_ID_REQUEST};
    BNO085_SendSHTPCommand(SHTP_CHANNEL_CONTROL, payload, sizeof(payload));
}

static void BNO085_SetFeature(uint8_t report_id, uint32_t interval_us)
{
    uint8_t cmd[17] = {0};

    cmd[0] = SHTP_CMD_SET_FEATURE;
    cmd[1] = report_id;
    cmd[2] = 0x00;
    cmd[3] = 0x00;
    cmd[4] = 0x00;
    cmd[5] = (uint8_t)(interval_us & 0xFFU);
    cmd[6] = (uint8_t)((interval_us >> 8) & 0xFFU);
    cmd[7] = (uint8_t)((interval_us >> 16) & 0xFFU);
    cmd[8] = (uint8_t)((interval_us >> 24) & 0xFFU);

    BNO085_SendSHTPCommand(SHTP_CHANNEL_CONTROL, cmd, sizeof(cmd));
}

/**
 * @brief 使能旋转矢量（姿态）
 * @param interval_ms 报告间隔（毫秒）
 */
void BNO085_EnableRotationVector(uint16_t interval_ms)
{
    BNO085_SetFeature(REPORT_ID_ROTATION_VECTOR, (uint32_t)interval_ms * 1000U);
    BNO085_SetFeature(REPORT_ID_GAME_ROTATION_VECTOR, (uint32_t)interval_ms * 1000U);
}

/**
 * @brief 使能校准陀螺仪（角速度）
 * @param interval_ms 报告间隔（毫秒）
 */
void BNO085_EnableGyroCalibrated(uint16_t interval_ms)
{
    BNO085_SetFeature(REPORT_ID_GYRO_CALIBRATED, (uint32_t)interval_ms * 1000U);
}

static void BNO085_EnableReports(void)
{
    BNO085_EnableGyroCalibrated(5);
    BNO085_EnableRotationVector(10);
    bno085_reports_enabled = 1;
    bno085_data.state = BNO085_STATE_CONFIGURING;
}

/**
 * @brief 解析旋转矢量报告
 */
static void BNO085_ParseRotationVector(const uint8_t *data, uint16_t len)
{
    if (len < 12U) {
        return;
    }

    bno085_data.rotation.status = data[2] & 0x03U;

    int16_t q1_raw = BNO085_ReadI16(data + 4);
    int16_t q2_raw = BNO085_ReadI16(data + 6);
    int16_t q3_raw = BNO085_ReadI16(data + 8);
    int16_t q4_raw = BNO085_ReadI16(data + 10);

    bno085_data.rotation.q1 = (float)q1_raw / 16384.0f;
    bno085_data.rotation.q2 = (float)q2_raw / 16384.0f;
    bno085_data.rotation.q3 = (float)q3_raw / 16384.0f;
    bno085_data.rotation.q4 = (float)q4_raw / 16384.0f;

    BNO085_ConvertToEuler(&bno085_data.rotation);
    bno085_data.data_ready = 1;

    if (bno085_data.state == BNO085_STATE_CONFIGURING) {
        bno085_data.state = BNO085_STATE_RUNNING;
    }
}

/**
 * @brief 解析校准陀螺仪报告
 */
static void BNO085_ParseGyroCalibrated(const uint8_t *data, uint16_t len)
{
    if (len < 10U) {
        return;
    }

    bno085_data.gyro.status = data[2] & 0x03U;

    int16_t x_raw = BNO085_ReadI16(data + 4);
    int16_t y_raw = BNO085_ReadI16(data + 6);
    int16_t z_raw = BNO085_ReadI16(data + 8);
    const float q9_to_rad_s = 1.0f / 512.0f;

    bno085_data.gyro.x = (float)x_raw * q9_to_rad_s;
    bno085_data.gyro.y = (float)y_raw * q9_to_rad_s;
    bno085_data.gyro.z = (float)z_raw * q9_to_rad_s;
    bno085_data.data_ready = 1;

    if (bno085_data.state == BNO085_STATE_CONFIGURING) {
        bno085_data.state = BNO085_STATE_RUNNING;
    }
}

static void BNO085_ParseSensorReport(uint8_t channel, const uint8_t *payload, uint16_t len)
{
    (void)channel;

    while (len > 0U) {
        uint8_t report_id = payload[0];
        bno085_data.last_report = report_id;

        if (report_id == 0xFBU) {
            if (len < 5U) {
                return;
            }
            payload += 5;
            len = (uint16_t)(len - 5U);
            continue;
        }

        if (report_id == REPORT_ID_ROTATION_VECTOR || report_id == REPORT_ID_GAME_ROTATION_VECTOR) {
            BNO085_ParseRotationVector(payload, len);
            return;
        }

        if (report_id == REPORT_ID_GYRO_CALIBRATED) {
            BNO085_ParseGyroCalibrated(payload, len);
            return;
        }

        return;
    }
}

static void BNO085_ParseSHTPPacket(const uint8_t *packet, uint16_t len)
{
    if (len < 4U) {
        bno085_data.rx_bad++;
        return;
    }

    uint16_t packet_len = (uint16_t)(packet[0] | ((packet[1] & 0x7FU) << 8));
    uint8_t channel = packet[2];

    if (packet_len < 4U || packet_len > len || channel > SHTP_CHANNEL_GYRO_ROTATION) {
        bno085_data.rx_bad++;
        return;
    }

    const uint8_t *payload = packet + 4;
    uint16_t payload_len = (uint16_t)(packet_len - 4U);

    bno085_data.rx_bytes += packet_len;
    bno085_data.rx_packets++;
    bno085_data.last_len = packet_len;
    bno085_data.last_channel = channel;
    bno085_data.last_report = payload_len > 0U ? payload[0] : 0U;

    if (bno085_data.state == BNO085_STATE_WAIT_RESET) {
        bno085_data.state = BNO085_STATE_READY;
        bno085_data.initialized = 1;
    }

    switch (channel) {
        case SHTP_CHANNEL_EXECUTABLE:
            if (payload_len > 0U && payload[0] == 0x01U) {
                bno085_data.state = BNO085_STATE_READY;
                bno085_data.initialized = 1;
            }
            break;

        case SHTP_CHANNEL_CONTROL:
            if (payload_len > 0U && payload[0] == SHTP_CMD_PRODUCT_ID_RESPONSE) {
                bno085_data.initialized = 1;
            }
            break;

        case SHTP_CHANNEL_REPORT:
        case SHTP_CHANNEL_WAKE_REPORT:
        case SHTP_CHANNEL_GYRO_ROTATION:
            BNO085_ParseSensorReport(channel, payload, payload_len);
            break;

        default:
            break;
    }
}

static void BNO085_ParseUARTFrame(const uint8_t *frame, uint16_t len)
{
    if (len < 1U) {
        return;
    }

    if (frame[0] == UART_PROTO_BSQ_BSN) {
        if (len >= 3U) {
            bno085_data.tx_space = (uint16_t)frame[1] | ((uint16_t)frame[2] << 8);
        }
        return;
    }

    if (frame[0] == UART_PROTO_SHTP) {
        BNO085_ParseSHTPPacket(frame + 1, (uint16_t)(len - 1U));
    }
}

static void BNO085_ParseUARTByte(uint8_t b)
{
    if (b == UART_FLAG_BYTE) {
        if (uart_in_frame && uart_frame_len > 0U) {
            BNO085_ParseUARTFrame(uart_frame, uart_frame_len);
        }
        uart_in_frame = 1;
        uart_escape = 0;
        uart_frame_len = 0;
        return;
    }

    if (!uart_in_frame) {
        return;
    }

    if (b == UART_ESCAPE_BYTE) {
        uart_escape = 1;
        return;
    }

    if (uart_escape) {
        b ^= UART_ESCAPE_XOR;
        uart_escape = 0;
    }

    if (uart_frame_len < sizeof(uart_frame)) {
        uart_frame[uart_frame_len++] = b;
    } else {
        bno085_data.rx_bad++;
        uart_in_frame = 0;
        uart_escape = 0;
        uart_frame_len = 0;
    }
}

static void BNO085_ParseRxStream(void)
{
    uint16_t write_pos;

    if (huart2.hdmarx == NULL) {
        return;
    }

    write_pos = (uint16_t)(BNO085_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx));
    bno085_data.dma_pos = write_pos;

    while (bno085_last_read_pos != write_pos) {
        BNO085_ParseUARTByte(BNO085_DmaRead(bno085_last_read_pos));
        bno085_last_read_pos = (uint16_t)((bno085_last_read_pos + 1U) % BNO085_DMA_BUF_SIZE);
    }
}

/**
 * @brief 处理接收到的SHTP数据
 */
void BNO085_ProcessData(uint8_t *data, uint16_t len)
{
    BNO085_ParseSHTPPacket(data, len);
}

/**
 * @brief 兼容旧ReceiveToIdle路径：将收到的数据按UART-SHTP流解析
 */
void BNO085_PushRxData(uint8_t *data, uint16_t len)
{
    if (data == NULL) {
        return;
    }

    bno085_rx_len = len;
    for (uint16_t i = 0; i < len; i++) {
        BNO085_ParseUARTByte(data[i]);
    }
}

/**
 * @brief 从DMA循环缓冲解析UART-SHTP数据，并推进初始化状态机
 */
void BNO085_Service(void)
{
    BNO085_ParseRxStream();

    bno085_data.uart_error = HAL_UART_GetError(&huart2);

    if ((bno085_data.state == BNO085_STATE_READY) && !bno085_reports_enabled) {
        if (bno085_data.tx_space >= 80U) {
            BNO085_EnableReports();
        } else if ((HAL_GetTick() - bno085_last_bsq_ms) >= 20U) {
            bno085_last_bsq_ms = HAL_GetTick();
            BNO085_SendBSQ();
        }
    }
}

/**
 * @brief USART2 IDLE中断入口
 */
void BNO085_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);
        bno085_data.idle_irq++;
        bno085_frame_ready = 1;
    }
}

/**
 * @brief 将四元数转换为欧拉角（度）
 */
void BNO085_ConvertToEuler(RotationVector_t *rv)
{
    float qi = rv->q1;
    float qj = rv->q2;
    float qk = rv->q3;
    float qr = rv->q4;
    const float rad_to_deg = 180.0f / M_PI;

    float sinr_cosp = 2.0f * (qr * qi + qj * qk);
    float cosr_cosp = 1.0f - 2.0f * (qi * qi + qj * qj);
    rv->roll = atan2f(sinr_cosp, cosr_cosp) * rad_to_deg;

    float sinp = 2.0f * (qr * qj - qk * qi);
    if (sinp >= 1.0f) {
        rv->pitch = 90.0f;
    } else if (sinp <= -1.0f) {
        rv->pitch = -90.0f;
    } else {
        rv->pitch = asinf(sinp) * rad_to_deg;
    }

    float siny_cosp = 2.0f * (qr * qk + qi * qj);
    float cosy_cosp = 1.0f - 2.0f * (qj * qj + qk * qk);
    rv->yaw = atan2f(siny_cosp, cosy_cosp) * rad_to_deg;
}

/**
 * @brief BNO085初始化
 */
void BNO085_Init(void)
{
    memset(&bno085_data, 0, sizeof(BNO085_Data_t));
    memset(bno085_rx_buf, 0, sizeof(bno085_rx_buf));
    memset(shtp_sequence, 0, sizeof(shtp_sequence));

    bno085_rx_len = 0;
    bno085_frame_ready = 0;
    uart_frame_len = 0;
    uart_in_frame = 0;
    uart_escape = 0;
    bno085_last_read_pos = 0;
    bno085_last_bsq_ms = 0;
    bno085_reports_enabled = 0;
    bno085_data.state = BNO085_STATE_WAIT_RESET;

    HAL_Delay(100);

    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    bno085_data.rx_status = HAL_UART_Receive_DMA(&huart2, bno085_rx_buf, sizeof(bno085_rx_buf));

    HAL_Delay(20);
    BNO085_SendProductIDRequest();
    BNO085_SendBSQ();
}
