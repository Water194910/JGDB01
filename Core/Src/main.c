/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "gm6020_can.h"
#include "pid.h"
#include "bno085.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
  GIMBAL_STATE_BOOT_HOLD = 0,  // 开机短暂保持
  GIMBAL_STATE_SEARCH,         // 未找到目标，水平摆扫
  GIMBAL_STATE_TRACK,          // 视觉追踪
  GIMBAL_STATE_LOST_HOLD       // 目标短暂丢失，保持最后位置
} GimbalState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// maixcam2协议定义
#define CAM_HEADER1     0xAA
#define CAM_HEADER2     0xBB
#define CAM_CMD_SCAN    0
#define CAM_CMD_TRACK   1
#define CAM_STATE_SCAN  0
#define CAM_STATE_TRACK 1

// 云台参数 - 可通过串口动态修改
#define GIMBAL_X_KP_DEFAULT     0.5f     // X轴位置环KP
#define GIMBAL_X_KI_DEFAULT     0.0f     // X轴位置环KI
#define GIMBAL_X_KD_DEFAULT     0.0f     // X轴位置环KD
#define GIMBAL_X_SPEED_KP_DEFAULT 50.0f  // X轴速度环KP（RPM速度环）
#define GIMBAL_X_SPEED_KI_DEFAULT 2.0f   // X轴速度环KI（RPM速度环）
#define GIMBAL_X_SPEED_KD_DEFAULT 6.0f   // X轴速度环KD（RPM速度环）
#define GIMBAL_Y_KP_DEFAULT     0.5f     // Y轴位置环KP
#define GIMBAL_Y_KI_DEFAULT     0.0f     // Y轴位置环KI
#define GIMBAL_Y_KD_DEFAULT     0.0f     // Y轴位置环KD
#define GIMBAL_Y_TARGET_KP_DEFAULT 0.03f // Y轴视觉目标增量比例
#define GIMBAL_DEADZONE         8        // 死区像素
#define GIMBAL_Y_DELTA_MAX      2.0f     // Y轴单次最大角度增量
#define GIMBAL_BOOST_THRESHOLD   10.0f    // boost启动阈值（度）
#define GIMBAL_BOOST_GAIN        0.05f    // boost增益系数（每度误差增加的boost倍数）
#define GIMBAL_BOOST_MAX         3.0f     // boost上限
#define GIMBAL_BOOST_PX_THRESHOLD  120.0f  // boost像素阈值
#define GIMBAL_X_DELTA_MAX_BOOST     25.0f   // boost时X轴单次最大角度增量
#define GIMBAL_POSERR_GUARD_BOOST    180.0f  // boost时pos_err保护门限
#define GIMBAL_X_MAX            540.0f   // X轴最大角度（1.5圈）
#define GIMBAL_Y_MAX            20.0f    // Y轴最大角度
#define GIMBAL_Y_MIN            -20.0f   // Y轴最小角度
#define GIMBAL_K_GYRO_DEFAULT   0.5f     // IMU阻尼系数
#define GIMBAL_IMU_YAW_OFFSET_DEFAULT   0.0f    // Yaw开机偏移（自动Tare）
#define GIMBAL_IMU_ROLL_OFFSET_DEFAULT  0.0f    // Roll开机偏移（自动Tare）
#define GIMBAL_USE_IMU_POS_DEFAULT     0       // Y轴位置环模式（0=编码器，1=IMU Roll）
#define RADS_TO_DEGS           57.2957795f
#define GIMBAL_BOOT_HOLD_MS     500U     // 开机保持时间，等待外设稳定
#define GIMBAL_TRACK_TIMEOUT_MS 300U     // TRACK短暂丢帧保持时间
#define GIMBAL_SEARCH_RANGE_DEG 60.0f    // 搜索时相对开机点左右摆扫范围
#define GIMBAL_SEARCH_STEP_DEG  0.35f    // 搜索时每10ms目标角增量

// GM6020轴映射：motor_date[ID-1]，CAN_Transmit第1~4路分别对应ID1~ID4
#define GIMBAL_X_MOTOR_ID       2       // 水平方向/Yaw：GM6020 ID2
#define GIMBAL_Y_MOTOR_ID       1       // 俯仰方向/Roll：GM6020 ID1
#define GIMBAL_X_MOTOR_INDEX    (GIMBAL_X_MOTOR_ID - 1)
#define GIMBAL_Y_MOTOR_INDEX    (GIMBAL_Y_MOTOR_ID - 1)

// 串口调参缓冲区
// Flash参数存储
#define FLASH_SAVE_ADDR         0x08060000   // Sector7起始地址
#define FLASH_SAVE_MAGIC        0x594E494C   // "LINY" 灵犀的校验头
#define FLASH_SAVE_VERSION      6

typedef struct {
  uint32_t magic;
  uint32_t version;
  float x_kp, x_ki, x_kd;
  float x_speed_kp, x_speed_ki, x_speed_kd;
  float y_kp, y_ki, y_kd;
  float y_target_kp;
  float boost_threshold, boost_gain, boost_max;
  float boost_px_threshold;
  float x_delta_max_boost;
  float poserr_guard_boost;
  float k_gyro;               // IMU阻尼系数
  float imu_yaw_offset;       // Yaw开机偏移
  float imu_roll_offset;      // Roll开机偏移
  uint8_t use_imu_pos;        // Y轴位置环模式（0=编码器，1=IMU Roll）
  uint32_t crc;
} FlashParams;

#define DBG_RX_BUF_SIZE  32
uint8_t dbg_rx_buf[DBG_RX_BUF_SIZE];
volatile uint8_t dbg_rx_idx = 0;
volatile uint8_t dbg_cmd_len = 0;
volatile uint8_t dbg_cmd_ready = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// GM6020 X/Y轴
PID_TypeDef pid_x_speed, pid_x_pos;
PID_TypeDef pid_y_speed, pid_y_pos;
int out;
float sd;
uint8_t time_ms = 0;
volatile uint32_t time_ms_ms = 0;

// 动态PID参数（可通过串口修改）
float gimbal_x_kp = GIMBAL_X_KP_DEFAULT;
float gimbal_x_ki = GIMBAL_X_KI_DEFAULT;
float gimbal_x_kd = GIMBAL_X_KD_DEFAULT;
float gimbal_x_speed_kp = GIMBAL_X_SPEED_KP_DEFAULT;
float gimbal_x_speed_ki = GIMBAL_X_SPEED_KI_DEFAULT;
float gimbal_x_speed_kd = GIMBAL_X_SPEED_KD_DEFAULT;
float gimbal_y_kp = GIMBAL_Y_KP_DEFAULT;
float gimbal_y_ki = GIMBAL_Y_KI_DEFAULT;
float gimbal_y_kd = GIMBAL_Y_KD_DEFAULT;
float gimbal_y_target_kp = GIMBAL_Y_TARGET_KP_DEFAULT;
float gimbal_boost_threshold = GIMBAL_BOOST_THRESHOLD;
float gimbal_boost_gain = GIMBAL_BOOST_GAIN;
float gimbal_boost_max = GIMBAL_BOOST_MAX;
float gimbal_boost_px_threshold = GIMBAL_BOOST_PX_THRESHOLD;
float gimbal_x_delta_max_boost = GIMBAL_X_DELTA_MAX_BOOST;
float gimbal_poserr_guard_boost = GIMBAL_POSERR_GUARD_BOOST;
float gimbal_k_gyro = GIMBAL_K_GYRO_DEFAULT;  // IMU阻尼系数
float gimbal_imu_yaw_offset = GIMBAL_IMU_YAW_OFFSET_DEFAULT;   // Yaw偏移（仅用于观测/调试）
float gimbal_imu_roll_offset = GIMBAL_IMU_ROLL_OFFSET_DEFAULT;  // Roll偏移（Y轴IMU外环零点）
uint8_t gimbal_use_imu_pos = GIMBAL_USE_IMU_POS_DEFAULT;       // Y轴位置环模式（0=编码器，1=IMU Roll）
volatile uint8_t gimbal_x_boost_active = 0;   // boost状态标志(主循环→ISR)

// maixcam2 DMA接收
#define CAM_RX_BUF_SIZE  64
uint8_t cam_rx_buf[CAM_RX_BUF_SIZE];
volatile uint16_t cam_rx_len = 0;       // 本次接收长度
volatile uint8_t cam_frame_ready = 0;   // 帧就绪标志
uint8_t cam_cmd = 0;
int16_t cam_dx = 0;
int16_t cam_dy = 0;
volatile uint8_t cam_connected = 0;
volatile uint32_t cam_rx_count = 0;     // 接收次数计数
volatile uint32_t cam_parse_ok = 0;     // 解析成功次数
volatile uint32_t cam_last_track_ms = 0; // 最近一次识别到目标的时间

// 云台状态
float gimbal_x_angle = 0.0f;    // X轴当前角度
float gimbal_y_angle = 0.0f;    // Y轴当前角度
float gimbal_x_target = 0.0f;   // X轴目标角度
float gimbal_y_target = 0.0f;   // Y轴目标角度
float gimbal_x_speed_target = 0.0f;  // 位置环输出的目标速度
float gimbal_y_speed_target = 0.0f;  // 位置环输出的目标速度
float gimbal_search_center = 0.0f;    // 搜索摆扫中心
int8_t gimbal_search_dir = 1;         // 搜索方向
uint32_t gimbal_boot_ready_ms = 0;    // 允许开始搜索的时间
GimbalState_t gimbal_state = GIMBAL_STATE_BOOT_HOLD;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Gimbal_ProcessCamData(void);
void HAL_TIM3_PeriodElapsedCallback(void);
void Gimbal_CANTransmitAxes(int16_t x_voltage, int16_t y_voltage);
void Gimbal_UpdateStateAndSearch(void);
void Debug_ParseCommand(uint8_t *cmd, uint16_t len);
void Debug_PrintParams(void);
void FlashParams_Save(void);
void FlashParams_Load(void);
void Gimbal_SetImuYMode(uint8_t enable);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief 简单CRC校验
 */
static uint32_t Flash_CRC32(const uint8_t *data, uint32_t len)
{
  uint32_t crc = 0xFFFFFFFF;
  for (uint32_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320;
      else crc >>= 1;
    }
  }
  return ~crc;
}

/**
 * @brief 处理一帧BNO085 DMA接收数据
 */
static void BNO085_ServiceRx(void)
{
  BNO085_Service();
}

/**
 * @brief 等待BNO085输出首帧有效姿态数据
 */
static uint8_t BNO085_WaitForFirstData(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();

  while ((HAL_GetTick() - start) < timeout_ms) {
    BNO085_ServiceRx();
    if (bno085_data.data_ready) {
      return 1;
    }
    HAL_Delay(1);
  }

  return 0;
}

/**
 * @brief 保存参数到Flash Sector7
 */
void FlashParams_Save(void)
{
  FlashParams params;
  memset(&params, 0, sizeof(params));
  params.magic = FLASH_SAVE_MAGIC;
  params.version = FLASH_SAVE_VERSION;
  params.x_kp = gimbal_x_kp;
  params.x_ki = gimbal_x_ki;
  params.x_kd = gimbal_x_kd;
  params.x_speed_kp = gimbal_x_speed_kp;
  params.x_speed_ki = gimbal_x_speed_ki;
  params.x_speed_kd = gimbal_x_speed_kd;
  params.y_kp = gimbal_y_kp;
  params.y_ki = gimbal_y_ki;
  params.y_kd = gimbal_y_kd;
  params.y_target_kp = gimbal_y_target_kp;
  params.boost_threshold = gimbal_boost_threshold;
  params.boost_gain = gimbal_boost_gain;
  params.boost_max = gimbal_boost_max;
  params.boost_px_threshold = gimbal_boost_px_threshold;
  params.x_delta_max_boost = gimbal_x_delta_max_boost;
  params.poserr_guard_boost = gimbal_poserr_guard_boost;
  params.k_gyro = gimbal_k_gyro;
  params.imu_yaw_offset = gimbal_imu_yaw_offset;
  params.imu_roll_offset = gimbal_imu_roll_offset;
  params.use_imu_pos = gimbal_use_imu_pos;
  // 计算CRC（不含crc字段本身）
  params.crc = Flash_CRC32((uint8_t *)&params, sizeof(params) - 4);

  __disable_irq();  // 关中断，防止Flash擦写期间取指冲突
  HAL_FLASH_Unlock();
  // 擦除Sector7
  FLASH_EraseInitTypeDef erase = {0};
  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Sector = FLASH_SECTOR_7;
  erase.NbSectors = 1;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  uint32_t page_err;
  if (HAL_FLASHEx_Erase(&erase, &page_err) != HAL_OK) {
    HAL_UART_Transmit(&huart1, (uint8_t *)"Flash擦除失败!\r\n", 16, 100);
    HAL_FLASH_Lock();
    __enable_irq();
    return;
  }
  // 按32位写入
  uint32_t *src = (uint32_t *)&params;
  uint32_t addr = FLASH_SAVE_ADDR;
  for (uint32_t i = 0; i < (sizeof(params) + 3) / 4; i++) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, src[i]) != HAL_OK) {
      HAL_UART_Transmit(&huart1, (uint8_t *)"Flash写入失败!\r\n", 16, 100);
      HAL_FLASH_Lock();
      __enable_irq();
      return;
    }
    addr += 4;
  }
  HAL_FLASH_Lock();
  __enable_irq();
  HAL_UART_Transmit(&huart1, (uint8_t *)"参数已保存到Flash ✓\r\n", 21, 100);
}

/**
 * @brief 从Flash加载参数，无有效数据则用默认值
 */
void FlashParams_Load(void)
{
  FlashParams *params = (FlashParams *)FLASH_SAVE_ADDR;

  if (params->magic != FLASH_SAVE_MAGIC || params->version != FLASH_SAVE_VERSION) {
    HAL_UART_Transmit(&huart1, (uint8_t *)"无保存参数，使用默认值\r\n", 22, 100);
    return;
  }

  uint32_t crc = Flash_CRC32((uint8_t *)params, sizeof(FlashParams) - 4);
  if (crc != params->crc) {
    HAL_UART_Transmit(&huart1, (uint8_t *)"Flash参数CRC错误，使用默认值\r\n", 26, 100);
    return;
  }

  gimbal_x_kp = params->x_kp;
  gimbal_x_ki = params->x_ki;
  gimbal_x_kd = params->x_kd;
  gimbal_x_speed_kp = params->x_speed_kp;
  gimbal_x_speed_ki = params->x_speed_ki;
  gimbal_x_speed_kd = params->x_speed_kd;
  gimbal_y_kp = params->y_kp;
  gimbal_y_ki = params->y_ki;
  gimbal_y_kd = params->y_kd;
  gimbal_y_target_kp = params->y_target_kp;
  gimbal_boost_threshold = params->boost_threshold;
  gimbal_boost_gain = params->boost_gain;
  gimbal_boost_max = params->boost_max;
  gimbal_boost_px_threshold = params->boost_px_threshold;
  gimbal_x_delta_max_boost = params->x_delta_max_boost;
  gimbal_poserr_guard_boost = params->poserr_guard_boost;
  gimbal_k_gyro = params->k_gyro;
  gimbal_imu_yaw_offset = params->imu_yaw_offset;
  gimbal_imu_roll_offset = params->imu_roll_offset;
  // 开机始终先用编码器搜索目标，避免IMU相对零点和编码器多圈目标混用。
  gimbal_use_imu_pos = GIMBAL_USE_IMU_POS_DEFAULT;

  HAL_UART_Transmit(&huart1, (uint8_t *)"已从Flash加载参数 ✓\r\n", 20, 100);
}

/**
 * @brief 浮点转字符串，避免部分newlib-nano配置下printf不支持%f
 */
static void ftoa(float val, char *out, int decimals)
{
  if (val < 0.0f) {
    *out++ = '-';
    val = -val;
  }

  int ipart = (int)val;
  float fpart = val - (float)ipart;
  char tmp[16];
  int i = 0;

  if (ipart == 0) {
    tmp[i++] = '0';
  } else {
    while (ipart > 0 && i < (int)sizeof(tmp)) {
      tmp[i++] = (char)('0' + (ipart % 10));
      ipart /= 10;
    }
  }

  while (--i >= 0) {
    *out++ = tmp[i];
  }

  if (decimals > 0) {
    *out++ = '.';
    while (decimals-- > 0) {
      fpart *= 10.0f;
      *out++ = (char)('0' + ((int)fpart % 10));
    }
  }

  *out = '\0';
}

/**
 * @brief 打印当前调参参数
 */
void Debug_PrintParams(void)
{
  char buf[500];
  char xkp[12], xki[12], xkd[12];
  char xsp[12], xsi[12], xsd[12];
  char ykp[12], yki[12], ykd[12], ytp[12];
  char bt[12], bg[12], bm[12], bp[12], bd[12], be[12];
  char kg[12], io[12], ir[12];

  ftoa(gimbal_x_kp, xkp, 3);
  ftoa(gimbal_x_ki, xki, 3);
  ftoa(gimbal_x_kd, xkd, 3);
  ftoa(gimbal_x_speed_kp, xsp, 3);
  ftoa(gimbal_x_speed_ki, xsi, 3);
  ftoa(gimbal_x_speed_kd, xsd, 3);
  ftoa(gimbal_y_kp, ykp, 3);
  ftoa(gimbal_y_ki, yki, 3);
  ftoa(gimbal_y_kd, ykd, 3);
  ftoa(gimbal_y_target_kp, ytp, 3);
  ftoa(gimbal_boost_threshold, bt, 3);
  ftoa(gimbal_boost_gain, bg, 3);
  ftoa(gimbal_boost_max, bm, 3);
  ftoa(gimbal_boost_px_threshold, bp, 1);
  ftoa(gimbal_x_delta_max_boost, bd, 3);
  ftoa(gimbal_poserr_guard_boost, be, 3);
  ftoa(gimbal_k_gyro, kg, 3);
  ftoa(gimbal_imu_yaw_offset, io, 3);
  ftoa(gimbal_imu_roll_offset, ir, 3);

  int len = snprintf(buf, sizeof(buf),
                     "\r\n=== 当前参数 ===\r\n"
                     "X位置: KP=%s KI=%s KD=%s\r\n"
                     "X速度: KP=%s KI=%s KD=%s\r\n"
                     "Y位置: KP=%s KI=%s KD=%s\r\n"
                     "Y目标比例: KP=%s\r\n"
                     "Boost: BT=%s BG=%s BM=%s BP=%s BD=%s BE=%s\r\n"
                     "IMU: KG=%s IO=%s IR=%s IU(Y)=%d\r\n"
                     "================\r\n",
                     xkp, xki, xkd, xsp, xsi, xsd, ykp, yki, ykd, ytp,
                     bt, bg, bm, bp, bd, be, kg, io, ir, gimbal_use_imu_pos);
  if (len > 0) {
    if (len > (int)sizeof(buf)) {
      len = sizeof(buf);
    }
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 200);
  }
}

/**
 * @brief 解析串口调参命令
 */
void Debug_ParseCommand(uint8_t *cmd, uint16_t len)
{
  if (len == 0) {
    return;
  }

  if (strcmp((char *)cmd, "PID") == 0) {
    Debug_PrintParams();
    return;
  }

  if (strcmp((char *)cmd, "SAVE") == 0) {
    FlashParams_Save();
    return;
  }

  float value = 0.0f;
  uint8_t updated_pid = 0;

  if ((cmd[0] == 'X' || cmd[0] == 'x') && len >= 3) {
    if ((cmd[1] == 'S' || cmd[1] == 's') && len >= 4) {
      value = atof((char *)&cmd[3]);
      switch (cmd[2]) {
        case 'p': case 'P': gimbal_x_speed_kp = value; updated_pid = 1; break;
        case 'i': case 'I': gimbal_x_speed_ki = value; updated_pid = 1; break;
        case 'd': case 'D': gimbal_x_speed_kd = value; updated_pid = 1; break;
        default: break;
      }
      if (updated_pid) {
        PID_Init(&pid_x_speed, gimbal_x_speed_kp, gimbal_x_speed_ki, gimbal_x_speed_kd);
        PID_Init(&pid_y_speed, gimbal_x_speed_kp, gimbal_x_speed_ki, gimbal_x_speed_kd);
      }
    } else {
      value = atof((char *)&cmd[2]);
      switch (cmd[1]) {
        case 'p': case 'P': gimbal_x_kp = value; updated_pid = 1; break;
        case 'i': case 'I': gimbal_x_ki = value; updated_pid = 1; break;
        case 'd': case 'D': gimbal_x_kd = value; updated_pid = 1; break;
        default: break;
      }
      if (updated_pid) {
        PID_Init(&pid_x_pos, gimbal_x_kp, gimbal_x_ki, gimbal_x_kd);
      }
    }
  } else if ((cmd[0] == 'Y' || cmd[0] == 'y') && len >= 3) {
    if ((cmd[1] == 'T' || cmd[1] == 't') && len >= 4) {
      value = atof((char *)&cmd[3]);
      if (cmd[2] == 'P' || cmd[2] == 'p') {
        gimbal_y_target_kp = value;
        updated_pid = 1;
      }
    } else {
      value = atof((char *)&cmd[2]);
      switch (cmd[1]) {
        case 'p': case 'P': gimbal_y_kp = value; updated_pid = 1; break;
        case 'i': case 'I': gimbal_y_ki = value; updated_pid = 1; break;
        case 'd': case 'D': gimbal_y_kd = value; updated_pid = 1; break;
        default: break;
      }
      if (updated_pid) {
        PID_Init(&pid_y_pos, gimbal_y_kp, gimbal_y_ki, gimbal_y_kd);
      }
    }
  } else if ((cmd[0] == 'B' || cmd[0] == 'b') && len >= 3) {
    value = atof((char *)&cmd[2]);
    switch (cmd[1]) {
      case 't': case 'T': gimbal_boost_threshold = value; updated_pid = 1; break;
      case 'g': case 'G': gimbal_boost_gain = value; updated_pid = 1; break;
      case 'm': case 'M': gimbal_boost_max = value; updated_pid = 1; break;
      case 'p': case 'P': gimbal_boost_px_threshold = value; updated_pid = 1; break;
      case 'd': case 'D': gimbal_x_delta_max_boost = value; updated_pid = 1; break;
      case 'e': case 'E': gimbal_poserr_guard_boost = value; updated_pid = 1; break;
      default: break;
    }
  } else if ((cmd[0] == 'K' || cmd[0] == 'k') && len >= 3) {
    if (cmd[1] == 'G' || cmd[1] == 'g') {
      value = atof((char *)&cmd[2]);
      gimbal_k_gyro = value;
      updated_pid = 1;
    }
  } else if ((cmd[0] == 'I' || cmd[0] == 'i') && len >= 3) {
    switch (cmd[1]) {
      case 'o': case 'O':  // Yaw偏移
        value = atof((char *)&cmd[2]);
        gimbal_imu_yaw_offset = value;
        updated_pid = 1;
        break;
      case 'r': case 'R':  // Roll偏移
        value = atof((char *)&cmd[2]);
        gimbal_imu_roll_offset = value;
        updated_pid = 1;
        break;
      case 'u': case 'U':  // Y轴使用IMU Roll位置环
        value = atof((char *)&cmd[2]);
        Gimbal_SetImuYMode((uint8_t)value);
        updated_pid = 1;
        break;
    }
  }

  if (updated_pid) {
    char buf[64];
    char vstr[12];
    ftoa(value, vstr, 3);
    int l = snprintf(buf, sizeof(buf), "已设置: %s = %s\r\n", (char *)cmd, vstr);
    if (l > 0) {
      if (l > (int)sizeof(buf)) {
        l = sizeof(buf);
      }
      HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)l, 100);
    }
    Debug_PrintParams();
  } else {
    HAL_UART_Transmit(&huart1, (uint8_t *)"未知命令\r\n", strlen("未知命令\r\n"), 100);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_CAN1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // 从Flash加载上次保存的参数（必须在PID_Init之前）
  FlashParams_Load();

  // 初始化GM6020电机PID（使用动态参数）
  PID_Init(&pid_x_speed, gimbal_x_speed_kp, gimbal_x_speed_ki, gimbal_x_speed_kd);  // X速度环
  PID_Init(&pid_x_pos, gimbal_x_kp, gimbal_x_ki, gimbal_x_kd);  // X位置环
  PID_Init(&pid_y_speed, gimbal_x_speed_kp, gimbal_x_speed_ki, gimbal_x_speed_kd);  // Y速度环，与X轴相同参数
  PID_Init(&pid_y_pos, gimbal_y_kp, gimbal_y_ki, gimbal_y_kd);  // Y位置环

  // 启动USART1中断接收（用于串口调参）
  if (HAL_UART_Receive_IT(&huart1, &dbg_rx_buf[0], 1) != HAL_OK) {
    HAL_UART_Transmit(&huart1, (uint8_t *)"USART1 RX启动失败!\r\n", 19, 100);
  }
 
  // 打印调参说明
  HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n=== 串口调参命令 ===\r\n", 25, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"XP1.5  - X轴位置环KP\r\n", 23, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"XI0.1  - X轴位置环KI\r\n", 23, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"XD0.05 - X轴位置环KD\r\n", 24, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"XSP50  - X轴速度环KP\r\n", 23, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"XSI2.0 - X轴速度环KI\r\n", 23, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"XSD6.0 - X轴速度环KD\r\n", 23, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"YP0.5  - Y轴位置环KP\r\n", 23, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"YI0.0  - Y轴位置环KI\r\n", 23, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"YD0.0  - Y轴位置环KD\r\n", 23, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"YTP0.03- Y轴视觉目标比例\r\n", 27, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"BT10   - boost阈值(°)\r\n", 23, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"BG0.05 - boost增益\r\n", 19, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"BM3.0  - boost上限\r\n", 19, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"BP120  - boost像素阈值\r\n", 21, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"BD25   - boost时x_delta上限(°)\r\n", 30, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"BE180  - boost时pos_err门限(°)\r\n", 30, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"KG0.5  - IMU阻尼系数\r\n", 21, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"IO0.0  - Yaw开机偏移(自动Tare)\r\n", 31, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"IR0.0  - Roll开机偏移(自动Tare)\r\n", 32, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"IU0    - Y轴位置环(0=编码器,1=IMU Roll)\r\n", 42, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"PID    - 查询参数\r\n", 19, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"SAVE   - 保存参数到Flash\r\n", 23, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"====================\r\n\r\n", 25, 100);

  Configure_Filter();  //CAN过滤器
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);  // 10ms定时器，触发控制循环

  // 启动maixcam2串口DMA接收（IDLE空闲中断方式）
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, cam_rx_buf, CAM_RX_BUF_SIZE);

  // 初始化BNO085 IMU（UART-SHTP模式）
  BNO085_Init();
  HAL_UART_Transmit(&huart1, (uint8_t *)"BNO085 IMU初始化完成\r\n", 21, 100);

  // 开机自动Tare：记录当前IMU姿态作为零点
  if (!BNO085_WaitForFirstData(300)) {
    HAL_UART_Transmit(&huart1, (uint8_t *)"BNO085首帧等待超时\r\n", strlen("BNO085首帧等待超时\r\n"), 100);
  }
  gimbal_imu_yaw_offset = bno085_data.rotation.yaw;
  gimbal_imu_roll_offset = bno085_data.rotation.roll;
  {
    char tare_buf[64];
    int tare_len = snprintf(tare_buf, sizeof(tare_buf),
                            "IMU Tare: Yaw=%.1f Roll=%.1f\r\n",
                            gimbal_imu_yaw_offset, gimbal_imu_roll_offset);
    if (tare_len > 0) {
      HAL_UART_Transmit(&huart1, (uint8_t *)tare_buf, (uint16_t)tare_len, 100);
    }
  }

  // 等待电机反馈数据稳定
  HAL_Delay(500);

  // 不归零，保持电机当前位置
  gimbal_x_target = motor_date[GIMBAL_X_MOTOR_INDEX].total_du;
  gimbal_x_angle = motor_date[GIMBAL_X_MOTOR_INDEX].total_du;
  gimbal_y_target = motor_date[GIMBAL_Y_MOTOR_INDEX].total_du;
  gimbal_y_angle = motor_date[GIMBAL_Y_MOTOR_INDEX].total_du;
  gimbal_search_center = gimbal_x_target;
  gimbal_search_dir = 1;
  gimbal_boot_ready_ms = time_ms_ms + GIMBAL_BOOT_HOLD_MS;
  gimbal_state = GIMBAL_STATE_BOOT_HOLD;

  HAL_UART_Transmit(&huart1, (uint8_t *)"GM6020 X/Y保持当前位置\r\n", strlen("GM6020 X/Y保持当前位置\r\n"), 100);

  HAL_UART_Transmit(&huart1, (uint8_t *)"云台初始化完成\r\n", strlen("云台初始化完成\r\n"), 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 处理maixcam2 DMA数据
    if (cam_frame_ready) {
      cam_frame_ready = 0;
      cam_rx_count++;

      // 在DMA缓冲区中搜索帧头 0xAA 0xBB
      uint16_t i = 0;
      while (i + 7 < cam_rx_len) {  // 需要至少8字节：header(2)+cmd(1)+dx(2)+dy(2)+checksum(1)
        if (cam_rx_buf[i] == CAM_HEADER1 && cam_rx_buf[i+1] == CAM_HEADER2) {
          // 校验和：cmd + dx(2) + dy(2)
          uint8_t checksum = (cam_rx_buf[i+2] + cam_rx_buf[i+3]
                            + cam_rx_buf[i+4] + cam_rx_buf[i+5]
                            + cam_rx_buf[i+6]) & 0xFF;
          if (checksum == cam_rx_buf[i+7]) {
            cam_cmd = cam_rx_buf[i+2];
            cam_dx = (int16_t)(cam_rx_buf[i+3] | (cam_rx_buf[i+4] << 8));
            cam_dy = (int16_t)(cam_rx_buf[i+5] | (cam_rx_buf[i+6] << 8));
            cam_connected = 1;
            cam_parse_ok++;
            if (cam_cmd == CAM_CMD_TRACK) {
              cam_last_track_ms = time_ms_ms;
            }

            // 处理视觉数据，更新目标
            Gimbal_ProcessCamData();
            break;  // 只解析最新一帧
          }
        }
        i++;
      }
    }

    // 处理BNO085 DMA数据
    BNO085_ServiceRx();

    // 临时测试：每100ms打印一次IMU数据
    static uint32_t last_imu_print = 0;
    if (time_ms_ms - last_imu_print >= 100) {
      last_imu_print = time_ms_ms;

      char imu_buf[128];
      int len = snprintf(imu_buf, sizeof(imu_buf),
                         "IMU: yaw=%6.1f pitch=%6.1f roll=%6.1f gx=%6.2f gy=%6.2f gz=%6.2f\r\n",
                         bno085_data.rotation.yaw,
                         bno085_data.rotation.pitch,
                         bno085_data.rotation.roll,
                         bno085_data.gyro.x,
                         bno085_data.gyro.y,
                         bno085_data.gyro.z);
      if (len > 0) {
        HAL_UART_Transmit(&huart1, (uint8_t *)imu_buf, (uint16_t)len, 100);
      }
    }

    // 处理串口调参命令
    if (dbg_cmd_ready) {
      dbg_cmd_ready = 0;
      Debug_ParseCommand(dbg_rx_buf, dbg_cmd_len);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief 处理maixcam2数据，更新云台目标
 */
void Gimbal_ProcessCamData(void)
{
  if (cam_cmd == CAM_CMD_TRACK) {
    float pos_err = fabsf(gimbal_x_target - motor_date[GIMBAL_X_MOTOR_INDEX].total_du);
    float cam_dx_abs = (float)abs(cam_dx);
    uint8_t in_boost = (cam_dx_abs > gimbal_boost_px_threshold) ? 1 : 0;

    // 跨上下文boost标志（位置PID积分清零用）
    gimbal_x_boost_active = in_boost;

    // X轴：动态死区和动态增量限幅
    if (abs(cam_dx) > GIMBAL_DEADZONE) {
      // boost模式下放宽pos_err门限，防止"跟不上→停止更新"恶性循环
      float guard = in_boost ? gimbal_poserr_guard_boost : 30.0f;

      if (pos_err < guard) {
        float x_delta = cam_dx * gimbal_x_kp;

        if (in_boost) {
          // 线性缩放：阈值处=5°，cam_dx=400px处=配置上限
          float scale = (cam_dx_abs - gimbal_boost_px_threshold)
                      / (400.0f - gimbal_boost_px_threshold);
          if (scale > 1.0f) scale = 1.0f;
          if (scale < 0.0f) scale = 0.0f;
          float dyn_max = 5.0f + scale * (gimbal_x_delta_max_boost - 5.0f);

          if (x_delta > dyn_max)  x_delta = dyn_max;
          if (x_delta < -dyn_max) x_delta = -dyn_max;
        } else {
          // 正常模式：保持原始5°硬钳位
          if (x_delta > 5.0f)  x_delta = 5.0f;
          if (x_delta < -5.0f) x_delta = -5.0f;
        }

        gimbal_x_target -= x_delta;
      }
    }

    // Y轴：视觉偏差更新GM6020目标角度
    if (abs(cam_dy) > GIMBAL_DEADZONE) {
      float y_delta = cam_dy * gimbal_y_target_kp;
      if (y_delta > GIMBAL_Y_DELTA_MAX) y_delta = GIMBAL_Y_DELTA_MAX;
      if (y_delta < -GIMBAL_Y_DELTA_MAX) y_delta = -GIMBAL_Y_DELTA_MAX;
      gimbal_y_target += y_delta;

      if (gimbal_y_target > GIMBAL_Y_MAX) gimbal_y_target = GIMBAL_Y_MAX;
      if (gimbal_y_target < GIMBAL_Y_MIN) gimbal_y_target = GIMBAL_Y_MIN;
    }
  } else {
    gimbal_x_boost_active = 0;
  }
}

/**
 * @brief 根据轴到电机ID映射发送GM6020控制量
 */
void Gimbal_CANTransmitAxes(int16_t x_voltage, int16_t y_voltage)
{
  int16_t can_data[4] = {0, 0, 0, 0};

  can_data[GIMBAL_X_MOTOR_INDEX] = x_voltage;
  can_data[GIMBAL_Y_MOTOR_INDEX] = y_voltage;

  CAN_Transmit(can_data[0], can_data[1], can_data[2], can_data[3], Voltage);
}

/**
 * @brief 更新云台搜索/追踪状态，未锁定目标时执行水平摆扫
 */
void Gimbal_UpdateStateAndSearch(void)
{
  uint8_t target_valid = (cam_connected &&
                          cam_cmd == CAM_CMD_TRACK &&
                          (time_ms_ms - cam_last_track_ms) <= GIMBAL_TRACK_TIMEOUT_MS);

  if (target_valid) {
    if (gimbal_state != GIMBAL_STATE_TRACK) {
      gimbal_search_center = gimbal_x_target;
      pid_x_pos.err_sum = 0.0f;
      gimbal_x_boost_active = 0;
    }
    gimbal_state = GIMBAL_STATE_TRACK;
    return;
  }

  if (gimbal_state == GIMBAL_STATE_TRACK) {
    gimbal_state = GIMBAL_STATE_LOST_HOLD;
    return;
  }

  if (gimbal_state == GIMBAL_STATE_LOST_HOLD) {
    if ((time_ms_ms - cam_last_track_ms) <= GIMBAL_TRACK_TIMEOUT_MS) {
      return;
    }
    gimbal_search_center = gimbal_x_angle;
    gimbal_search_dir = 1;
    pid_x_pos.err_sum = 0.0f;
    gimbal_x_boost_active = 0;
    gimbal_state = GIMBAL_STATE_SEARCH;
  }

  if (gimbal_state == GIMBAL_STATE_BOOT_HOLD) {
    if (time_ms_ms < gimbal_boot_ready_ms) {
      return;
    }
    gimbal_search_center = gimbal_x_angle;
    gimbal_search_dir = 1;
    pid_x_pos.err_sum = 0.0f;
    gimbal_x_boost_active = 0;
    gimbal_state = GIMBAL_STATE_SEARCH;
  }

  if (gimbal_state == GIMBAL_STATE_SEARCH) {
    gimbal_x_target += (float)gimbal_search_dir * GIMBAL_SEARCH_STEP_DEG;

    if (gimbal_x_target > gimbal_search_center + GIMBAL_SEARCH_RANGE_DEG) {
      gimbal_x_target = gimbal_search_center + GIMBAL_SEARCH_RANGE_DEG;
      gimbal_search_dir = -1;
    } else if (gimbal_x_target < gimbal_search_center - GIMBAL_SEARCH_RANGE_DEG) {
      gimbal_x_target = gimbal_search_center - GIMBAL_SEARCH_RANGE_DEG;
      gimbal_search_dir = 1;
    }
  }
}

/**
 * @brief 切换Y轴位置环反馈来源，切换时重建目标坐标避免冲击
 */
void Gimbal_SetImuYMode(uint8_t enable)
{
  enable = enable ? 1U : 0U;

  if (enable == gimbal_use_imu_pos) {
    return;
  }

  pid_y_pos.err_sum = 0.0f;
  pid_y_pos.err_last = 0.0f;
  gimbal_y_speed_target = 0.0f;

  if (enable) {
    gimbal_imu_roll_offset = bno085_data.rotation.roll;
    gimbal_y_target = 0.0f;
  } else {
    gimbal_y_target = motor_date[GIMBAL_Y_MOTOR_INDEX].total_du;
  }

  gimbal_use_imu_pos = enable;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) //1ms定时器，触发速度环和位置环
  {
    time_ms++;
    time_ms_ms++;


    // 速度环：每2ms执行一次
    if (time_ms % 2 == 0) {
      // IMU阻尼项（在速度环目标上叠加）
      // BNO085角速度单位为rad/s，位置环输出单位为deg/s。
      float x_damping_dps = gimbal_k_gyro * bno085_data.gyro.z * RADS_TO_DEGS;
      float y_damping_dps = gimbal_k_gyro * bno085_data.gyro.x * RADS_TO_DEGS;
      float x_speed_cmd_dps = gimbal_x_speed_target - x_damping_dps;
      float y_speed_cmd_dps = gimbal_y_speed_target - y_damping_dps;

      // 速度环使用RPM单位，与参考工程保持一致
      float x_speed_target_rpm = x_speed_cmd_dps / RPM_TO_DEGS;
      float y_speed_target_rpm = y_speed_cmd_dps / RPM_TO_DEGS;
      int16_t x_voltage = (int16_t)PID_SpeedLoop(&pid_x_speed,
                                                  (float)motor_date[GIMBAL_X_MOTOR_INDEX].rotor_speed,
                                                  x_speed_target_rpm);
      int16_t y_voltage = (int16_t)PID_SpeedLoop(&pid_y_speed,
                                                  (float)motor_date[GIMBAL_Y_MOTOR_INDEX].rotor_speed,
                                                  y_speed_target_rpm);

      // 限幅
      if (x_voltage > 25000) x_voltage = 25000;
      if (x_voltage < -25000) x_voltage = -25000;
      if (y_voltage > 25000) y_voltage = 25000;
      if (y_voltage < -25000) y_voltage = -25000;

      Gimbal_CANTransmitAxes(x_voltage, y_voltage);
    }

    // 位置环：每10ms执行一次
    if (time_ms >= 10) {
      // 位置环积分防饱和：boost状态切换时清零
      static uint8_t pid_last_boost = 0;
      if (gimbal_x_boost_active != pid_last_boost) {
        pid_x_pos.err_sum = 0.0f;
        pid_last_boost = gimbal_x_boost_active;
      }

      Gimbal_UpdateStateAndSearch();

      // X轴无限旋转，始终使用GM6020编码器多圈角度闭环。
      gimbal_x_speed_target = PID_PositionLoop(&pid_x_pos,
                                                motor_date[GIMBAL_X_MOTOR_INDEX].total_du,
                                                gimbal_x_target);

      // Y轴可选择编码器角度或IMU Roll姿态外环。
      if (gimbal_use_imu_pos) {
        float imu_roll_relative = ShortestPath_Error(bno085_data.rotation.roll,
                                                     gimbal_imu_roll_offset);
        gimbal_y_speed_target = PID_PositionLoop_Angle(&pid_y_pos,
                                                        imu_roll_relative,
                                                        gimbal_y_target);
      } else {
        gimbal_y_speed_target = PID_PositionLoop(&pid_y_pos,
                                                  motor_date[GIMBAL_Y_MOTOR_INDEX].total_du,
                                                  gimbal_y_target);
      }
      // boost：像素偏移大时加速追赶（仅追踪模式）
      float cam_dx_abs = (float)(abs(cam_dx));
      if (cam_cmd == CAM_CMD_TRACK && cam_dx_abs > gimbal_boost_px_threshold) {
        float boost = 1.0f + (cam_dx_abs - gimbal_boost_px_threshold) * gimbal_boost_gain;
        if (boost > gimbal_boost_max) boost = gimbal_boost_max;
        gimbal_x_speed_target *= boost;
      }

      // 更新当前角度
      gimbal_x_angle = motor_date[GIMBAL_X_MOTOR_INDEX].total_du;
      gimbal_y_angle = motor_date[GIMBAL_Y_MOTOR_INDEX].total_du;

      time_ms = 0;
    }
  }
  else if (htim->Instance == TIM3) //10ms定时器，触发（）任务
  {
    HAL_TIM3_PeriodElapsedCallback();
  }
}

/**
 * @brief TIM3中断回调（10ms周期）
 */
void HAL_TIM3_PeriodElapsedCallback(void)
{
  // 此处可用于后续扩展：定时触发控制任务
  // 目前视觉数据处理已在主循环中通过cam_frame_ready触发
}

/**
 * @brief DMA空闲中断回调 - USART2/USART3接收到完整帧
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART3)
  {
    if (Size > 0)
    {
      cam_rx_len = Size;
      cam_frame_ready = 1;
    }
    // 重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, cam_rx_buf, CAM_RX_BUF_SIZE);
  }
  else if (huart->Instance == USART2)
  {
    if (Size > 0)
    {
      bno085_rx_len = Size;
      BNO085_PushRxData(bno085_rx_buf, Size);
    }
    // 重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, bno085_rx_buf, 256);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
