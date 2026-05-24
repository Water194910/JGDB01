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
#include "user_uart.h"
#include "fashion_star_uart_servo.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "gm6020_can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
#define GIMBAL_X_SPEED_KP_DEFAULT 3.0f   // X轴速度环KP
#define GIMBAL_X_SPEED_KI_DEFAULT 1.2f   // X轴速度环KI
#define GIMBAL_X_SPEED_KD_DEFAULT 0.0f   // X轴速度环KD
#define GIMBAL_Y_KP_DEFAULT     0.03f    // Y轴KP
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
#define SERVO_ID                0        // Y轴舵机ID
#define SERVO_INTERVAL          50       // 舵机运动时间ms
#define SERVO_POWER             1000     // 舵机功率

// 串口调参缓冲区
// Flash参数存储
#define FLASH_SAVE_ADDR         0x08060000   // Sector7起始地址
#define FLASH_SAVE_MAGIC        0x594E494C   // "LINY" 灵犀的校验头
#define FLASH_SAVE_VERSION      1

typedef struct {
  uint32_t magic;
  uint32_t version;
  float x_kp, x_ki, x_kd;
  float x_speed_kp, x_speed_ki, x_speed_kd;
  float y_kp;
  float boost_threshold, boost_gain, boost_max;
  float boost_px_threshold;
  float x_delta_max_boost;
  float poserr_guard_boost;
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
// GM6020 X轴
pid pid_sd, pid_wz;
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
float gimbal_boost_threshold = GIMBAL_BOOST_THRESHOLD;
float gimbal_boost_gain = GIMBAL_BOOST_GAIN;
float gimbal_boost_max = GIMBAL_BOOST_MAX;
float gimbal_boost_px_threshold = GIMBAL_BOOST_PX_THRESHOLD;
float gimbal_x_delta_max_boost = GIMBAL_X_DELTA_MAX_BOOST;
float gimbal_poserr_guard_boost = GIMBAL_POSERR_GUARD_BOOST;
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

// 云台状态
float gimbal_x_angle = 0.0f;    // X轴当前角度
float gimbal_y_angle = 0.0f;    // Y轴当前角度
float gimbal_x_target = 0.0f;   // X轴目标角度
float gimbal_y_target = 0.0f;   // Y轴目标角度
float gimbal_x_err_sum = 0.0f;  // X轴位置误差积分
float gimbal_y_err_sum = 0.0f;  // Y轴误差积分
float gimbal_x_err_last = 0.0f; // X轴位置上次误差
float gimbal_y_err_last = 0.0f; // Y轴上次误差
float gimbal_x_speed_target = 0.0f;  // 位置环输出的目标速度
volatile uint8_t gimbal_speed_test_mode = 0;  // 速度环测试模式标志
float gimbal_speed_test_val = 0.0f;  // 测试目标转速（度/秒）

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Gimbal_ProcessCamData(void);
float Gimbal_CalcPID(float kp, float ki, float kd, float error, float *err_sum, float *err_last);
void HAL_TIM3_PeriodElapsedCallback(void);
void Debug_ParseCommand(uint8_t *cmd, uint16_t len);
void Debug_PrintParams(void);
void FlashParams_Save(void);
void FlashParams_Load(void);
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
  params.boost_threshold = gimbal_boost_threshold;
  params.boost_gain = gimbal_boost_gain;
  params.boost_max = gimbal_boost_max;
  params.boost_px_threshold = gimbal_boost_px_threshold;
  params.x_delta_max_boost = gimbal_x_delta_max_boost;
  params.poserr_guard_boost = gimbal_poserr_guard_boost;
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
  gimbal_boost_threshold = params->boost_threshold;
  gimbal_boost_gain = params->boost_gain;
  gimbal_boost_max = params->boost_max;
  gimbal_boost_px_threshold = params->boost_px_threshold;
  gimbal_x_delta_max_boost = params->x_delta_max_boost;
  gimbal_poserr_guard_boost = params->poserr_guard_boost;

  HAL_UART_Transmit(&huart1, (uint8_t *)"已从Flash加载参数 ✓\r\n", 20, 100);
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_CAN1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // 初始化舵机串口
  User_Uart_Init(&huart2);
  HAL_UART_Transmit(&huart1, (uint8_t *)"USART2 init OK\r\n", 16, 100);

  // 舵机上电初始化（必须！等待1秒→卸力→重置圈数）
  FSUS_INIT();
  HAL_UART_Transmit(&huart1, (uint8_t *)"舵机初始化完成\r\n", strlen("舵机初始化完成\r\n"), 100);

  // 从Flash加载上次保存的参数（必须在Pid_Init之前）
  FlashParams_Load();

  // 初始化GM6020电机PID（使用动态参数）
  Pid_Init(&pid_sd, gimbal_x_speed_kp, gimbal_x_speed_ki, gimbal_x_speed_kd);  //速度环
  Pid_Init(&pid_wz, gimbal_x_kp, gimbal_x_ki, gimbal_x_kd);  //位置环

  // 启动USART1中断接收（用于串口调参）
  if (HAL_UART_Receive_IT(&huart1, &dbg_rx_buf[0], 1) != HAL_OK) {
    HAL_UART_Transmit(&huart1, (uint8_t *)"USART1 RX启动失败!\r\n", 19, 100);
  }
 
  // 打印调参说明
  HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n=== 串口调参命令 ===\r\n", 25, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"XP1.5  - X轴位置环KP\r\n", 23, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"XI0.1  - X轴位置环KI\r\n", 23, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"XD0.05 - X轴位置环KD\r\n", 24, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"XSP3.0 - X轴速度环KP\r\n", 23, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"XSI1.0 - X轴速度环KI\r\n", 23, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"XSD0.0 - X轴速度环KD\r\n", 23, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"YP0.03 - Y轴KP\r\n", 17, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"T100   - 测试转速(°/s), T0退出\r\n", 30, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"BT10   - boost阈值(°)\r\n", 23, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"BG0.05 - boost增益\r\n", 19, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"BM3.0  - boost上限\r\n", 19, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"BP120  - boost像素阈值\r\n", 21, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"BD25   - boost时x_delta上限(°)\r\n", 30, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"BE180  - boost时pos_err门限(°)\r\n", 30, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"PID    - 查询参数\r\n", 19, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"SAVE   - 保存参数到Flash\r\n", 23, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"====================\r\n\r\n", 25, 100);

  Configure_Filter();  //CAN过滤器
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);  // 10ms定时器，触发控制循环

  // 启动maixcam2串口DMA接收（IDLE空闲中断方式）
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, cam_rx_buf, CAM_RX_BUF_SIZE);

  // 等待电机反馈数据稳定
  HAL_Delay(500);

  // 不归零，保持电机当前位置
  gimbal_x_target = motor_date[0].total_du;
  gimbal_x_angle = motor_date[0].total_du;

  HAL_UART_Transmit(&huart1, (uint8_t *)"GM6020保持当前位置\r\n", strlen("GM6020保持当前位置\r\n"), 100);

  // 初始化舵机到中位
  gimbal_y_target = 0.0f;
  FSUS_SetServoAngle(&FSUS_usart2, SERVO_ID, 0.0f, SERVO_INTERVAL, SERVO_POWER);

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

            // 处理视觉数据，更新目标
            Gimbal_ProcessCamData();
            break;  // 只解析最新一帧
          }
        }
        i++;
      }
    }

    // Y轴舵机控制（每100ms更新一次，避免过于频繁）
    static uint16_t servo_cnt = 0;
    static float last_y_cmd = 0.0f;
    servo_cnt++;
    if (servo_cnt >= 10) {  // 10ms * 10 = 100ms
      servo_cnt = 0;

      // 限幅保护（双保险）
      float y_cmd = gimbal_y_target;
      if (y_cmd > GIMBAL_Y_MAX) y_cmd = GIMBAL_Y_MAX;
      if (y_cmd < GIMBAL_Y_MIN) y_cmd = GIMBAL_Y_MIN;

      // 只有当目标变化超过0.5度时才发送指令，减少抖动
      if (fabsf(y_cmd - last_y_cmd) > 0.5f) {
        FSUS_SetServoAngle(&FSUS_usart2, SERVO_ID, y_cmd, SERVO_INTERVAL, SERVO_POWER);
        last_y_cmd = y_cmd;
      }
      gimbal_y_angle = y_cmd;
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
    float pos_err = fabsf(gimbal_x_target - motor_date[0].total_du);
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

    // Y轴：保持不变
    if (abs(cam_dy) > GIMBAL_DEADZONE) {
      float y_delta = cam_dy * gimbal_y_kp;
      if (y_delta > GIMBAL_Y_DELTA_MAX) y_delta = GIMBAL_Y_DELTA_MAX;
      if (y_delta < -GIMBAL_Y_DELTA_MAX) y_delta = -GIMBAL_Y_DELTA_MAX;
      gimbal_y_target += y_delta;

      if (gimbal_y_target > GIMBAL_Y_MAX) gimbal_y_target = GIMBAL_Y_MAX;
      if (gimbal_y_target < GIMBAL_Y_MIN) gimbal_y_target = GIMBAL_Y_MIN;
    }
  }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) //1ms定时器，触发速度环和位置环
  {
    time_ms++;
    time_ms_ms++;


    // 速度环：每2ms执行一次
    if (time_ms % 2 == 0) {
      // 速度环：根据速度计算电压
      float x_speed_error = gimbal_x_speed_target - motor_date[0].rotor_du_speed;
      int16_t x_voltage = (int16_t)Gimbal_CalcPID(pid_sd.Kp, pid_sd.Ki, pid_sd.Kd,
                                                   x_speed_error, &pid_sd.err_sum, &pid_sd.err_last);

      // 限幅
      if (x_voltage > 25000) x_voltage = 25000;
      if (x_voltage < -25000) x_voltage = -25000;

      CAN_Transmit(x_voltage, 0, 0, 0, Voltage);
    }

    // 位置环：每10ms执行一次
    if (time_ms >= 10) {
      if (gimbal_speed_test_mode) {
        // 测试模式：直接使用串口设定的目标转速
        gimbal_x_speed_target = gimbal_speed_test_val;
      } else {
        // 位置环积分防饱和：boost状态切换时清零
        static uint8_t pid_last_boost = 0;
        if (gimbal_x_boost_active != pid_last_boost) {
          gimbal_x_err_sum = 0.0f;
          pid_last_boost = gimbal_x_boost_active;
        }

        // 正常模式：位置环根据角度偏差计算目标转速
        float x_error = gimbal_x_target - motor_date[0].total_du;
        gimbal_x_speed_target = Gimbal_CalcPID(gimbal_x_kp, gimbal_x_ki, gimbal_x_kd,
                                                x_error, &gimbal_x_err_sum, &gimbal_x_err_last);
        // boost：像素偏移大时加速追赶（仅追踪模式）
        float cam_dx_abs = (float)(abs(cam_dx));
        if (cam_cmd == CAM_CMD_TRACK && cam_dx_abs > gimbal_boost_px_threshold) {
          float boost = 1.0f + (cam_dx_abs - gimbal_boost_px_threshold) * gimbal_boost_gain;
          if (boost > gimbal_boost_max) boost = gimbal_boost_max;
          gimbal_x_speed_target *= boost;
        }
      }

      // 更新当前角度
      gimbal_x_angle = motor_date[0].total_du;

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
 * @brief DMA空闲中断回调 - USART3接收到完整帧
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
