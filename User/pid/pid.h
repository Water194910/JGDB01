#ifndef __PID_H
#define __PID_H

#include <stdint.h>

#define PID_LIMIT_MIN_MAX(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define PID_VOLTAGE_MAX     30000
#define PID_VOLTAGE_MIN    -30000
#define PID_SPEED_MAX_DEG   792.0f   // 132RPM * 6deg/s

typedef struct
{
    float Kp, Ki, Kd;
    float actual_val;
    float target_val;
    float err;
    float err_last;
    float err_sum;
} PID_TypeDef;

typedef PID_TypeDef *PID_Handle_t;

typedef struct
{
    PID_TypeDef outer;   // 位置外环
    PID_TypeDef inner;   // 速度内环
    float output;
} PID_Cascade_t;

void PID_Init(PID_Handle_t pid, float kp, float ki, float kd);
float PID_SpeedLoop(PID_Handle_t pid, float actual_val, float target_val);
float PID_PositionLoop(PID_Handle_t pid, float actual_val, float target_val);
float ShortestPath_Error(float target, float current);

#endif
