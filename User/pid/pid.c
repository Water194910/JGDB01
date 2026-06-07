#include "pid.h"

void PID_Init(PID_Handle_t pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->actual_val = 0;
    pid->target_val = 0;
    pid->err = 0;
    pid->err_last = 0;
    pid->err_sum = 0;
}

float PID_SpeedLoop(PID_Handle_t pid, float actual_val, float target_val)
{
    pid->target_val = target_val;
    pid->actual_val = actual_val;
    pid->err = pid->target_val - pid->actual_val;
    pid->err_sum += pid->err;

    float err_out = pid->Kp * pid->err
                  + pid->Ki * pid->err_sum
                  + pid->Kd * (pid->err - pid->err_last);
    pid->err_last = pid->err;

    return PID_LIMIT_MIN_MAX(err_out, PID_VOLTAGE_MIN, PID_VOLTAGE_MAX);
}

float PID_PositionLoop(PID_Handle_t pid, float actual_val, float target_val)
{
    pid->target_val = target_val;
    pid->actual_val = actual_val;
    pid->err = pid->target_val - pid->actual_val;
    pid->err_sum += pid->err;

    float err_out = pid->Kp * pid->err
                  + pid->Ki * pid->err_sum
                  + pid->Kd * (pid->err - pid->err_last);
    pid->err_last = pid->err;

    return PID_LIMIT_MIN_MAX(err_out, -PID_SPEED_MAX_DEG, PID_SPEED_MAX_DEG);
}

float ShortestPath_Error(float target, float current)
{
    float error = target - current;

    if (error > 180.0f) {
        error -= 360.0f;
    } else if (error < -180.0f) {
        error += 360.0f;
    }

    return error;
}

float PID_PositionLoop_Angle(PID_Handle_t pid, float actual_val, float target_val)
{
    pid->target_val = target_val;
    pid->actual_val = actual_val;
    pid->err = ShortestPath_Error(target_val, actual_val);  // 使用最短路径误差
    pid->err_sum += pid->err;

    float err_out = pid->Kp * pid->err
                  + pid->Ki * pid->err_sum
                  + pid->Kd * (pid->err - pid->err_last);
    pid->err_last = pid->err;

    return PID_LIMIT_MIN_MAX(err_out, -PID_SPEED_MAX_DEG, PID_SPEED_MAX_DEG);
}
