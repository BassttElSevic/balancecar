/*
 * ============================================================================
 *  foc_pid —— 实现（逐行对齐 SimpleFOC src/common/pid.cpp）
 * ============================================================================
 */

#include <math.h>

#include "foc_math.h"
#include "foc_pid.h"
#include "foc_time.h"

#define PID_FALLBACK_DT     1e-3f       /* dt 异常时的回退采样时间 (s) */
#define PID_MAX_DT          0.5f        /* dt 超过这个值判为异常 */

void foc_pid_init(foc_pid_t *pid, float P, float I, float D,
                  float ramp, float limit, float Ts)
{
    if (!pid) {
        return;
    }
    pid->P             = P;
    pid->I             = I;
    pid->D             = D;
    pid->output_ramp   = ramp;
    pid->limit         = limit;
    pid->Ts            = Ts;
    pid->error_prev    = 0.0f;
    pid->output_prev   = 0.0f;
    pid->integral_prev = 0.0f;
    pid->timestamp_prev = foc_micros();
}

void foc_pid_reset(foc_pid_t *pid)
{
    if (!pid) {
        return;
    }
    pid->integral_prev = 0.0f;
    pid->output_prev   = 0.0f;
    pid->error_prev    = 0.0f;
}

void foc_pid_set_gains(foc_pid_t *pid, float P, float I, float D)
{
    if (!pid) {
        return;
    }
    pid->P = P;
    pid->I = I;
    pid->D = D;
}

float foc_pid_run(foc_pid_t *pid, float error)
{
    if (!pid) {
        return 0.0f;
    }

    /* ---- 采样时间：固定则用 Ts，否则量测 ---- */
    float dt = pid->Ts;
    if (!foc_is_set(dt)) {
        const int64_t now = foc_micros();
        dt = (float)(now - pid->timestamp_prev) * 1e-6f;
        if (dt <= 0.0f || dt > PID_MAX_DT) {
            dt = PID_FALLBACK_DT;
        }
        pid->timestamp_prev = now;
    }

    /* u(s) = (P + I/s + Ds) e(s) */

    /* 比例项 */
    const float proportional = pid->P * error;

    /* 积分项：Tustin（梯形）离散化  u_ik = u_ik_1 + I*dt/2*(ek + ek_1) */
    float integral = pid->integral_prev + pid->I * dt * 0.5f * (error + pid->error_prev);
    if (foc_is_set(pid->limit)) {
        integral = foc_clampf(integral, -pid->limit, pid->limit);   /* 抗饱和 */
    }

    /* 微分项：u_dk = D*(ek - ek_1)/dt */
    const float derivative = pid->D * (error - pid->error_prev) / dt;

    /* 求和 + 输出限幅 */
    float output = proportional + integral + derivative;
    if (foc_is_set(pid->limit)) {
        output = foc_clampf(output, -pid->limit, pid->limit);       /* 抗饱和 */
    }

    /* 输出斜坡（限制输出变化率） */
    if (foc_is_set(pid->output_ramp) && pid->output_ramp > 0.0f) {
        const float rate = (output - pid->output_prev) / dt;
        if (rate > pid->output_ramp) {
            output = pid->output_prev + pid->output_ramp * dt;
        } else if (rate < -pid->output_ramp) {
            output = pid->output_prev - pid->output_ramp * dt;
        }
    }

    /* 保存状态供下一次使用 */
    pid->integral_prev = integral;
    pid->output_prev   = output;
    pid->error_prev    = error;
    return output;
}
