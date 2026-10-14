/*
 * ============================================================================
 *  foc_lpf —— 实现（逐行对齐 SimpleFOC src/common/lowpass_filter.cpp）
 * ============================================================================
 */

#include "foc_lpf.h"
#include "foc_time.h"

#define LPF_FALLBACK_DT     1e-3f
#define LPF_RESET_DT        0.3f        /* dt 超过这个值判为断流，直接跟随输入 */

void foc_lpf_init(foc_lpf_t *lpf, float Tf, float Ts)
{
    if (!lpf) {
        return;
    }
    lpf->Tf             = Tf;
    lpf->Ts             = Ts;
    lpf->y_prev         = 0.0f;
    lpf->timestamp_prev = foc_micros();
}

void foc_lpf_reset(foc_lpf_t *lpf)
{
    if (!lpf) {
        return;
    }
    lpf->y_prev = 0.0f;
}

float foc_lpf_run(foc_lpf_t *lpf, float x)
{
    if (!lpf) {
        return x;
    }

    /* ---- 采样时间 ---- */
    float dt = lpf->Ts;
    if (!foc_is_set(dt)) {
        const int64_t now = foc_micros();
        dt = (float)(now - lpf->timestamp_prev) * 1e-6f;

        if (dt < 0.0f) {
            dt = LPF_FALLBACK_DT;
        } else if (dt > LPF_RESET_DT) {
            /* 长时间没调用，滤波器状态已无意义，直接跟随输入 */
            lpf->y_prev         = x;
            lpf->timestamp_prev = now;
            return x;
        }
        lpf->timestamp_prev = now;
    }

    /* 一阶低通 */
    const float alpha = lpf->Tf / (lpf->Tf + dt);
    const float y     = alpha * lpf->y_prev + (1.0f - alpha) * x;

    lpf->y_prev = y;
    return y;
}
