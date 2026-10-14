/*
 * ============================================================================
 *  foc_math —— 实现
 *
 *  _sin/_cos 的 65 点查找表 + 线性插值、_atan2 的多项式近似，
 *  都直接取自 SimpleFOC src/common/foc_utils.cpp（保持同样的精度与行为）。
 * ============================================================================
 */

#include <math.h>

#include "foc_math.h"

/* -------------------------------------------------------------------------- */
/*  _sin / _cos：65 点查找表 + 线性插值                                        */
/* -------------------------------------------------------------------------- */

/* SimpleFOC 的表：16bit 精度，表长 65（含端点），插值用 8 位小数 */
static const uint16_t s_sine_lut[65] = {
    0,     804,   1608,  2411,  3212,  4011,  4808,  5602,  6393,  7180,
    7962,  8740,  9512,  10279, 11039, 11793, 12540, 13279, 14010, 14733,
    15447, 16151, 16846, 17531, 18205, 18868, 19520, 20160, 20788, 21403,
    22006, 22595, 23170, 23732, 24279, 24812, 25330, 25833, 26320, 26791,
    27246, 27684, 28106, 28511, 28899, 29269, 29622, 29957, 30274, 30572,
    30853, 31114, 31357, 31581, 31786, 31972, 32138, 32286, 32413, 32522,
    32610, 32679, 32729, 32758, 32768
};

/* 查找表取址用的定点定义（与 SimpleFOC 一致）*/
#define SINE_LUT_POINTS     64u          /* 表点数 */
#define SINE_LUT_QUARTERS   4u           /* 一圈分成 4 个象限 */
#define SINE_LUT_FRAC_BITS  8u           /* 插值用的小数位 */
#define SINE_LUT_FRAC_MASK  ((1u << SINE_LUT_FRAC_BITS) - 1u)
#define SINE_LUT_PHASE_MASK 0xFFu        /* 象限内相位索引的掩码 */
#define SINE_LUT_SIZE       (SINE_LUT_POINTS * SINE_LUT_QUARTERS * (1u << SINE_LUT_FRAC_BITS))
#define SINE_LUT_SCALE      ((float)SINE_LUT_SIZE / FOC_2PI)

#define SINE_LUT_AMPL_SCALE 32768.0f    /* 表值的满幅 */

#define SINE_LUT_Q1_END     64u
#define SINE_LUT_Q2_END     128u
#define SINE_LUT_Q3_END     192u
#define SINE_LUT_Q4_END     256u

float foc_sin(float a)
{
    /* 先把负角度折到正，避免无符号转换出问题 */
    if (a < 0.0f) {
        a = -a;
        /* sin(-x) = -sin(x)，用符号位翻转，下面统一取值 */
        const unsigned int i = (unsigned int)(a * SINE_LUT_SCALE);
        const int frac = (int)(i & SINE_LUT_FRAC_MASK);
        unsigned int idx = (i >> SINE_LUT_FRAC_BITS) & SINE_LUT_PHASE_MASK;
        int t1, t2;
        if (idx < SINE_LUT_Q1_END) {
            t1 = (int)s_sine_lut[idx];
            t2 = (int)s_sine_lut[idx + 1u];
        } else if (idx < SINE_LUT_Q2_END) {
            t1 = (int)s_sine_lut[SINE_LUT_Q2_END - idx];
            t2 = (int)s_sine_lut[SINE_LUT_Q2_END - 1u - idx];
        } else if (idx < SINE_LUT_Q3_END) {
            t1 = -(int)s_sine_lut[idx - SINE_LUT_Q2_END];
            t2 = -(int)s_sine_lut[idx - SINE_LUT_Q2_END + 1u];
        } else {
            t1 = -(int)s_sine_lut[SINE_LUT_Q4_END - idx];
            t2 = -(int)s_sine_lut[SINE_LUT_Q4_END - 1u - idx];
        }
        return -(1.0f / SINE_LUT_AMPL_SCALE) * (float)(t1 + (((t2 - t1) * frac) >> SINE_LUT_FRAC_BITS));
    }

    const unsigned int i = (unsigned int)(a * SINE_LUT_SCALE);
    const int frac = (int)(i & SINE_LUT_FRAC_MASK);
    unsigned int idx = (i >> SINE_LUT_FRAC_BITS) & SINE_LUT_PHASE_MASK;
    int t1, t2;
    if (idx < SINE_LUT_Q1_END) {
        t1 = (int)s_sine_lut[idx];
        t2 = (int)s_sine_lut[idx + 1u];
    } else if (idx < SINE_LUT_Q2_END) {
        t1 = (int)s_sine_lut[SINE_LUT_Q2_END - idx];
        t2 = (int)s_sine_lut[SINE_LUT_Q2_END - 1u - idx];
    } else if (idx < SINE_LUT_Q3_END) {
        t1 = -(int)s_sine_lut[idx - SINE_LUT_Q2_END];
        t2 = -(int)s_sine_lut[idx - SINE_LUT_Q2_END + 1u];
    } else {
        t1 = -(int)s_sine_lut[SINE_LUT_Q4_END - idx];
        t2 = -(int)s_sine_lut[SINE_LUT_Q4_END - 1u - idx];
    }
    return (1.0f / SINE_LUT_AMPL_SCALE) * (float)(t1 + (((t2 - t1) * frac) >> SINE_LUT_FRAC_BITS));
}

float foc_cos(float a)
{
    /* SimpleFOC: cos(a) = sin(a + π/2)，并把结果折回一圈 */
    float a_sin = a + FOC_PI_2;
    if (a_sin > FOC_2PI) {
        a_sin -= FOC_2PI;
    }
    return foc_sin(a_sin);
}

void foc_sincos(float a, float *out_sin, float *out_cos)
{
    *out_sin = foc_sin(a);
    *out_cos = foc_cos(a);
}

/* -------------------------------------------------------------------------- */
/*  _atan2：多项式近似（源自 ODrive，SimpleFOC 同款）                          */
/* -------------------------------------------------------------------------- */

float foc_atan2(float y, float x)
{
    const float abs_y = fabsf(y);
    const float abs_x = fabsf(x);

    /* a = min(|x|,|y|) / max(|x|,|y|)；分母加 FLT_MIN 避免除零 */
    const float mn = (abs_x < abs_y) ? abs_x : abs_y;
    const float mx = (abs_x < abs_y) ? abs_y : abs_x;
    const float a  = mn / (mx + FOC_FLT_MIN);

    const float s = a * a;
    float r = ((FOC_ATAN2_C1 * s + FOC_ATAN2_C2) * s + FOC_ATAN2_C3) * s * a + a;

    if (abs_y > abs_x) {
        r = FOC_ATAN2_PI_2 - r;
    }
    if (x < 0.0f) {
        r = FOC_ATAN2_PI - r;
    }
    if (y < 0.0f) {
        r = -r;
    }
    return r;
}

/* -------------------------------------------------------------------------- */
/*  角度                                                                       */
/* -------------------------------------------------------------------------- */

float foc_normalize_angle(float angle)
{
    const float a = fmodf(angle, FOC_2PI);
    return (a >= 0.0f) ? a : (a + FOC_2PI);
}

float foc_electrical_angle(float shaft_angle, int pole_pairs)
{
    return shaft_angle * (float)pole_pairs;
}

/* -------------------------------------------------------------------------- */
/*  变换                                                                       */
/* -------------------------------------------------------------------------- */

foc_ab_t foc_clarke(foc_abc_t iabc)
{
    /* SimpleFOC: 用 a + b + c = 0 的约束，把三相先去掉共模再变换 */
    const float mid = FOC_ONE_THIRD * (iabc.a + iabc.b + iabc.c);
    const float a = iabc.a - mid;
    const float b = iabc.b - mid;

    const foc_ab_t out = {
        .alpha = a,
        .beta  = FOC_1_SQRT3 * a + FOC_2_SQRT3 * b,
    };
    return out;
}

foc_dq_t foc_park(foc_ab_t iab, float angle_el)
{
    float s, c;
    foc_sincos(angle_el, &s, &c);

    const foc_dq_t out = {
        .d =  iab.alpha * c + iab.beta * s,
        .q =  iab.beta  * c - iab.alpha * s,
    };
    return out;
}

foc_ab_t foc_inverse_park(foc_dq_t udq, float angle_el)
{
    float s, c;
    foc_sincos(angle_el, &s, &c);

    const foc_ab_t out = {
        .alpha = c * udq.d - s * udq.q,
        .beta  = s * udq.d + c * udq.q,
    };
    return out;
}

foc_abc_t foc_inverse_clarke(foc_ab_t uab)
{
    const foc_abc_t out = {
        .a = uab.alpha,
        .b = FOC_INV_CLARKE_K * uab.alpha + FOC_SQRT3_2 * uab.beta,
        .c = FOC_INV_CLARKE_K * uab.alpha - FOC_SQRT3_2 * uab.beta,
    };
    return out;
}
