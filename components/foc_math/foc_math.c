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

#define SINE_LUT_LAST   (64u)
#define SINE_LUT_SIZE   (64u * 4u * 256u)   /* 一整圈对应的定点刻度数 */

float foc_sin(float a)
{
    /* 先把负角度折到正，避免无符号转换出问题 */
    if (a < 0.0f) {
        a = -a;
        /* sin(-x) = -sin(x)，用符号位翻转，下面统一取值 */
        const unsigned int i = (unsigned int)(a * ((float)SINE_LUT_SIZE / FOC_2PI));
        const int frac = (int)(i & 0xffu);
        unsigned int idx = (i >> 8) & 0xffu;
        int t1, t2;
        if (idx < 64u) {
            t1 = (int)s_sine_lut[idx];
            t2 = (int)s_sine_lut[idx + 1u];
        } else if (idx < 128u) {
            t1 = (int)s_sine_lut[128u - idx];
            t2 = (int)s_sine_lut[127u - idx];
        } else if (idx < 192u) {
            t1 = -(int)s_sine_lut[idx - 128u];
            t2 = -(int)s_sine_lut[idx - 127u];
        } else {
            t1 = -(int)s_sine_lut[256u - idx];
            t2 = -(int)s_sine_lut[255u - idx];
        }
        return -(1.0f / 32768.0f) * (float)(t1 + (((t2 - t1) * frac) >> 8));
    }

    const unsigned int i = (unsigned int)(a * ((float)SINE_LUT_SIZE / FOC_2PI));
    const int frac = (int)(i & 0xffu);
    unsigned int idx = (i >> 8) & 0xffu;
    int t1, t2;
    if (idx < 64u) {
        t1 = (int)s_sine_lut[idx];
        t2 = (int)s_sine_lut[idx + 1u];
    } else if (idx < 128u) {
        t1 = (int)s_sine_lut[128u - idx];
        t2 = (int)s_sine_lut[127u - idx];
    } else if (idx < 192u) {
        t1 = -(int)s_sine_lut[idx - 128u];
        t2 = -(int)s_sine_lut[idx - 127u];
    } else {
        t1 = -(int)s_sine_lut[256u - idx];
        t2 = -(int)s_sine_lut[255u - idx];
    }
    return (1.0f / 32768.0f) * (float)(t1 + (((t2 - t1) * frac) >> 8));
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
    const float a  = mn / (mx + 1.17549435e-38f);

    const float s = a * a;
    float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;

    if (abs_y > abs_x) {
        r = FOC_PI_2 - 0.0000000149f - r;      /* 1.57079637 - r */
    }
    if (x < 0.0f) {
        r = FOC_PI - 0.0000004404f - r;        /* 3.14159274 - r */
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
    const float mid = (1.0f / 3.0f) * (iabc.a + iabc.b + iabc.c);
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
        .b = -0.5f * uab.alpha + FOC_SQRT3_2 * uab.beta,
        .c = -0.5f * uab.alpha - FOC_SQRT3_2 * uab.beta,
    };
    return out;
}
