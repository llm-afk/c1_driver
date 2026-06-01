/**
 * @file    torque_calib.c
 * @brief   等幅值 Iq-力矩标定转换模块 (不考虑温度)
 *
 * 约定:
 *  - table[0] 必须为 {0.0f, 0.0f}
 *  - table 严格按 iq (及 torque) 升序排列
 *  - 输入超出表范围时, 用最后两个点的斜率线性外推
 *  - 查找 O(log N), 无动态内存
 */

#include "torque_calib.h"
#include <math.h>
#include <stddef.h>

/* =========================================================================
 * 内部工具
 * ========================================================================= */

/**
 * @brief  Iq 绝对值 -> 力矩绝对值 (一维线性插值/外推)
 */
static float interp_iq_to_torque(float abs_iq,
                                 const tTorqueCalibPoint *table,
                                 uint16_t length)
{
    /* ---- 下边界: 0 点直接返回 ---- */
    if (abs_iq <= table[0].iq) {
        return table[0].torque;                     /* {0,0} 保证为 0 */
    }

    /* ---- 上边界外推 ---- */
    if (abs_iq >= table[length - 1].iq) {
        float dx = table[length - 1].iq     - table[length - 2].iq;
        float dy = table[length - 1].torque - table[length - 2].torque;
        float slope = dy / (dx + 1e-9f);
        return table[length - 1].torque + slope * (abs_iq - table[length - 1].iq);
    }

    /* ---- 二分查找区间 [idx, idx+1] ---- */
    uint16_t lo = 0;
    uint16_t hi = length - 2;
    uint16_t idx = 0;

    while (lo <= hi) {
        uint16_t mid = (lo + hi) >> 1;
        if (abs_iq >= table[mid].iq && abs_iq <= table[mid + 1].iq) {
            idx = mid;
            break;
        }
        if (abs_iq < table[mid].iq) {
            hi = mid - 1;
        } else {
            lo = mid + 1;
        }
    }

    /* ---- 区间内线性插值 ---- */
    float dx = table[idx + 1].iq - table[idx].iq;
    float dy = table[idx + 1].torque - table[idx].torque;
    return table[idx].torque + (abs_iq - table[idx].iq) * dy / (dx + 1e-9f);
}


/**
 * @brief  力矩绝对值 -> Iq 绝对值 (一维线性插值/外推)
 */
static float interp_torque_to_iq(float abs_torque,
                                 const tTorqueCalibPoint *table,
                                 uint16_t length)
{
    /* ---- 下边界 ---- */
    if (abs_torque <= table[0].torque) {
        return table[0].iq;                         /* {0,0} 保证为 0 */
    }

    /* ---- 上边界外推 ---- */
    if (abs_torque >= table[length - 1].torque) {
        float dx = table[length - 1].torque - table[length - 2].torque;
        float dy = table[length - 1].iq     - table[length - 2].iq;
        float slope = dy / (dx + 1e-9f);
        return table[length - 1].iq + slope * (abs_torque - table[length - 1].torque);
    }

    /* ---- 二分查找区间 [idx, idx+1] ---- */
    uint16_t lo = 0;
    uint16_t hi = length - 2;
    uint16_t idx = 0;

    while (lo <= hi) {
        uint16_t mid = (lo + hi) >> 1;
        if (abs_torque >= table[mid].torque && abs_torque <= table[mid + 1].torque) {
            idx = mid;
            break;
        }
        if (abs_torque < table[mid].torque) {
            hi = mid - 1;
        } else {
            lo = mid + 1;
        }
    }

    /* ---- 区间内线性插值 ---- */
    float dx = table[idx + 1].torque - table[idx].torque;
    float dy = table[idx + 1].iq     - table[idx].iq;
    return table[idx].iq + (abs_torque - table[idx].torque) * dy / (dx + 1e-9f);
}


/* =========================================================================
 * 公开 API
 * ========================================================================= */

float calib_current_to_torque(float iq,
                              const tTorqueCalibPoint *table,
                              uint16_t length)
{
    if (table == NULL || length < 2) {
        return 0.0f;
    }

    float sign   = (iq >= 0.0f) ? 1.0f : -1.0f;
    float abs_iq = fabsf(iq);

    return sign * interp_iq_to_torque(abs_iq, table, length);
}


float calib_torque_to_current(float torque,
                              const tTorqueCalibPoint *table,
                              uint16_t length)
{
    if (table == NULL || length < 2) {
        return 0.0f;
    }

    float sign       = (torque >= 0.0f) ? 1.0f : -1.0f;
    float abs_torque = fabsf(torque);

    return sign * interp_torque_to_iq(abs_torque, table, length);
}
