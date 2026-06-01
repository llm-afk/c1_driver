/**
 * @file    torque_calib.h
 * @brief   等幅值 Iq-力矩标定转换模块 (不考虑温度)
 *
 * 基于实测标定表 + 二分查找 + 线性插值，实现 q 轴电流与输出力矩的双向转换。
 * 标定表仅存放第一象限数据 (Iq >= 0, torque >= 0)，函数内部自动处理符号。
 *
 * 用法:
 *   1. 定义标定表 (通常用脚本从 CSV 生成):
 *        static const tTorqueCalibPoint calib_table[] = {
 *            {0.0f, 0.0f},
 *            {0.1f, 0.0105f},
 *            ...
 *        };
 *   2. 调用:
 *        float iq = calib_torque_to_current(1.5f, calib_table, ARRAY_SIZE(calib_table));
 *        float tq = calib_current_to_torque(10.0f, calib_table, ARRAY_SIZE(calib_table));
 */

#ifndef TORQUE_CALIB_H
#define TORQUE_CALIB_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * 类型定义
 * -------------------------------------------------------------------------- */

/** 单个标定点: Iq [A] -> 输出力矩 [Nm] */
typedef struct {
    float iq;       /**< 等幅值 q 轴电流 [A] */
    float torque;   /**< 电机输出端实测力矩 [Nm] */
} tTorqueCalibPoint;

/* --------------------------------------------------------------------------
 * API
 * -------------------------------------------------------------------------- */

/**
 * @brief   Iq -> 力矩 (正向)
 * @param   iq      等幅值 q 轴电流 [A], 支持正负
 * @param   table   标定表 (必须升序, 首点必须为 {0,0})
 * @param   length  标定表长度 (必须 >= 2)
 * @return  输出力矩 [Nm], 符号与 iq 一致
 */
float calib_current_to_torque(float iq,
                              const tTorqueCalibPoint *table,
                              uint16_t length);

/**
 * @brief   力矩 -> Iq (反向)
 * @param   torque  目标力矩 [Nm], 支持正负
 * @param   table   标定表 (必须升序, 首点必须为 {0,0})
 * @param   length  标定表长度 (必须 >= 2)
 * @return  所需 q 轴电流 [A], 符号与 torque 一致
 */
float calib_torque_to_current(float torque,
                              const tTorqueCalibPoint *table,
                              uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* TORQUE_CALIB_H */
