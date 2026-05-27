#include "assemble.h"

#include "attitude.h"

SystemStatus assemble_attitude(void) {
    AttitudeConfig config = {
        /**
         * 先让角度环有明显纠偏能力
         * 如果出现左右摆头，再降 yaw_kp 或升 yaw_kd
         */
        .yaw_kp = 6.0f,
        .yaw_kd = 0.05f,

        .max_wz_correction = 1.2f,
        .max_wz_slew_rate = 12.0f,

        /**
         * attitude_correct_chassis_cmd() 如果在 remote_process() 里调用，
         * remote_process() 是 100Hz，则这里填 0.01f
         */
        .correction_period_s = 0.01f,

        /**
         * manual_wz_deadband 必须比遥控器中位抖动大
         * 之前 0.05f 很可能太小
         */
        .manual_wz_deadband = 0.15f,

        /**
         * 低速平移也要进入 yaw-hold，所以不要太大
         */
        .translation_deadband = 0.03f,

        /**
         * 第一轮先给很小死区，避免完全没反应
         */
        .yaw_error_deadband = 0.0015f,

        /**
         * 100Hz 调用：
         * manual_enter_count = 3 约等于 30ms；
         * static_reset_count = 50 约等于 500ms
         */
        .manual_enter_count = 3u,
        .static_reset_count = 50u,

        /**
         * 你的实测趋势：
         * vx > 0  -> 实际 wz > 0 漂，所以 trim_vx_to_wz 应该为负；
         * vy > 0  -> 实际 wz < 0 漂，所以 trim_vy_to_wz 应该为正
         *
         * 第一轮可以先全 0，只验证 yaw-hold
         */
        .trim_vx_to_wz = 0.0f,
        .trim_vy_to_wz = 0.0f,

        /**
         * 当 yaw_error 已经超过 0.006rad 但 correction 太小时，
         * 至少给 0.06rad/s 的修正
         */
        .min_wz_correction = 0.06f,
        .min_correction_error = 0.006f,

        .enable_yaw_hold = 1u,
    };

    if(attitude_init(&config) != ATTITUDE_STATUS_OK) {
        return SYSTEM_STATUS_ERROR;
    }

    return SYSTEM_STATUS_OK;
}