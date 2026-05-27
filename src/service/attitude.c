#include "attitude.h"

#include "imu/imu.h"

#include <math.h>
#include <string.h>

// ! ========================= 宏 定 义 ========================= ! //

/**
 * @brief 圆周率
 */
#define ATTITUDE_PI   3.14159265358979323846f

/**
 * @brief 2*pi
 */
#define ATTITUDE_2PI  (2.0f * ATTITUDE_PI)

/**
 * @brief 默认 correction 周期
 *
 * 当前建议 remote_process() 100Hz 调用 correction，所以默认 0.01s
 */
#define ATTITUDE_DEFAULT_CORRECTION_PERIOD_S 0.01f

/**
 * @brief 默认主动旋转确认计数
 */
#define ATTITUDE_DEFAULT_MANUAL_ENTER_COUNT 3u

/**
 * @brief 默认静止重置 yaw_ref 计数
 */
#define ATTITUDE_DEFAULT_STATIC_RESET_COUNT 50u

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief attitude 配置
 */
static AttitudeConfig s_config;

/**
 * @brief attitude 状态
 */
static AttitudeState s_state;

/**
 * @brief 上一帧归一化 yaw
 */
static float s_last_yaw = 0.0f;

/**
 * @brief yaw 多圈展开计数
 */
static int32_t s_yaw_round_count = 0;

/**
 * @brief 是否已有上一帧 yaw
 */
static uint8_t s_has_last_yaw = 0u;

/**
 * @brief 上一次最终 wz correction
 *
 * 用于斜率限制
 */
static float s_last_wz_correction = 0.0f;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 将角度归一化到 (-pi, pi]
 *
 * @param angle 输入角度
 * @return float 归一化后的角度
 */
static float attitude_wrap_pi(float angle);

/**
 * @brief 浮点限幅
 *
 * @param value 输入值
 * @param min_value 最小值
 * @param max_value 最大值
 * @return float 限幅后的值
 */
static float attitude_clampf(float value, float min_value, float max_value);

/**
 * @brief 斜率限制
 *
 * @param target 目标值
 * @param last 上一次值
 * @param max_step 单次最大变化量
 * @return float 限制后的值
 */
static float attitude_slew_limit(float target, float last, float max_step);

/**
 * @brief 对误差应用死区
 *
 * @param error 原始误差
 * @param deadband 死区
 * @return float 死区处理后的误差
 */
static float attitude_apply_deadband(float error, float deadband);

/**
 * @brief 对小 correction 应用最小输出量
 *
 * @param correction 原始 correction
 * @param yaw_error yaw 误差
 * @return float 修正后的 correction
 */
static float attitude_apply_min_correction(float correction, float yaw_error);

/**
 * @brief 应用默认配置
 *
 * @param config 配置指针
 */
static void attitude_apply_default_config(AttitudeConfig* config);

/**
 * @brief 清理一次 correction 相关状态
 */
static void attitude_clear_correction_state(void);

/**
 * @brief 更新 yaw_total 展开角
 *
 * @param yaw 当前归一化 yaw
 */
static void attitude_update_yaw_unwrap(float yaw);

/**
 * @brief 清零 correction 输出与斜率状态
 */
static void attitude_reset_correction_output(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

AttitudeStatus attitude_init(const AttitudeConfig* config) {
    if(config == 0) {
        return ATTITUDE_STATUS_INVALID_PARAM;
    }

    s_config = *config;
    attitude_apply_default_config(&s_config);

    memset(&s_state, 0, sizeof(s_state));

    s_last_yaw = 0.0f;
    s_yaw_round_count = 0;
    s_has_last_yaw = 0u;
    s_last_wz_correction = 0.0f;

    return ATTITUDE_STATUS_OK;
}

AttitudeStatus attitude_update(void) {
    ImuAngle angle;
    ImuGyro gyro;
    float yaw;

    /**
     * 注意：
     * 这里使用 imu_get_xxx() 包装函数，而不是 imu.xxx 宏
     * 这样即使 imu_instance 未绑定，也不会空指针解引用
     */
    angle = imu_get_angle();
    gyro = imu_get_gyro();

    yaw = attitude_wrap_pi(angle.yaw);

    if(s_has_last_yaw == 0u) {
        s_last_yaw = yaw;
        s_yaw_round_count = 0;
        s_has_last_yaw = 1u;

        s_state.yaw = yaw;
        s_state.yaw_total = yaw;
        s_state.yaw_ref = yaw;
        s_state.gyro_z = gyro.z;
        s_state.yaw_ready = 1u;

        return ATTITUDE_STATUS_OK;
    }

    attitude_update_yaw_unwrap(yaw);

    s_state.yaw = yaw;
    s_state.gyro_z = gyro.z;
    s_state.yaw_ready = 1u;

    return ATTITUDE_STATUS_OK;
}

AttitudeStatus attitude_reset_yaw(float yaw) {
    yaw = attitude_wrap_pi(yaw);

    s_last_yaw = yaw;
    s_yaw_round_count = 0;
    s_has_last_yaw = 1u;

    s_state.yaw = yaw;
    s_state.yaw_total = yaw;
    s_state.yaw_ref = yaw;
    s_state.yaw_error = 0.0f;
    s_state.yaw_error_for_control = 0.0f;
    s_state.yaw_ready = 1u;
    s_state.hold_latched = 0u;

    attitude_reset_correction_output();

    return ATTITUDE_STATUS_OK;
}

AttitudeStatus attitude_set_yaw_ref_current(void) {
    if(s_state.yaw_ready == 0u) {
        return ATTITUDE_STATUS_NOT_READY;
    }

    s_state.yaw_ref = s_state.yaw_total;
    s_state.yaw_error = 0.0f;
    s_state.yaw_error_for_control = 0.0f;
    s_state.hold_latched = 1u;

    attitude_reset_correction_output();

    return ATTITUDE_STATUS_OK;
}

AttitudeChassisCmd attitude_correct_chassis_cmd(float vx, float vy, float wz) {
    return attitude_correct_chassis_cmd_dt(vx, vy, wz, s_config.correction_period_s);
}

AttitudeChassisCmd attitude_correct_chassis_cmd_dt(float vx, float vy, float wz, float dt) {
    AttitudeChassisCmd out;
    float translation_speed;
    float yaw_error;
    float yaw_error_for_control;
    float yaw_hold_correction;
    float empirical_trim;
    float target_correction;
    float max_step;
    uint8_t manual_input;
    uint8_t translating;

    out.vx = vx;
    out.vy = vy;
    out.wz = wz;

    attitude_clear_correction_state();

    s_state.input_vx = vx;
    s_state.input_vy = vy;
    s_state.input_wz = wz;

    s_state.output_vx = vx;
    s_state.output_vy = vy;
    s_state.output_wz = wz;

    if(s_state.yaw_ready == 0u) {
        return out;
    }

    if(dt <= 0.0f || !isfinite(dt)) {
        dt = s_config.correction_period_s;
    }

    translation_speed = sqrtf(vx * vx + vy * vy);
    manual_input = fabsf(wz) > s_config.manual_wz_deadband ? 1u : 0u;
    translating = translation_speed > s_config.translation_deadband ? 1u : 0u;

    /**
     * 1. 主动旋转判定：
     *
     * 只要出现明显 wz 输入，本帧就不做 yaw-hold，避免和用户抢控制权
     * 但只有连续若干帧检测到 manual_input，才真正重置 yaw_ref
     *
     * 这样可以避免遥控器中位一帧噪声导致 yaw_ref 被频繁重置
     */
    if(manual_input != 0u) {
        if(s_state.manual_count < s_config.manual_enter_count) {
            s_state.manual_count++;
        }

        if(s_state.manual_count >= s_config.manual_enter_count) {
            s_state.manual_rotate_active = 1u;
            s_state.yaw_ref = s_state.yaw_total;
            s_state.hold_latched = 0u;
            attitude_reset_correction_output();
        }

        return out;
    }

    s_state.manual_count = 0u;

    /**
     * 2. 静止判定：
     *
     * 静止时不启用 yaw-hold
     * 但不要一检测到静止就重置 yaw_ref，而是连续静止一段时间后再重置
     *
     * 否则遥控速度抖动、任务切换间隙会不断把 yaw_ref 刷成当前 yaw，
     * yaw-hold 就失去了“保持原方向”的意义
     */
    if(translating == 0u) {
        if(s_state.static_count < s_config.static_reset_count) {
            s_state.static_count++;
        }

        if(s_state.static_count >= s_config.static_reset_count) {
            s_state.static_active = 1u;
            s_state.yaw_ref = s_state.yaw_total;
            s_state.hold_latched = 0u;
            attitude_reset_correction_output();
        }

        return out;
    }

    s_state.static_count = 0u;

    /**
     * 3. 第一次进入平移时锁定 yaw_ref
     *
     * 一旦 hold_latched 置位，平移过程中不再随便重置 yaw_ref
     */
    if(s_state.hold_latched == 0u) {
        s_state.yaw_ref = s_state.yaw_total;
        s_state.hold_latched = 1u;
        attitude_reset_correction_output();
    }

    s_state.yaw_hold_active = 1u;

    /**
     * 4. 计算 yaw error
     *
     * yaw_total 是多圈展开角，因此这里不 wrap_pi
     * 但为了防止异常跳变，把误差限制在 [-pi, pi]
     */
    yaw_error = s_state.yaw_ref - s_state.yaw_total;
    yaw_error = attitude_clampf(yaw_error, -ATTITUDE_PI, ATTITUDE_PI);

    yaw_error_for_control = attitude_apply_deadband(yaw_error, s_config.yaw_error_deadband);

    /**
     * 5. yaw-hold 角度环
     *
     * correction = kp * yaw_error - kd * gyro_z
     *
     * 如果实测方向反了，优先检查 IMU yaw 正方向与 chassis wz 正方向是否一致
     */
    if(s_config.enable_yaw_hold != 0u) {
        yaw_hold_correction =
            s_config.yaw_kp * yaw_error_for_control -
            s_config.yaw_kd * s_state.gyro_z;
    }
    else {
        yaw_hold_correction = 0.0f;
    }

    /**
     * 6. 经验前馈补偿
     *
     * 根据你实测：
     * vx > 0 时实际 wz > 0 漂，因此 trim_vx_to_wz 通常应取负；
     * vy > 0 时实际 wz < 0 漂，因此 trim_vy_to_wz 通常应取正
     */
    empirical_trim =
        s_config.trim_vx_to_wz * vx +
        s_config.trim_vy_to_wz * vy;

    target_correction = yaw_hold_correction + empirical_trim;

    /**
     * 7. 最小 correction
     *
     * 如果 yaw_error 已经不小，但 correction 太小，底盘可能没有明显响应
     * 此处把 correction 提升到一个最小可见输出
     */
    target_correction = attitude_apply_min_correction(target_correction, yaw_error);

    /**
     * 8. 总限幅
     */
    target_correction = attitude_clampf(
        target_correction,
        -s_config.max_wz_correction,
        s_config.max_wz_correction
    );

    /**
     * 9. 斜率限制
     */
    max_step = s_config.max_wz_slew_rate * dt;

    target_correction = attitude_slew_limit(
        target_correction,
        s_last_wz_correction,
        max_step
    );

    s_last_wz_correction = target_correction;

    out.wz = wz + target_correction;

    s_state.yaw_error = yaw_error;
    s_state.yaw_error_for_control = yaw_error_for_control;
    s_state.yaw_hold_correction = yaw_hold_correction;
    s_state.empirical_trim = empirical_trim;
    s_state.final_wz_correction = target_correction;

    s_state.output_vx = out.vx;
    s_state.output_vy = out.vy;
    s_state.output_wz = out.wz;

    return out;
}

const AttitudeState* attitude_get_state(void) {
    return &s_state;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static float attitude_wrap_pi(float angle) {
    if(!isfinite(angle)) {
        return 0.0f;
    }

    angle = fmodf(angle, ATTITUDE_2PI);

    if(angle >= ATTITUDE_PI) {
        angle -= ATTITUDE_2PI;
    }
    else if(angle < -ATTITUDE_PI) {
        angle += ATTITUDE_2PI;
    }

    return angle;
}

static float attitude_clampf(float value, float min_value, float max_value) {
    if(value < min_value) {
        return min_value;
    }

    if(value > max_value) {
        return max_value;
    }

    return value;
}

static float attitude_slew_limit(float target, float last, float max_step) {
    float delta;

    if(max_step <= 0.0f || !isfinite(max_step)) {
        return target;
    }

    delta = target - last;

    if(delta > max_step) {
        return last + max_step;
    }

    if(delta < -max_step) {
        return last - max_step;
    }

    return target;
}

static float attitude_apply_deadband(float error, float deadband) {
    if(deadband <= 0.0f || !isfinite(deadband)) {
        return error;
    }

    if(error > deadband) {
        return error - deadband;
    }

    if(error < -deadband) {
        return error + deadband;
    }

    return 0.0f;
}

static float attitude_apply_min_correction(float correction, float yaw_error) {
    float abs_correction;

    if(s_config.min_wz_correction <= 0.0f ||
        s_config.min_correction_error <= 0.0f) {
        return correction;
    }

    if(fabsf(yaw_error) < s_config.min_correction_error) {
        return correction;
    }

    if(correction == 0.0f) {
        return correction;
    }

    abs_correction = fabsf(correction);

    if(abs_correction >= s_config.min_wz_correction) {
        return correction;
    }

    return correction > 0.0f ?
        s_config.min_wz_correction :
        -s_config.min_wz_correction;
}

static void attitude_apply_default_config(AttitudeConfig* config) {
    if(config == 0) {
        return;
    }

    if(config->correction_period_s <= 0.0f ||
        !isfinite(config->correction_period_s)) {
        config->correction_period_s = ATTITUDE_DEFAULT_CORRECTION_PERIOD_S;
    }

    if(config->max_wz_correction <= 0.0f ||
        !isfinite(config->max_wz_correction)) {
        config->max_wz_correction = 1.0f;
    }

    if(config->max_wz_slew_rate <= 0.0f ||
        !isfinite(config->max_wz_slew_rate)) {
        config->max_wz_slew_rate = 8.0f;
    }

    if(config->manual_wz_deadband < 0.0f ||
        !isfinite(config->manual_wz_deadband)) {
        config->manual_wz_deadband = 0.15f;
    }

    if(config->translation_deadband < 0.0f ||
        !isfinite(config->translation_deadband)) {
        config->translation_deadband = 0.03f;
    }

    if(config->yaw_error_deadband < 0.0f ||
        !isfinite(config->yaw_error_deadband)) {
        config->yaw_error_deadband = 0.0f;
    }

    if(config->manual_enter_count == 0u) {
        config->manual_enter_count = ATTITUDE_DEFAULT_MANUAL_ENTER_COUNT;
    }

    if(config->static_reset_count == 0u) {
        config->static_reset_count = ATTITUDE_DEFAULT_STATIC_RESET_COUNT;
    }

    if(config->min_wz_correction < 0.0f ||
        !isfinite(config->min_wz_correction)) {
        config->min_wz_correction = 0.0f;
    }

    if(config->min_correction_error < 0.0f ||
        !isfinite(config->min_correction_error)) {
        config->min_correction_error = 0.0f;
    }
}

static void attitude_clear_correction_state(void) {
    s_state.yaw_hold_active = 0u;
    s_state.manual_rotate_active = 0u;
    s_state.static_active = 0u;

    s_state.yaw_error = 0.0f;
    s_state.yaw_error_for_control = 0.0f;
    s_state.yaw_hold_correction = 0.0f;
    s_state.empirical_trim = 0.0f;
    s_state.final_wz_correction = 0.0f;
}

static void attitude_update_yaw_unwrap(float yaw) {
    if(yaw - s_last_yaw > ATTITUDE_PI) {
        s_yaw_round_count--;
    }
    else if(yaw - s_last_yaw < -ATTITUDE_PI) {
        s_yaw_round_count++;
    }

    s_last_yaw = yaw;
    s_state.yaw_total = (float)s_yaw_round_count * ATTITUDE_2PI + yaw;
}

static void attitude_reset_correction_output(void) {
    s_last_wz_correction = 0.0f;

    s_state.yaw_error = 0.0f;
    s_state.yaw_error_for_control = 0.0f;
    s_state.yaw_hold_correction = 0.0f;
    s_state.empirical_trim = 0.0f;
    s_state.final_wz_correction = 0.0f;
}