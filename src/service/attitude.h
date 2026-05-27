#ifndef _attitude_h_
#define _attitude_h_

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 姿态服务状态码
 */
typedef enum {
    ATTITUDE_STATUS_OK = 0,             /**< 操作成功 */
    ATTITUDE_STATUS_ERROR,              /**< 通用错误 */
    ATTITUDE_STATUS_INVALID_PARAM,      /**< 参数无效 */
    ATTITUDE_STATUS_NOT_READY,          /**< 姿态尚未就绪 */
} AttitudeStatus;

/**
 * @brief 姿态服务配置
 *
 * attitude 服务不负责 IMU 内部姿态解算，也不做底盘 odom 融合
 *
 * 它只做一件事：
 *
 * @code
 * 用户/上层输入 vx, vy, wz
 *      ↓
 * 使用 IMU yaw / gyro_z 修正 wz
 *      ↓
 * 输出 vx, vy, wz_corrected
 * @endcode
 */
typedef struct {
    /**
     * @brief yaw 角度环比例增益
     *
     * 单位近似为 1/s
     *
     * @note correction_p = yaw_kp * yaw_error
     */
    float yaw_kp;

    /**
     * @brief yaw 角速度阻尼增益
     *
     * 使用 IMU gyro_z 抑制角度环过冲
     *
     * @note correction_d = -yaw_kd * gyro_z
     */
    float yaw_kd;

    /**
     * @brief 最大 wz 修正量
     *
     * 单位 rad/s
     *
     * attitude 最多只能给底盘额外叠加这么大的 wz
     */
    float max_wz_correction;

    /**
     * @brief wz 修正量最大变化率
     *
     * 单位 rad/s^2
     *
     * 用于避免 correction 突变导致底盘抖动
     */
    float max_wz_slew_rate;

    /**
     * @brief correction 函数调用周期
     *
     * 单位 s
     *
     * 如果 attitude_correct_chassis_cmd() 在 remote_process() 中调用，
     * 而 remote_process() 是 100Hz，则填 0.01f
     */
    float correction_period_s;

    /**
     * @brief 用户主动旋转死区
     *
     * 当 |wz_user| 大于该值时，认为用户主动要求旋转，
     * 此时 attitude 不抢控制权
     */
    float manual_wz_deadband;

    /**
     * @brief 平移速度死区
     *
     * 当 sqrt(vx^2 + vy^2) 小于该值时，认为底盘近似静止，
     * 此时不启用 yaw-hold
     */
    float translation_deadband;

    /**
     * @brief yaw 角度误差死区
     *
     * 单位 rad
     *
     * 用于避免 IMU 极小慢漂引起底盘持续微小自转
     */
    float yaw_error_deadband;

    /**
     * @brief 进入主动旋转状态所需连续次数
     *
     * 例如 correction 100Hz 调用，填 3 表示连续 30ms 检测到 wz 输入后，
     * 才认为用户确实在主动旋转
     */
    uint16_t manual_enter_count;

    /**
     * @brief 静止后重置 yaw_ref 所需连续次数
     *
     * 例如 correction 100Hz 调用，填 50 表示静止 500ms 后，
     * 才把 yaw_ref 重置为当前 yaw
     */
    uint16_t static_reset_count;

    /**
     * @brief vx 到 wz 的经验前馈补偿系数
     *
     * 单位约为 (rad/s) / (m/s)
     *
     * @note empirical_trim += trim_vx_to_wz * vx
     */
    float trim_vx_to_wz;

    /**
     * @brief vy 到 wz 的经验前馈补偿系数
     *
     * 单位约为 (rad/s) / (m/s)
     *
     * @note empirical_trim += trim_vy_to_wz * vy
     */
    float trim_vy_to_wz;

    /**
     * @brief 最小 wz 修正量
     *
     * 单位 rad/s
     *
     * 当 yaw_error 超过 min_correction_error，且 correction 非零但太小时，
     * 会提升到该最小修正量
     *
     * 这个参数用于解决 correction 太小、实际底盘没有明显响应的问题
     */
    float min_wz_correction;

    /**
     * @brief 启用最小 correction 的 yaw 误差阈值
     *
     * 单位 rad
     */
    float min_correction_error;

    /**
     * @brief 是否启用 yaw-hold 角度环
     *
     * 0：关闭 yaw-hold，只保留经验前馈 trim
     * 1：启用 yaw-hold
     */
    uint8_t enable_yaw_hold;
} AttitudeConfig;

/**
 * @brief attitude 修正后的底盘命令
 */
typedef struct {
    float vx; /**< x 方向速度，单位 m/s */
    float vy; /**< y 方向速度，单位 m/s */
    float wz; /**< z 轴角速度，单位 rad/s */
} AttitudeChassisCmd;

/**
 * @brief attitude 运行状态
 */
typedef struct {
    float yaw;       /**< 当前 IMU yaw，范围 (-pi, pi]，单位 rad */
    float yaw_total; /**< 多圈展开 yaw，单位 rad */
    float yaw_ref;   /**< yaw-hold 目标角，单位 rad */
    float gyro_z;    /**< IMU z 轴角速度，单位 rad/s */

    float yaw_error;             /**< yaw_ref - yaw_total，单位 rad */
    float yaw_error_for_control; /**< 死区处理后的 yaw error */
    float yaw_hold_correction;   /**< yaw 角度环输出，单位 rad/s */
    float empirical_trim;        /**< vx/vy 前馈补偿，单位 rad/s */
    float final_wz_correction;   /**< 最终叠加到 wz 上的修正量，单位 rad/s */

    float input_vx; /**< 最近一次输入 vx */
    float input_vy; /**< 最近一次输入 vy */
    float input_wz; /**< 最近一次输入 wz */

    float output_vx; /**< 最近一次输出 vx */
    float output_vy; /**< 最近一次输出 vy */
    float output_wz; /**< 最近一次输出 wz */

    uint16_t manual_count; /**< 主动旋转连续计数 */
    uint16_t static_count; /**< 静止连续计数 */

    uint8_t yaw_ready;            /**< yaw 是否就绪 */
    uint8_t yaw_hold_active;      /**< 当前是否启用 yaw-hold */
    uint8_t manual_rotate_active; /**< 当前是否处于主动旋转状态 */
    uint8_t static_active;        /**< 当前是否处于静止状态 */
    uint8_t hold_latched;         /**< 是否已经锁定 yaw_ref */
} AttitudeState;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化 attitude 服务
 *
 * @param config attitude 配置
 * @return AttitudeStatus 状态码
 */
AttitudeStatus attitude_init(const AttitudeConfig* config);

/**
 * @brief 更新 attitude 内部 yaw 状态
 *
 * 该函数应在 imu_update() 成功后调用
 *
 * @return AttitudeStatus 状态码
 */
AttitudeStatus attitude_update(void);

/**
 * @brief 重置 yaw 和 yaw_ref
 *
 * @param yaw 新 yaw，单位 rad
 * @return AttitudeStatus 状态码
 */
AttitudeStatus attitude_reset_yaw(float yaw);

/**
 * @brief 把当前 yaw_total 设置为 yaw_ref
 *
 * @return AttitudeStatus 状态码
 */
AttitudeStatus attitude_set_yaw_ref_current(void);

/**
 * @brief 对底盘速度命令进行 yaw-hold 修正
 *
 * 使用 config.correction_period_s 作为斜率限制周期
 *
 * @param vx 输入 x 方向速度
 * @param vy 输入 y 方向速度
 * @param wz 输入 z 轴角速度
 * @return AttitudeChassisCmd 修正后的底盘速度命令
 */
AttitudeChassisCmd attitude_correct_chassis_cmd(float vx, float vy, float wz);

/**
 * @brief 对底盘速度命令进行 yaw-hold 修正，并显式传入 dt
 *
 * @param vx 输入 x 方向速度
 * @param vy 输入 y 方向速度
 * @param wz 输入 z 轴角速度
 * @param dt correction 调用周期，单位 s
 * @return AttitudeChassisCmd 修正后的底盘速度命令
 */
AttitudeChassisCmd attitude_correct_chassis_cmd_dt(float vx, float vy, float wz, float dt);

/**
 * @brief 获取 attitude 只读状态
 *
 * @return const AttitudeState* 状态指针
 */
const AttitudeState* attitude_get_state(void);

#endif