#ifndef _chassis_yaw_hold_h_
#define _chassis_yaw_hold_h_

#include <stdbool.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

typedef struct {
    bool enabled;       /**< true 启用 yaw 保持，false 仅透传用户 wz */
    float kp;           /**< yaw 角误差比例增益，单位 roughly rad/s per rad */
    float kd;           /**< gyro z 阻尼增益，单位 roughly rad/s per rad/s */
    float k_vx;         /**< vx 引起的偏航漂移前馈系数，wz = k_vx * vx */
    float k_vy;         /**< vy 引起的偏航漂移前馈系数，wz = -k_vy * vy */
    float v_deadband;   /**< 平移速度死区，单位 m/s */
    float wz_deadband;  /**< 用户旋转命令死区，单位 rad/s */
    float wz_limit;     /**< yaw 保持输出角速度限幅，单位 rad/s */
} ChassisYawHoldConfig;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

ChassisYawHoldConfig chassis_yaw_hold_default_config(void);
void chassis_yaw_hold_init(const ChassisYawHoldConfig* config);
void chassis_yaw_hold_reset(void);
float chassis_yaw_hold_apply(float vx_cmd, float vy_cmd, float wz_cmd, float yaw, float gyro_z_corrected);

bool chassis_yaw_hold_is_active(void);
float chassis_yaw_hold_get_yaw_ref(void);
float chassis_yaw_hold_get_yaw_error(void);
float chassis_yaw_hold_get_ff_wz(void);
float chassis_yaw_hold_get_fb_wz(void);
float chassis_yaw_hold_get_output_wz(void);

#endif
