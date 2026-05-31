#include "chassis_yaw_hold.h"

#include "pid.h"

#include <math.h>
#include <string.h>

#define CHASSIS_YAW_HOLD_PI      3.14159265358979323846f
#define CHASSIS_YAW_HOLD_2PI     (2.0f * CHASSIS_YAW_HOLD_PI)
typedef struct {
    ChassisYawHoldConfig config;
    Pid pd;
    bool initialized;
    bool active;
    float yaw_ref;
    float last_yaw_error;
    float last_fb_wz;
    float last_output_wz;
} ChassisYawHoldState;

static ChassisYawHoldState s_yaw_hold = { 0 };

static float chassis_yaw_hold_wrap_pi(float angle);

ChassisYawHoldConfig chassis_yaw_hold_default_config(void) {
    ChassisYawHoldConfig config;

    config.enabled = true;
    config.kp = 48.0f;
    config.kd = 2.0f;
    config.v_deadband = 0.01f;
    config.wz_deadband = 0.08f;
    config.wz_limit = 1.0f;
    return config;
}

void chassis_yaw_hold_init(const ChassisYawHoldConfig* config) {
    ChassisYawHoldConfig default_config = chassis_yaw_hold_default_config();

    memset(&s_yaw_hold, 0, sizeof(s_yaw_hold));
    s_yaw_hold.config = (config != NULL) ? *config : default_config;
    pid_init(&s_yaw_hold.pd, PID_MODE_P, PID_FEAT_OUTPUT_LIMIT);
    pid_set_gains(&s_yaw_hold.pd, s_yaw_hold.config.kp, 0.0f, s_yaw_hold.config.kd);
    pid_set_params(&s_yaw_hold.pd, s_yaw_hold.config.wz_limit, 0.0f, 0.0f, 0.0f, 0.0f);
    s_yaw_hold.initialized = true;
}

void chassis_yaw_hold_set_target(float yaw_ref) {
    if(!s_yaw_hold.initialized) {
        chassis_yaw_hold_init(NULL);
    }

    s_yaw_hold.yaw_ref = chassis_yaw_hold_wrap_pi(yaw_ref);
    s_yaw_hold.active = true;
    s_yaw_hold.last_yaw_error = 0.0f;
    s_yaw_hold.last_fb_wz = 0.0f;
    s_yaw_hold.last_output_wz = 0.0f;
    pid_reset(&s_yaw_hold.pd);
}

void chassis_yaw_hold_disable(void) {
    if(!s_yaw_hold.initialized) {
        chassis_yaw_hold_init(NULL);
    }

    s_yaw_hold.active = false;
    s_yaw_hold.yaw_ref = 0.0f;
    s_yaw_hold.last_yaw_error = 0.0f;
    s_yaw_hold.last_fb_wz = 0.0f;
    s_yaw_hold.last_output_wz = 0.0f;
    pid_reset(&s_yaw_hold.pd);
}

void chassis_yaw_hold_reset(void) {
    s_yaw_hold.last_yaw_error = 0.0f;
    s_yaw_hold.last_fb_wz = 0.0f;
    s_yaw_hold.last_output_wz = 0.0f;
    pid_reset(&s_yaw_hold.pd);
}

float chassis_yaw_hold_apply(float vx_cmd, float vy_cmd, float wz_cmd, float yaw, float gyro_z_corrected, float dt_s) {
    const ChassisYawHoldConfig* config = &s_yaw_hold.config;
    bool user_rotating;
    bool user_translating;
    float yaw_error;
    float p_wz;
    float d_wz;

    if(!s_yaw_hold.initialized) {
        chassis_yaw_hold_init(NULL);
    }

    if(!config->enabled || !isfinite(yaw) || !isfinite(gyro_z_corrected)
        || dt_s <= 0.0f || dt_s > 0.05f) {
        s_yaw_hold.last_yaw_error = 0.0f;
        s_yaw_hold.last_fb_wz = 0.0f;
        s_yaw_hold.last_output_wz = wz_cmd;
        pid_reset(&s_yaw_hold.pd);
        return wz_cmd;
    }

    user_rotating = fabsf(wz_cmd) > config->wz_deadband;
    user_translating = fabsf(vx_cmd) > config->v_deadband || fabsf(vy_cmd) > config->v_deadband;

    if(user_rotating) {
        s_yaw_hold.last_yaw_error = 0.0f;
        s_yaw_hold.last_fb_wz = 0.0f;
        s_yaw_hold.last_output_wz = wz_cmd;
        pid_reset(&s_yaw_hold.pd);
        return wz_cmd;
    }

    if(!user_translating || !s_yaw_hold.active) {
        s_yaw_hold.last_yaw_error = 0.0f;
        s_yaw_hold.last_fb_wz = 0.0f;
        s_yaw_hold.last_output_wz = wz_cmd;
        pid_reset(&s_yaw_hold.pd);
        return wz_cmd;
    }

    yaw_error = chassis_yaw_hold_wrap_pi(s_yaw_hold.yaw_ref - yaw);
    p_wz = pid_calculate(&s_yaw_hold.pd, yaw_error, 0.0f, dt_s);
    d_wz = -config->kd * gyro_z_corrected;
    s_yaw_hold.last_fb_wz = p_wz + d_wz;
    if(s_yaw_hold.last_fb_wz > config->wz_limit) {
        s_yaw_hold.last_fb_wz = config->wz_limit;
    }
    else if(s_yaw_hold.last_fb_wz < -config->wz_limit) {
        s_yaw_hold.last_fb_wz = -config->wz_limit;
    }
    s_yaw_hold.last_yaw_error = yaw_error;
    s_yaw_hold.last_output_wz = wz_cmd + s_yaw_hold.last_fb_wz;
    return s_yaw_hold.last_output_wz;
}

bool chassis_yaw_hold_is_active(void) {
    return s_yaw_hold.active;
}

float chassis_yaw_hold_get_yaw_ref(void) {
    return s_yaw_hold.yaw_ref;
}

float chassis_yaw_hold_get_yaw_error(void) {
    return s_yaw_hold.last_yaw_error;
}

float chassis_yaw_hold_get_fb_wz(void) {
    return s_yaw_hold.last_fb_wz;
}

float chassis_yaw_hold_get_output_wz(void) {
    return s_yaw_hold.last_output_wz;
}

static float chassis_yaw_hold_wrap_pi(float angle) {
    if(!isfinite(angle)) {
        return 0.0f;
    }

    angle = fmodf(angle, CHASSIS_YAW_HOLD_2PI);
    if(angle >= CHASSIS_YAW_HOLD_PI) {
        angle -= CHASSIS_YAW_HOLD_2PI;
    }
    else if(angle < -CHASSIS_YAW_HOLD_PI) {
        angle += CHASSIS_YAW_HOLD_2PI;
    }

    return angle;
}
