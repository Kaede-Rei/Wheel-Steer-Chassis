#ifndef _entry_h_
#define _entry_h_

// ! system ! //



// ! app ! //



// ! service ! //
#include "assemble.h"
#include "chassis.h"

// ! device ! //
#include "imu/imu.h"
#include "imu/bmi088.h"

// ! domain ! //



// ! infra ! //
#include "log.h"
#include "delay.h"

// ! platform ! //



// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

static ms_t log_task = 0;
// static ms_t imu_task = 0;

static ImuAcc accel = { 0.0f, 0.0f, 0.0f };
static ImuGyro gyro = { 0.0f, 0.0f, 0.0f };
static ImuAngle angle = { 0.0f, 0.0f, 0.0f };
static float temp = 0.0f;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 程序初始化入口函数
 */
static inline void entry_init(void) {
    assemble_init();
    chassis.set_velocity(0.0f, 0.0f, 0.0f);
}

/**
 * @brief 程序主循环入口函数
 */
static inline void entry_loop(void) {
    // ! 事 件 驱 动 任 务 ! //
    if(chassis_control_flag) {
        chassis_control_flag = false;
        chassis.process();
    }

    imu_update();
    accel = imu_get_acc();
    gyro = imu_get_gyro();
    angle = imu_get_angle();

    // ! 周 期 性 任 务 ! //
    if(delay_nb_ms(&log_task, 1000)) {
        SteerWheelState state = *chassis.get_state();

        temp = bmi088_get_temp();
        log_vofa(state.cur_vx, state.cur_vy, state.cur_wz,
            gyro.x, gyro.y, gyro.z, temp,
            angle.roll, angle.pitch, angle.yaw);
    }
}

#endif
