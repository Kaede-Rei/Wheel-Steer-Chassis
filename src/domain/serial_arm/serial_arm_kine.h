#ifndef _serial_arm_kine_h_
#define _serial_arm_kine_h_

#include <stdbool.h>
#include <stdint.h>

/**
 * @file serial_arm_kine.h
 * @brief 串联机械臂运动学模块对外接口
 */

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 串联机械臂运动学模块统一入口别名
 */
#define serial_arm serial_arm_kine_instance

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef M_2PI
#define M_2PI (2.0f * M_PI)
#endif

#define SERIAL_ARM_MAX_DOF 8u
#define SERIAL_ARM_TASK_MAX_DIM 6u
#define SERIAL_ARM_MAX_SOLUTIONS 16u

/**
 * @brief 串联机械臂运动学模块状态码
 */
typedef enum {
    /** 操作成功 */
    SERIAL_ARM_STATUS_SUCCESS = 0,
    /** 通用错误 */
    SERIAL_ARM_STATUS_ERROR,
    /** 模块尚未初始化 */
    SERIAL_ARM_STATUS_NOT_INITIALIZED,
    /** 模型参数无效 */
    SERIAL_ARM_STATUS_INVALID_MODEL,
    /** 关节数组无效 */
    SERIAL_ARM_STATUS_INVALID_JOINTS,
    /** 目标位姿无效 */
    SERIAL_ARM_STATUS_INVALID_POSE,
    /** 求解过程中检测到奇异性 */
    SERIAL_ARM_STATUS_SINGULARITY,
    /** 目标超出可达范围 */
    SERIAL_ARM_STATUS_OUT_OF_REACH,
    /** 未找到可行解 */
    SERIAL_ARM_STATUS_NO_SOLUTION,
} SerialArmStatus;

/**
 * @brief DH 参数约定类型
 */
typedef enum {
    /** 传统 DH，A_i = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha) */
    SERIAL_ARM_DH_STANDARD = 0,
    /** 改进 DH 或 MDH，A_i = Tx(a) * Rx(alpha) * Rz(theta) * Tz(d) */
    SERIAL_ARM_DH_MODIFIED,
} SerialArmDhConvention;

/**
 * @brief 关节类型
 */
typedef enum {
    /** 转动关节 */
    SERIAL_ARM_JOINT_REVOLUTE = 0,
    /** 移动关节 */
    SERIAL_ARM_JOINT_PRISMATIC,
} SerialArmJointType;

/**
 * @brief 三维点或平移向量
 */
typedef struct {
    float x;
    float y;
    float z;
} SerialArmPoint;

/**
 * @brief 四元数，顺序为 w、x、y、z
 */
typedef struct {
    float w;
    float x;
    float y;
    float z;
} SerialArmQuaternion;

/**
 * @brief 欧拉角，单位 rad，顺序为 roll、pitch、yaw
 */
typedef struct {
    float roll;
    float pitch;
    float yaw;
} SerialArmRPY;

/**
 * @brief 位姿，包含位置和姿态
 */
typedef struct {
    SerialArmPoint position;
    SerialArmQuaternion orientation;
} SerialArmPose;

/**
 * @brief 4x4 齐次变换矩阵
 */
typedef struct {
    float m[4][4];
} SerialArmTransform;

/**
 * @brief 单个连杆的 DH 或 MDH 参数
 */
typedef struct {
    /** 关节类型 */
    SerialArmJointType type;

    /** DH 或 MDH 固定参数，长度单位 m，角度单位 rad */
    float theta;
    float d;
    float a;
    float alpha;

    /** 关节变量偏移，q_model = q_user + q_offset */
    float q_offset;
    /** 关节最小值 */
    float q_min;
    /** 关节最大值 */
    float q_max;
} SerialArmLink;

/**
 * @brief 数值 IK 配置
 */
typedef struct {
    /** 最大迭代次数，默认 300 */
    float max_iterations;
    /** 位置收敛阈值，单位 m，默认 1e-4 */
    float position_tolerance;
    /** 姿态收敛阈值，单位 rad，默认 1e-3 */
    float orientation_tolerance;
    /** 单步增益，默认 0.5 */
    float step_gain;
    /** 阻尼最小二乘阻尼项，默认 1e-3 */
    float damping;
    /** 数值雅可比扰动量，默认 1e-5 */
    float numeric_eps;

    /** 位置误差权重 */
    float position_weight;
    /** 姿态误差权重 */
    float orientation_weight;
} SerialArmIkConfig;

/**
 * @brief 串联机械臂模型
 */
typedef struct {
    /** 自由度数量 */
    uint8_t dof;
    /** DH 约定 */
    SerialArmDhConvention convention;
    /** 连杆参数数组 */
    SerialArmLink link[SERIAL_ARM_MAX_DOF];

    /** 可选固定变换，通常保持单位阵，base_T * A1 * ... * An * tool_T */
    SerialArmTransform base_T;
    /** 末端工具坐标系固定变换 */
    SerialArmTransform tool_T;

    /** 数值 IK 配置 */
    SerialArmIkConfig ik;
} SerialArmModel;

/**
 * @brief 关节数组，q[i] 为用户层关节变量
 */
typedef struct {
    /** 自由度数量 */
    uint8_t dof;
    /** 关节变量数组 */
    float q[SERIAL_ARM_MAX_DOF];
} SerialArmJointArray;

/**
 * @brief IK 多解结果
 */
typedef struct {
    /** 有效解数量 */
    uint8_t num_solutions;
    /** 解数组 */
    SerialArmJointArray solution[SERIAL_ARM_MAX_SOLUTIONS];
} SerialArmJointSolutions;

/**
 * @brief 自动推断出的 IK 任务行信息
 * @details row 含义为 0=x，1=y，2=z，3=rx，4=ry，5=rz
 */
typedef struct {
    /** 任务维度 */
    uint8_t task_dim;
    /** 任务行索引 */
    uint8_t row[SERIAL_ARM_TASK_MAX_DIM];
} SerialArmTaskInfo;

/**
 * @brief 模块统一接口表，外部可通过 serial_arm.xxx(...) 调用
 */
typedef struct SerialArmKineInterface {
    /**
     * @brief 重置模型，并设置自由度与 DH 约定
     * @param model 输出模型
     * @param dof 自由度数量
     * @param convention DH 约定类型
     * @return 运动学模块状态码
     */
    SerialArmStatus(*model_reset)(SerialArmModel* model, uint8_t dof, SerialArmDhConvention convention);
    /**
     * @brief 设置转动关节连杆参数
     * @param model 目标模型
     * @param index 关节索引，从 0 开始
     * @param theta_home 零位角，单位 rad
     * @param d 固定偏移量，单位 m
     * @param a 连杆长度，单位 m
     * @param alpha 连杆扭角，单位 rad
     * @param q_offset 关节变量偏移
     * @param q_min 关节最小值
     * @param q_max 关节最大值
     * @return 运动学模块状态码
     */
    SerialArmStatus(*model_set_revolute)(SerialArmModel* model, uint8_t index,
        float theta_home, float d, float a, float alpha,
        float q_offset, float q_min, float q_max);
    /**
     * @brief 设置移动关节连杆参数
     * @param model 目标模型
     * @param index 关节索引，从 0 开始
     * @param theta 固定角，单位 rad
     * @param d_home 零位距离，单位 m
     * @param a 连杆长度，单位 m
     * @param alpha 连杆扭角，单位 rad
     * @param q_offset 关节变量偏移
     * @param q_min 关节最小值
     * @param q_max 关节最大值
     * @return 运动学模块状态码
     */
    SerialArmStatus(*model_set_prismatic)(SerialArmModel* model, uint8_t index,
        float theta, float d_home, float a, float alpha,
        float q_offset, float q_min, float q_max);

    /**
     * @brief 载入模型并初始化模块内部状态
     * @param model 已配置完成的模型
     * @return 运动学模块状态码
     */
    SerialArmStatus(*init)(const SerialArmModel* model);
    /**
     * @brief 获取当前自动推断的 IK 任务信息
     * @param info 输出任务信息
     * @return 运动学模块状态码
     */
    SerialArmStatus(*get_task_info)(SerialArmTaskInfo* info);
    /**
     * @brief 获取任务行名称字符串
     * @param row 任务行索引
     * @return 行名称字符串
     */
    const char* (*task_row_name)(uint8_t row);
    /**
     * @brief 获取状态码字符串
     * @param status 状态码
     * @return 状态码字符串
     */
    const char* (*status_str)(SerialArmStatus status);

    /**
     * @brief 计算正运动学，输出末端位姿
     * @param joints 关节数组
     * @param pose 输出位姿
     * @return 运动学模块状态码
     */
    SerialArmStatus(*fk)(const SerialArmJointArray* joints, SerialArmPose* pose);
    /**
     * @brief 计算正运动学，输出 4x4 齐次变换矩阵
     * @param joints 关节数组
     * @param T 输出齐次变换矩阵
     * @return 运动学模块状态码
     */
    SerialArmStatus(*fk_matrix)(const SerialArmJointArray* joints, SerialArmTransform* T);
    /**
     * @brief 计算逆运动学，使用给定 seed 作为初值
     * @param target 目标位姿
     * @param joints 输出关节解
     * @param seed 初始迭代种子
     * @return 运动学模块状态码
     */
    SerialArmStatus(*ik)(const SerialArmPose* target, SerialArmJointArray* joints,
        const SerialArmJointArray* seed);
    /**
     * @brief 尝试搜索全部可行 IK 解
     * @param target 目标位姿
     * @param solutions 输出多解集合
     * @return 运动学模块状态码
     */
    SerialArmStatus(*all_ik)(const SerialArmPose* target, SerialArmJointSolutions* solutions);
    /**
     * @brief 从多解结果中选择指定解
     * @param solutions 多解集合
     * @param index 解索引
     * @return 目标解指针，无效时返回 NULL
     */
    SerialArmJointArray* (*solution_select)(SerialArmJointSolutions* solutions, uint8_t index);

    /**
     * @brief 将欧拉角转换为四元数
     * @param rpy 欧拉角，单位 rad
     * @param quat 输出四元数
     * @return 运动学模块状态码
     */
    SerialArmStatus(*rpy_to_quat)(const SerialArmRPY rpy, SerialArmQuaternion* quat);
    /**
     * @brief 将四元数转换为欧拉角
     * @param quat 输入四元数
     * @param rpy 输出欧拉角，单位 rad
     * @return 运动学模块状态码
     */
    SerialArmStatus(*quat_to_rpy)(const SerialArmQuaternion quat, SerialArmRPY* rpy);
    /**
     * @brief 由 xyz 和 rpy 生成位姿
     * @param x x 坐标，单位 m
     * @param y y 坐标，单位 m
     * @param z z 坐标，单位 m
     * @param roll 横滚角，单位 rad
     * @param pitch 俯仰角，单位 rad
     * @param yaw 偏航角，单位 rad
     * @param pose 输出位姿
     * @return 运动学模块状态码
     */
    SerialArmStatus(*pose_from_xyz_rpy)(float x, float y, float z,
        float roll, float pitch, float yaw, SerialArmPose* pose);
} SerialArmKineInterface;

/**
 * @brief 模块统一接口实例
 */
extern const SerialArmKineInterface serial_arm_kine_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 重置机械臂模型并设置默认参数
 * @param model 输出模型
 * @param dof 自由度数量
 * @param convention DH 约定类型
 * @return 运动学模块状态码
 */
SerialArmStatus s_serial_arm_model_reset(SerialArmModel* model, uint8_t dof, SerialArmDhConvention convention);
/**
 * @brief 设置转动关节的连杆参数
 * @param model 目标模型
 * @param index 关节索引，从 0 开始
 * @param theta_home 零位角，单位 rad
 * @param d 固定偏移量，单位 m
 * @param a 连杆长度，单位 m
 * @param alpha 连杆扭角，单位 rad
 * @param q_offset 关节变量偏移
 * @param q_min 关节最小值
 * @param q_max 关节最大值
 * @return 运动学模块状态码
 */
SerialArmStatus s_serial_arm_model_set_revolute(SerialArmModel* model, uint8_t index,
    float theta_home, float d, float a, float alpha,
    float q_offset, float q_min, float q_max);
/**
 * @brief 设置移动关节的连杆参数
 * @param model 目标模型
 * @param index 关节索引，从 0 开始
 * @param theta 固定角，单位 rad
 * @param d_home 零位距离，单位 m
 * @param a 连杆长度，单位 m
 * @param alpha 连杆扭角，单位 rad
 * @param q_offset 关节变量偏移
 * @param q_min 关节最小值
 * @param q_max 关节最大值
 * @return 运动学模块状态码
 */
SerialArmStatus s_serial_arm_model_set_prismatic(SerialArmModel* model, uint8_t index,
    float theta, float d_home, float a, float alpha,
    float q_offset, float q_min, float q_max);
/**
 * @brief 初始化串联机械臂运动学模块
 * @param model 已配置完成的模型
 * @return 运动学模块状态码
 */
SerialArmStatus s_serial_arm_init(const SerialArmModel* model);
/**
 * @brief 获取当前自动推断的 IK 任务信息
 * @param info 输出任务信息
 * @return 运动学模块状态码
 */
SerialArmStatus s_serial_arm_get_task_info(SerialArmTaskInfo* info);
/**
 * @brief 获取任务行索引对应的名称
 * @param row 任务行索引
 * @return 行名称字符串
 */
const char* s_serial_arm_task_row_name(uint8_t row);
/**
 * @brief 获取状态码对应的字符串
 * @param status 状态码
 * @return 状态码字符串
 */
const char* s_serial_arm_status_str(SerialArmStatus status);
/**
 * @brief 计算末端执行器位姿
 * @param joints 关节数组
 * @param pose 输出位姿
 * @return 运动学模块状态码
 */
SerialArmStatus s_serial_arm_fk(const SerialArmJointArray* joints, SerialArmPose* pose);
/**
 * @brief 计算末端执行器齐次变换矩阵
 * @param joints 关节数组
 * @param T 输出齐次变换矩阵
 * @return 运动学模块状态码
 */
SerialArmStatus s_serial_arm_fk_matrix(const SerialArmJointArray* joints, SerialArmTransform* T);
/**
 * @brief 使用数值法求解逆运动学
 * @param target 目标位姿
 * @param joints 输出关节解
 * @param seed 初始迭代种子
 * @return 运动学模块状态码
 */
SerialArmStatus s_serial_arm_ik(const SerialArmPose* target, SerialArmJointArray* joints,
    const SerialArmJointArray* seed);
/**
 * @brief 尝试搜索全部可行逆运动学解
 * @param target 目标位姿
 * @param solutions 输出多解集合
 * @return 运动学模块状态码
 */
SerialArmStatus s_serial_arm_all_ik(const SerialArmPose* target, SerialArmJointSolutions* solutions);
/**
 * @brief 从多解集合中取出指定解
 * @param solutions 多解集合
 * @param index 解索引
 * @return 目标解指针，无效时返回 NULL
 */
SerialArmJointArray* s_serial_arm_solution_select(SerialArmJointSolutions* solutions, uint8_t index);
/**
 * @brief 将欧拉角转换为四元数
 * @param rpy 欧拉角，单位 rad
 * @param quat 输出四元数
 * @return 运动学模块状态码
 */
SerialArmStatus s_serial_arm_rpy_to_quat(const SerialArmRPY rpy, SerialArmQuaternion* quat);
/**
 * @brief 将四元数转换为欧拉角
 * @param quat 输入四元数
 * @param rpy 输出欧拉角，单位 rad
 * @return 运动学模块状态码
 */
SerialArmStatus s_serial_arm_quat_to_rpy(const SerialArmQuaternion quat, SerialArmRPY* rpy);
/**
 * @brief 由 xyz 和 rpy 生成位姿
 * @param x x 坐标，单位 m
 * @param y y 坐标，单位 m
 * @param z z 坐标，单位 m
 * @param roll 横滚角，单位 rad
 * @param pitch 俯仰角，单位 rad
 * @param yaw 偏航角，单位 rad
 * @param pose 输出位姿
 * @return 运动学模块状态码
 */
SerialArmStatus s_serial_arm_pose_from_xyz_rpy(float x, float y, float z,
    float roll, float pitch, float yaw, SerialArmPose* pose);

#endif
