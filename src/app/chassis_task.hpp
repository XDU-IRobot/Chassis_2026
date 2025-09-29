#ifndef __CHASSIS_TASK_HPP__
#define __CHASSIS_TASK_HPP__

#include "can.h"
#include "cmsis_os.h"

#include "librm.hpp"

using namespace rm;
using namespace rm::device;
using namespace rm::modules::algorithm;
using namespace rm::modules::algorithm::utils;
using rm::hal::Can;
using rm::modules::algorithm::SteeringChassis;

class Chassis {
  Chassis();
  ~Chassis() = default;

 public:
  u8 chassis_mode() { return chassis_mode_data_.chassis_mode; };
  u8 chassis_speed_mode() { return chassis_mode_data_.chassis_speed_mode; };
  u8 buff_state() { return chassis_mode_data_.buff_state; };

 public:
  void ChassisInit();          // 底盘初始化
  void WheelYawAngleChange();  // 底盘舵电机和yaw轴角度变化
  void DataUpdateAndHandle();  // 数据更新与处理

 private:
  // 底盘模式
  typedef enum {
    NOFORCE = 0,         // 无力模式
    FOLLOW = 1,          // 跟随模式
    ROTATE = 2,          // 小陀螺模式
    REVERSE_ROTATE = 3,  // 反向小陀螺模式
    SINGLE_WHEEL = 4,    // 单轮上坡模式
    UNABLE = 5,          // 断电模式
    TEST = 6,            // 测试模式

    DAFU = 7,    // 大符模式
    XIAOFU = 8,  // 小符模式
    NORMAL = 9,  // 正常模式

    NORMAL_SPEED = 10,  // 正常速度模式
    HIGH_SPEED = 11,    // 高速模式
  } ChassisState;

  struct ChassisModeData {
    ChassisState chassis_mode;        // 底盘模式（默认无力）
    ChassisState chassis_speed_mode;  // 底盘速度模式（默认正常速度）
    ChassisState buff_state;          // 打符状态（0：未打符，1：大幅，2：小幅）
    bool high_speed_mode_flag;        // 高速模式标志（按下C键为true, 松开C键为false）
  };

  struct ChassisTargetSpeed {
    f32 speed_x;  // 横向速度
    f32 speed_y;  // 纵向速度
    f32 speed_w;  // 轴向角速度
  };  // 以底盘作为参考系

  struct YawMotorValue {
    f32 yaw_angle_delta;    // yaw轴角度差值
    f32 yaw_angle;          // yaw轴电机编码值
    f32 yaw_initial_angle;  // yaw轴电机初始位置编码值
  };

  struct MotorTargetValue {
    // 0：左前 1：右前 2：右后 3：左后
    f32 m3508_set_speed[4];      // 轮相电机速度设置值
    f32 gm6020_set_position[4];  // 舵向电机位置设置值
    f32 gm6020_set_speed[4];     // 舵向电机速度设置值
  };

  struct MotorCurrentValue {
    // 0：左前 1：右前 2：右后 3：左后
    f32 gm6020_encoder[4];  // 舵向电机编码值
    f32 gm6020_angle[4];    // 舵向电机角度值
    f32 gm6020_current[4];  // GM6020电机电流值
    f32 gm6020_speed[4];    // GM6020电机速度值
    f32 m3508_current[4];   // M3508电机电流值
    f32 m3508_speed[4];     // M3508电机速度值
    f32 chassis_power;      // 底盘功率值
  };

  struct ChassisKeyValue {
    f32 kyaw_speed;                         // yaw轴速度系数（与yaw轴速度相乘）
    f32 kspeed;                             // 速度系数（与速度设置值相乘，通过限制系数赋值）
    f32 knormal_speed_limit;                // 正常的速度限制系数（给速度系数赋值）
    f32 khigh_speed_limit;                  // 高速的速度限制系数（给速度系数赋值）
    f32 kpower_buffer_low_speed_limit;      // 功率缓冲低的速度限制系数（给速度系数赋值）
    f32 kup_slope_front_wheel_speed_limit;  // 上坡前轮速度限制系数（与目标值相乘）
    f32 kup_slope_rear_wheel_speed_limit;   // 上坡后轮速度限制系数（与目标值相乘）
    f32 ksingle_wheel_flont_speed_limit;    // 单轮前轮速度限制系数（与目标值相乘）
    f32 ksingle_wheel_rear_speed_limit;     // 单轮后轮速度限制系数（与目标值相乘）
    f32 ksingle_wheel_left_speed_limit;     // 单轮左轮速度限制系数（与目标值相乘）
    f32 ksingle_wheel_right_speed_limit;    // 单轮右轮速度限制系数（与目标值相乘）
  };

  struct RemoteData {
    f32 speed_set_x;  // 遥控器横向速度设置值
    f32 speed_set_y;  // 遥控器纵向速度设置值
  };

  struct RefereeData {
    i16 remain_bullet_number;    // 剩余子弹量
    i16 allow_fire_bullet;       // 允许发弹量
    i16 last_allow_fire_bullet;  // 上次允许发弹量
    i16 chassis_power_limit;     // 底盘功率上限值
    u16 chassis_power_buffer;    // 底盘功率缓冲值
    f32 chassis_current_power;   // 底盘当前功率
  };

  struct SuperCapData {
    f32 super_cap_voltage;  // 超级电容电压值（0~30V）
    f32 super_cap_current;  // 超级电容电流值（-20~20A）
    bool super_cap_error_flag;  // 超级电容故障标志(超级电容故障时，为true，超级电容正常时，为false)
  };

  // yaw轴电机
  GM6020 motor_yaw_;

  // 底盘电机(左前1, 右前2, 右后3, 左后4)
  GM6020 gm6020_1_;
  GM6020 gm6020_2_;
  GM6020 gm6020_3_;
  GM6020 gm6020_4_;

  M3508 m3508_1_;
  M3508 m3508_2_;
  M3508 m3508_3_;
  M3508 m3508_4_;

  // pid实例化
  RingPID<PIDType::kPosition> yaw_pid_position_;

  RingPID<PIDType::kPosition> gm6020_pid_position_1_;
  RingPID<PIDType::kPosition> gm6020_pid_position_2_;
  RingPID<PIDType::kPosition> gm6020_pid_position_3_;
  RingPID<PIDType::kPosition> gm6020_pid_position_4_;

  PID<PIDType::kPosition> gm6020_pid_speed_1_;
  PID<PIDType::kPosition> gm6020_pid_speed_2_;
  PID<PIDType::kPosition> gm6020_pid_speed_3_;
  PID<PIDType::kPosition> gm6020_pid_speed_4_;

  PID<PIDType::kPosition> m3508_pid_speed_1_;
  PID<PIDType::kPosition> m3508_pid_speed_2_;
  PID<PIDType::kPosition> m3508_pid_speed_3_;
  PID<PIDType::kPosition> m3508_pid_speed_4_;

  RingPID<PIDType::kPosition> gm6020_single_wheel_pid_position_1_;
  RingPID<PIDType::kPosition> gm6020_single_wheel_pid_position_2_;
  RingPID<PIDType::kPosition> gm6020_single_wheel_pid_position_3_;
  RingPID<PIDType::kPosition> gm6020_single_wheel_pid_position_4_;

  PID<PIDType::kPosition> gm6020_single_wheel_pid_speed_1_;
  PID<PIDType::kPosition> gm6020_single_wheel_pid_speed_2_;
  PID<PIDType::kPosition> gm6020_single_wheel_pid_speed_3_;
  PID<PIDType::kPosition> gm6020_single_wheel_pid_speed_4_;

  PID<PIDType::kPosition> m3508_single_wheel_pid_speed_1_;
  PID<PIDType::kPosition> m3508_single_wheel_pid_speed_2_;
  PID<PIDType::kPosition> m3508_single_wheel_pid_speed_3_;
  PID<PIDType::kPosition> m3508_single_wheel_pid_speed_4_;

  SteeringChassis steering_chassis_;

  f32 gm6020_initial_angle_[4];     // 舵电机初始角度
  f32 gm6020_initial_position_[4];  // 舵电机初始位置编码值

  ChassisModeData chassis_mode_data_;        // 底盘模式数据
  ChassisTargetSpeed chassis_target_speed_;  // 底盘目标速度
  YawMotorValue yaw_motor_value_;            // yaw轴电机数据
  MotorTargetValue motor_target_value_;      // 底盘电机目标值
  MotorCurrentValue motor_current_value_;    // 底盘电机当前值
  ChassisKeyValue chassis_key_value_;        // 底盘键值
  RemoteData remote_data_;                   // 遥控器数据
  RefereeData referee_data_;                 // 裁判系统数据
  SuperCapData super_cap_data_;              // 超级电容数据

 private:
  void DataUpdate();                // 数据更新
  void RemainBulletCount();         // 剩余子弹计算
  void SpeedLimitUpdate();          // 速度系数更新
  void PowerLimitLoop();            // 功率闭环
  void SpeedModeChangeJudge();      // 速度模式切换判断
  void ChassisMoveCalculate();      // 底盘移动计算
  void WheelSpeedAngleCalculate();  // 更新解算数据
  void UpSlopeSpeedAdjust();        // 上坡速度调整
  void ChassisCurrentUpdate();      // 底盘电流数据更新
};

#ifdef __cplusplus
extern "C" {
#endif

extern void ChassisTask(const void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_TASK_HPP__ */