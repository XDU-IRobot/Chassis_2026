#include "chassis_task.hpp"
#include "attitude_task.hpp"
#include "communicate_task.hpp"

Chassis *chassis;
Can can1(hcan1);
Can can2(hcan2);

extern INS_t INS;
extern SuperCap *super_cap;
extern GimbalCommunicator *gimbal_communicator;
extern Referee<rm::device::RefereeRevision::kV170> referee_data_buffer;

Chassis::Chassis()
    : motor_yaw_(can1, 4),

      gm6020_1_(can2, 1),
      gm6020_2_(can2, 2),
      gm6020_3_(can2, 3),
      gm6020_4_(can2, 4),

      m3508_1_(can2, 1),
      m3508_2_(can2, 2),
      m3508_3_(can2, 3),
      m3508_4_(can2, 4),
      // yaw轴电机的位置环PID
      yaw_pid_position_(10.0f, 0.0f, 600.0f, 8000.0f, 0.0f, 8191.0f),
      // GM6020电机的位置环PID
      gm6020_pid_position_1_(8.0f, 0.0f, 1.0f, 10000.0f, 0.0f, 8191.0f),
      gm6020_pid_position_2_(8.0f, 0.0f, 1.0f, 10000.0f, 0.0f, 8191.0f),
      gm6020_pid_position_3_(8.0f, 0.0f, 1.0f, 10000.0f, 0.0f, 8191.0f),
      gm6020_pid_position_4_(8.0f, 0.0f, 1.0f, 10000.0f, 0.0f, 8191.0f),
      // GM6020电机的速度环PID
      gm6020_pid_speed_1_(5.0f, 0.0f, 0.1f, 10000.0f, 0.0f),
      gm6020_pid_speed_2_(5.0f, 0.0f, 0.1f, 10000.0f, 0.0f),
      gm6020_pid_speed_3_(5.0f, 0.0f, 0.1f, 10000.0f, 0.0f),
      gm6020_pid_speed_4_(5.0f, 0.0f, 0.1f, 10000.0f, 0.0f),
      // M3508电机的速度单环PID
      m3508_pid_speed_1_(3.0f, 0.0f, 0.5f, 6000.0f, 0.0f),
      m3508_pid_speed_2_(3.0f, 0.0f, 0.5f, 6000.0f, 0.0f),
      m3508_pid_speed_3_(3.0f, 0.0f, 0.5f, 6000.0f, 0.0f),
      m3508_pid_speed_4_(3.0f, 0.0f, 0.5f, 6000.0f, 0.0f),
      // 单轮模式PID（没啥用）
      gm6020_single_wheel_pid_position_1_(12.0f, 0.0f, 1.0f, 10000.0f, 0.0f, 8191.0f),
      gm6020_single_wheel_pid_position_2_(12.0f, 0.0f, 1.0f, 10000.0f, 0.0f, 8191.0f),
      gm6020_single_wheel_pid_position_3_(12.0f, 0.0f, 1.0f, 10000.0f, 0.0f, 8191.0f),
      gm6020_single_wheel_pid_position_4_(12.0f, 0.0f, 1.0f, 10000.0f, 0.0f, 8191.0f),
      gm6020_single_wheel_pid_speed_1_(8.0f, 0.0f, 0.1f, 10000.0f, 0.0f),
      gm6020_single_wheel_pid_speed_2_(8.0f, 0.0f, 0.1f, 10000.0f, 0.0f),
      gm6020_single_wheel_pid_speed_3_(8.0f, 0.0f, 0.1f, 10000.0f, 0.0f),
      gm6020_single_wheel_pid_speed_4_(8.0f, 0.0f, 0.1f, 10000.0f, 0.0f),
      m3508_single_wheel_pid_speed_1_(10.0f, 0.0f, 0.0f, 10000.0f, 0.0f),
      m3508_single_wheel_pid_speed_2_(10.0f, 0.0f, 0.0f, 10000.0f, 0.0f),
      m3508_single_wheel_pid_speed_3_(10.0f, 0.0f, 0.0f, 10000.0f, 0.0f),
      m3508_single_wheel_pid_speed_4_(20.0f, 0.0f, 10.0f, 10000.0f, 0.0f),

      steering_chassis_(0.45368f),

      gm6020_initial_angle_{0.0f, 0.0f, 0.0f, 0.0f},
      gm6020_initial_position_{0.0f, 0.0f, 0.0f, 0.0f},

      chassis_mode_data_{NOFORCE, NORMAL_SPEED, NORMAL, 0},
      chassis_target_speed_{0.0f, 0.0f, 0.0f},
      yaw_motor_value_{0.0f, 0.0f, 0.0f},
      motor_target_value_{{0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f}},
      motor_current_value_{{0.0f, 0.0f, 0.0f, 0.0f},
                           {0.0f, 0.0f, 0.0f, 0.0f},
                           {0.0f, 0.0f, 0.0f, 0.0f},
                           {0.0f, 0.0f, 0.0f, 0.0f},
                           {0.0f, 0.0f, 0.0f, 0.0f},
                           {0.0f, 0.0f, 0.0f, 0.0f},
                           0.0f},
      chassis_key_value_{60.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      remote_data_{0.0f, 0.0f},
      referee_data_{800, 0, 0, 0, 0, 0.0f},
      super_cap_data_{0.0f, 0.0f, false} {}

void Chassis::ChassisInit() {
  can1.SetFilter(0, 0);
  can2.SetFilter(0, 0);
  can1.Begin();
  can2.Begin();
}

void Chassis::WheelYawAngleChange() {
  if (chassis_mode_data_.chassis_mode == SINGLE_WHEEL) {
    yaw_motor_value_.yaw_initial_angle = 4861.0f;
    gm6020_initial_position_[0] = 1100.0f;
    gm6020_initial_position_[1] = 3100.0f;
    gm6020_initial_position_[2] = 2280.0f;
    gm6020_initial_position_[3] = 2920.0f;
  } else {
    yaw_motor_value_.yaw_initial_angle = 5885.0f;
    gm6020_initial_position_[0] = 1100.0f;
    gm6020_initial_position_[1] = 3100.0f;
    gm6020_initial_position_[2] = 2280.0f;
    gm6020_initial_position_[3] = 2920.0f;
  }
}

void Chassis::DataUpdate() {
  // 裁判系统数据更新
  referee_data_.chassis_power_limit = referee_data_buffer.data().robot_status.chassis_power_limit;
  referee_data_.chassis_power_buffer = referee_data_buffer.data().power_heat_data.buffer_energy;
  // 接收云台数据
  // 模式切换
  if (referee_data_buffer.data().robot_status.power_management_chassis_output == 0) {
    chassis_mode_data_.chassis_mode = UNABLE;
  } else {
    if ((gimbal_communicator->chassis_mode() >> 0 & 0x01) == 1) {
      if ((gimbal_communicator->chassis_mode() >> 3 & 0x01) == 1) {
        chassis_mode_data_.chassis_mode = SINGLE_WHEEL;
      } else if ((gimbal_communicator->chassis_mode() >> 1 & 0x01) == 1) {
        chassis_mode_data_.chassis_mode = ROTATE;
      } else if ((gimbal_communicator->chassis_mode() >> 2 & 0x01) == 1) {
        chassis_mode_data_.chassis_mode = REVERSE_ROTATE;
      } else {
        chassis_mode_data_.chassis_mode = FOLLOW;
      }
      if ((gimbal_communicator->chassis_mode() >> 4 & 0x01) == 1) {
        chassis_mode_data_.high_speed_mode_flag = true;
      } else {
        chassis_mode_data_.high_speed_mode_flag = false;
      }
      if ((gimbal_communicator->chassis_mode() >> 5 & 0x01) == 1) {
        chassis_mode_data_.buff_state = DAFU;
      } else if ((gimbal_communicator->chassis_mode() >> 6 & 0x01) == 1) {
        chassis_mode_data_.buff_state = XIAOFU;
      } else {
        chassis_mode_data_.buff_state = NORMAL;
      }
    } else {
      chassis_mode_data_.chassis_mode = NOFORCE;
    }
  }
  // 遥控器数据更新
  remote_data_.speed_set_x = -((f32)gimbal_communicator->remote_speed_x()) / 100.0f;
  remote_data_.speed_set_y = -((f32)gimbal_communicator->remote_speed_y()) / 100.0f;
  // 接收超级电容数据
  super_cap_data_.super_cap_error_flag = super_cap->error(SuperCapError::kOverVoltage);
  super_cap_data_.super_cap_voltage = super_cap->voltage();
  super_cap_data_.super_cap_current = super_cap->current();
  // 接收电机数据
  motor_current_value_.gm6020_encoder[0] = (f32)gm6020_1_.encoder();
  motor_current_value_.gm6020_encoder[1] = (f32)gm6020_2_.encoder();
  motor_current_value_.gm6020_encoder[2] = (f32)gm6020_3_.encoder();
  motor_current_value_.gm6020_encoder[3] = (f32)gm6020_4_.encoder();
  motor_current_value_.gm6020_current[0] = (f32)gm6020_1_.current();
  motor_current_value_.gm6020_current[1] = (f32)gm6020_2_.current();
  motor_current_value_.gm6020_current[2] = (f32)gm6020_3_.current();
  motor_current_value_.gm6020_current[3] = (f32)gm6020_4_.current();
  motor_current_value_.gm6020_speed[0] = (f32)gm6020_1_.rpm();
  motor_current_value_.gm6020_speed[1] = (f32)gm6020_2_.rpm();
  motor_current_value_.gm6020_speed[2] = (f32)gm6020_3_.rpm();
  motor_current_value_.gm6020_speed[3] = (f32)gm6020_4_.rpm();
  motor_current_value_.m3508_current[0] = (f32)m3508_1_.current();
  motor_current_value_.m3508_current[1] = (f32)m3508_2_.current();
  motor_current_value_.m3508_current[2] = (f32)m3508_3_.current();
  motor_current_value_.m3508_current[3] = (f32)m3508_4_.current();
  motor_current_value_.m3508_speed[0] = (f32)m3508_1_.rpm();
  motor_current_value_.m3508_speed[1] = (f32)m3508_2_.rpm();
  motor_current_value_.m3508_speed[2] = (f32)m3508_3_.rpm();
  motor_current_value_.m3508_speed[3] = (f32)m3508_4_.rpm();
  yaw_motor_value_.yaw_angle = motor_yaw_.encoder();
  // yaw轴电机角度
  yaw_motor_value_.yaw_angle_delta =
      (yaw_motor_value_.yaw_initial_angle - yaw_motor_value_.yaw_angle) / 8191.0f * M_PI * 2.0f;
  yaw_motor_value_.yaw_angle_delta = LoopConstrain(yaw_motor_value_.yaw_angle_delta, -M_PI, M_PI);
  // 更新pid控制器对象
  yaw_pid_position_.Update(yaw_motor_value_.yaw_initial_angle, yaw_motor_value_.yaw_angle);
}

void Chassis::RemainBulletCount() {
  referee_data_.allow_fire_bullet = referee_data_buffer.data().projectile_allowance.projectile_allowance_17mm;
  if (referee_data_.allow_fire_bullet <= referee_data_.last_allow_fire_bullet) {
    referee_data_.remain_bullet_number += referee_data_.allow_fire_bullet - referee_data_.last_allow_fire_bullet;
  }
  if (referee_data_buffer.data().game_status.game_progress == 3) {
    referee_data_.remain_bullet_number = 800;
  }
  referee_data_.last_allow_fire_bullet = referee_data_buffer.data().projectile_allowance.projectile_allowance_17mm;
}

void Chassis::SpeedLimitUpdate() {
  if (referee_data_.chassis_power_limit == -1) {
    chassis_key_value_.knormal_speed_limit = 5.5;
    chassis_key_value_.khigh_speed_limit = 6.5;
  } else {
    chassis_key_value_.knormal_speed_limit = referee_data_.chassis_power_limit / 30.0f + 0.6f;
    chassis_key_value_.khigh_speed_limit = referee_data_.chassis_power_limit / 50.0f + 4.0f;
  }
}

void Chassis::PowerLimitLoop() {
  // 缓冲能量过低判断
  if (referee_data_.chassis_power_buffer < 10) {
    chassis_key_value_.kpower_buffer_low_speed_limit = 0.0f;
  } else if (referee_data_.chassis_power_buffer < 60) {
    chassis_key_value_.kpower_buffer_low_speed_limit = pow(referee_data_.chassis_power_buffer / 60.0f, 2);
  } else {
    chassis_key_value_.kpower_buffer_low_speed_limit = 1.0f;
  }
}

void Chassis::SpeedModeChangeJudge() {
  // 模式切换判断
  // 超级电容是否可开启判断
  if (super_cap_data_.super_cap_voltage < 16.0f || super_cap_data_.super_cap_voltage > 35.0f ||
      super_cap_data_.super_cap_error_flag == true || referee_data_.chassis_power_buffer < 30) {
    chassis_mode_data_.chassis_speed_mode = NORMAL_SPEED;
  }
  // 正常模式转高速模式判断
  else if (chassis_mode_data_.high_speed_mode_flag == true && super_cap_data_.super_cap_voltage > 18.0f &&
           referee_data_.chassis_power_buffer > 30) {
    chassis_mode_data_.chassis_speed_mode = HIGH_SPEED;  // 切换到高速模式
  }
  // 高速模式转模式正常判断
  else if (chassis_mode_data_.high_speed_mode_flag == false) {
    chassis_mode_data_.chassis_speed_mode = NORMAL_SPEED;  // 切换到正常模式
  }
  // 切换速度系数
  if (chassis_mode_data_.chassis_speed_mode == HIGH_SPEED) {
    chassis_key_value_.kspeed = chassis_key_value_.khigh_speed_limit;  // 高速模式
  } else {
    chassis_key_value_.kspeed = chassis_key_value_.knormal_speed_limit;  // 正常模式
  }
}

void Chassis::ChassisMoveCalculate() {
  // 计算速度
  // 跟随模式
  if (chassis_mode_data_.chassis_mode == FOLLOW) {
    chassis_target_speed_.speed_y = remote_data_.speed_set_x * cos(yaw_motor_value_.yaw_angle_delta) +
                                    remote_data_.speed_set_y * sin(yaw_motor_value_.yaw_angle_delta);  // 左右速度
    chassis_target_speed_.speed_x = remote_data_.speed_set_y * cos(yaw_motor_value_.yaw_angle_delta) -
                                    remote_data_.speed_set_x * sin(yaw_motor_value_.yaw_angle_delta);  // 前后速度
    if (remote_data_.speed_set_y != 0.0f && remote_data_.speed_set_x != 0.0f) {
      chassis_target_speed_.speed_y *= 1600.0f * chassis_key_value_.kspeed /
                                       sqrt(pow(remote_data_.speed_set_x, 2) + pow(remote_data_.speed_set_y, 2));
      chassis_target_speed_.speed_x *= 1600.0f * chassis_key_value_.kspeed /
                                       sqrt(pow(remote_data_.speed_set_x, 2) + pow(remote_data_.speed_set_y, 2));
    } else {
      chassis_target_speed_.speed_x *= 1600.0f * chassis_key_value_.kspeed;
      chassis_target_speed_.speed_y *= 1600.0f * chassis_key_value_.kspeed;
    }
    // 旋转速度(只转弯不移动，功率最大为35左右)
    if (chassis_mode_data_.buff_state == NORMAL) {
      chassis_target_speed_.speed_w = -yaw_pid_position_.value() + chassis_key_value_.kyaw_speed * motor_yaw_.rpm();
    } else {
      chassis_target_speed_.speed_w = 0;
    }
    chassis_target_speed_.speed_w = Constrain(chassis_target_speed_.speed_w, -20000.0f, 20000.0f);
    if (abs(chassis_target_speed_.speed_w) >= 1000.0f) {
      chassis_target_speed_.speed_y *= pow((20000.0f - abs(chassis_target_speed_.speed_w)) / 20000.0f, 2);
      chassis_target_speed_.speed_x *= pow((20000.0f - abs(chassis_target_speed_.speed_w)) / 20000.0f, 2);
    }
  }
  // 小陀螺模式
  else if (chassis_mode_data_.chassis_mode == ROTATE) {
    chassis_target_speed_.speed_y =
        remote_data_.speed_set_x * cos(yaw_motor_value_.yaw_angle_delta + M_PI / 25.0f) +
        remote_data_.speed_set_y * sin(yaw_motor_value_.yaw_angle_delta + M_PI / 25.0f);  // 左右速度
    chassis_target_speed_.speed_x =
        remote_data_.speed_set_y * cos(yaw_motor_value_.yaw_angle_delta + M_PI / 25.0f) -
        remote_data_.speed_set_x * sin(yaw_motor_value_.yaw_angle_delta + M_PI / 25.0f);  // 前后速度
    chassis_target_speed_.speed_w = 1.0f;                                                 // 旋转速度
    if (remote_data_.speed_set_x != 0.0f || remote_data_.speed_set_y != 0.0f) {
      chassis_target_speed_.speed_w *= 0.4;
    }
    if (remote_data_.speed_set_y != 0.0f && remote_data_.speed_set_x != 0.0f) {
      chassis_target_speed_.speed_y *= 1000.0f * chassis_key_value_.kspeed /
                                       sqrt(pow(remote_data_.speed_set_x, 2) + pow(remote_data_.speed_set_y, 2));
      chassis_target_speed_.speed_x *= 1000.0f * chassis_key_value_.kspeed /
                                       sqrt(pow(remote_data_.speed_set_x, 2) + pow(remote_data_.speed_set_y, 2));
    } else {
      chassis_target_speed_.speed_x *= 1000.0f * chassis_key_value_.kspeed;
      chassis_target_speed_.speed_y *= 1000.0f * chassis_key_value_.kspeed;
    }
    chassis_target_speed_.speed_w *= 4000.0f * chassis_key_value_.kspeed;
  }
  // 反向小陀螺模式
  else if (chassis_mode_data_.chassis_mode == REVERSE_ROTATE) {
    chassis_target_speed_.speed_y =
        remote_data_.speed_set_x * cos(yaw_motor_value_.yaw_angle_delta - M_PI / 20.0f) +
        remote_data_.speed_set_y * sin(yaw_motor_value_.yaw_angle_delta - M_PI / 20.0f);  // 左右速度
    chassis_target_speed_.speed_x =
        remote_data_.speed_set_y * cos(yaw_motor_value_.yaw_angle_delta - M_PI / 20.0f) -
        remote_data_.speed_set_x * sin(yaw_motor_value_.yaw_angle_delta - M_PI / 20.0f);  // 前后速度
    chassis_target_speed_.speed_w = -1.0f;                                                // 旋转速度
    if (remote_data_.speed_set_x != 0.0f || remote_data_.speed_set_y != 0.0f) {
      chassis_target_speed_.speed_w *= 0.3;
    }
    if (remote_data_.speed_set_y != 0.0f && remote_data_.speed_set_x != 0.0f) {
      chassis_target_speed_.speed_y *= 1200.0f * chassis_key_value_.kspeed /
                                       sqrt(pow(remote_data_.speed_set_x, 2) + pow(remote_data_.speed_set_y, 2));
      chassis_target_speed_.speed_x *= 1200.0f * chassis_key_value_.kspeed /
                                       sqrt(pow(remote_data_.speed_set_x, 2) + pow(remote_data_.speed_set_y, 2));
    } else {
      chassis_target_speed_.speed_x *= 1200.0f * chassis_key_value_.kspeed;
      chassis_target_speed_.speed_y *= 1200.0f * chassis_key_value_.kspeed;
    }
    chassis_target_speed_.speed_w *= 3500.0f * chassis_key_value_.kspeed;
  }
  // 单轮上坡模式
  else if (chassis_mode_data_.chassis_mode == SINGLE_WHEEL) {
    chassis_target_speed_.speed_y =
        remote_data_.speed_set_x * cos(yaw_motor_value_.yaw_angle_delta + M_PI / 4.0f) +
        remote_data_.speed_set_y * sin(yaw_motor_value_.yaw_angle_delta + M_PI / 4.0f);  // 左右速度
    chassis_target_speed_.speed_x =
        remote_data_.speed_set_y * cos(yaw_motor_value_.yaw_angle_delta + M_PI / 4.0f) -
        remote_data_.speed_set_x * sin(yaw_motor_value_.yaw_angle_delta + M_PI / 4.0f);  // 前后速度
    if (remote_data_.speed_set_y != 0.0f && remote_data_.speed_set_x != 0.0f) {
      chassis_target_speed_.speed_y *= 1600.0f * chassis_key_value_.kspeed /
                                       sqrt(pow(remote_data_.speed_set_x, 2) + pow(remote_data_.speed_set_y, 2));
      chassis_target_speed_.speed_x *= 1600.0f * chassis_key_value_.kspeed /
                                       sqrt(pow(remote_data_.speed_set_x, 2) + pow(remote_data_.speed_set_y, 2));
    } else {
      chassis_target_speed_.speed_x *= 1600.0f * chassis_key_value_.kspeed;
      chassis_target_speed_.speed_y *= 1600.0f * chassis_key_value_.kspeed;
    }
    // 旋转速度(只转弯不移动，功率最大为35左右)
    chassis_target_speed_.speed_w = -yaw_pid_position_.value() + chassis_key_value_.kyaw_speed * motor_yaw_.rpm();
    chassis_target_speed_.speed_w = Constrain(chassis_target_speed_.speed_w, -20000.0f, 20000.0f);
    if (abs(chassis_target_speed_.speed_w) >= 1500.0f) {
      chassis_target_speed_.speed_y *= pow((20000.0f - abs(chassis_target_speed_.speed_w)) / 20000.0f, 2);
      chassis_target_speed_.speed_x *= pow((20000.0f - abs(chassis_target_speed_.speed_w)) / 20000.0f, 2);
    }
  }
  // 无力模式
  else {
    chassis_target_speed_.speed_x = 0.0f;
    chassis_target_speed_.speed_y = 0.0f;
    chassis_target_speed_.speed_w = 0.0f;
  }
}

void Chassis::WheelSpeedAngleCalculate() {
  for (int i = 0; i < 4; i++) {
    if ((i == 1 || i == 3) && chassis_mode_data_.chassis_mode == SINGLE_WHEEL &&
        chassis_target_speed_.speed_x == 0.0f && chassis_target_speed_.speed_y == 0.0f &&
        abs(chassis_target_speed_.speed_w) < 800.0f) {
      gm6020_initial_position_[i] += 2048.0f;
    }
    gm6020_initial_angle_[i] = Map(gm6020_initial_position_[i], 0.0f, 8191.0f, 0.0f, 2 * M_PI);
    gm6020_initial_angle_[i] = LoopConstrain(gm6020_initial_angle_[i], -M_PI, M_PI);
    motor_current_value_.gm6020_encoder[i] = Map(motor_current_value_.gm6020_encoder[i], 0.0f, 8191.0f, 0.0f, 2 * M_PI);
    motor_current_value_.gm6020_angle[i] = motor_current_value_.gm6020_encoder[i] - gm6020_initial_angle_[i];
    motor_current_value_.gm6020_angle[i] = LoopConstrain(motor_current_value_.gm6020_angle[i], -M_PI, M_PI);
  }
  // 库解算
  auto chassis_calculate_result = steering_chassis_.Forward(
      chassis_target_speed_.speed_x, chassis_target_speed_.speed_y, chassis_target_speed_.speed_w,
      motor_current_value_.gm6020_angle[0], motor_current_value_.gm6020_angle[1], motor_current_value_.gm6020_angle[3],
      motor_current_value_.gm6020_angle[2]);
  // 解算轮向电机角度
  motor_target_value_.gm6020_set_position[0] = chassis_calculate_result.lf_steer_position + gm6020_initial_angle_[0];
  motor_target_value_.gm6020_set_position[1] = chassis_calculate_result.rf_steer_position + gm6020_initial_angle_[1];
  motor_target_value_.gm6020_set_position[2] = chassis_calculate_result.rr_steer_position + gm6020_initial_angle_[2];
  motor_target_value_.gm6020_set_position[3] = chassis_calculate_result.lr_steer_position + gm6020_initial_angle_[3];
  for (int i = 0; i < 4; i++) {
    motor_target_value_.gm6020_set_position[i] = LoopConstrain(motor_target_value_.gm6020_set_position[i], 0, 2 * M_PI);
    motor_target_value_.gm6020_set_position[i] =
        Map(motor_target_value_.gm6020_set_position[i], 0.0f, 2 * M_PI, 0.0f, 8191.0f);
  }
  // 解算轮向电机速度
  motor_target_value_.m3508_set_speed[0] = -chassis_calculate_result.lf_wheel_speed;
  motor_target_value_.m3508_set_speed[1] = chassis_calculate_result.rf_wheel_speed;
  motor_target_value_.m3508_set_speed[2] = chassis_calculate_result.rr_wheel_speed;
  motor_target_value_.m3508_set_speed[3] = -chassis_calculate_result.lr_wheel_speed;
}

void Chassis::UpSlopeSpeedAdjust() {
  if (chassis_mode_data_.chassis_mode == FOLLOW) {
    if (INS.pitch > 5.0f) {
      chassis_key_value_.kup_slope_front_wheel_speed_limit = 1.0f - (INS.pitch / 60.0f);
      chassis_key_value_.kup_slope_rear_wheel_speed_limit = 1.0f + (INS.pitch / 30.0f);
    } else if (INS.pitch < -5.0f) {
      chassis_key_value_.kup_slope_front_wheel_speed_limit = 1.0f - (INS.pitch / 30.0f);
      chassis_key_value_.kup_slope_rear_wheel_speed_limit = 1.0f + (INS.pitch / 60.0f);
    } else {
      chassis_key_value_.kup_slope_front_wheel_speed_limit = 1.0f;
      chassis_key_value_.kup_slope_rear_wheel_speed_limit = 1.0f;
    }
    chassis_key_value_.kup_slope_front_wheel_speed_limit =
        LoopConstrain(chassis_key_value_.kup_slope_front_wheel_speed_limit, 0.1f, 3.0f);
    chassis_key_value_.kup_slope_rear_wheel_speed_limit =
        LoopConstrain(chassis_key_value_.kup_slope_rear_wheel_speed_limit, 0.1f, 3.0f);
    motor_target_value_.m3508_set_speed[0] *= chassis_key_value_.kup_slope_front_wheel_speed_limit;
    motor_target_value_.m3508_set_speed[1] *= chassis_key_value_.kup_slope_front_wheel_speed_limit;
    motor_target_value_.m3508_set_speed[2] *= chassis_key_value_.kup_slope_rear_wheel_speed_limit;
    motor_target_value_.m3508_set_speed[3] *= chassis_key_value_.kup_slope_rear_wheel_speed_limit;
  } else if (chassis_mode_data_.chassis_mode == SINGLE_WHEEL) {
    if (INS.pitch > 5.0f && INS.roll > 5.0f) {
      chassis_key_value_.ksingle_wheel_flont_speed_limit = 0.5f;
      chassis_key_value_.ksingle_wheel_rear_speed_limit = 8.0f;
      chassis_key_value_.ksingle_wheel_left_speed_limit = 0.2;
      chassis_key_value_.ksingle_wheel_right_speed_limit = 0.2;
    } else {
      chassis_key_value_.ksingle_wheel_flont_speed_limit = 1.0f;
      chassis_key_value_.ksingle_wheel_rear_speed_limit = 1.0f;
      chassis_key_value_.ksingle_wheel_left_speed_limit = 1.0f;
      chassis_key_value_.ksingle_wheel_right_speed_limit = 1.0f;
    }
    motor_target_value_.m3508_set_speed[0] *= chassis_key_value_.ksingle_wheel_left_speed_limit;
    motor_target_value_.m3508_set_speed[1] *= chassis_key_value_.ksingle_wheel_flont_speed_limit;
    motor_target_value_.m3508_set_speed[2] *= chassis_key_value_.ksingle_wheel_right_speed_limit;
    motor_target_value_.m3508_set_speed[3] *= chassis_key_value_.ksingle_wheel_rear_speed_limit;
  }
}

void Chassis::ChassisCurrentUpdate() {
  // 跟随模式
  if (chassis_mode_data_.chassis_mode == FOLLOW || chassis_mode_data_.chassis_mode == ROTATE ||
      chassis_mode_data_.chassis_mode == REVERSE_ROTATE) {
    // 更新pid控制器输出
    if ((abs(m3508_1_.rpm()) >= 50 || abs(m3508_2_.rpm()) >= 50 || abs(m3508_3_.rpm()) >= 50 ||
         abs(m3508_4_.rpm()) >= 50) &&
        chassis_target_speed_.speed_y == 0 && chassis_target_speed_.speed_x == 0 &&
        abs(chassis_target_speed_.speed_w) <= 1200) {
      // GM6020电机的速度环PID数据更新
      gm6020_pid_speed_1_.Update(0, gm6020_1_.rpm());
      gm6020_pid_speed_2_.Update(0, gm6020_2_.rpm());
      gm6020_pid_speed_3_.Update(0, gm6020_3_.rpm());
      gm6020_pid_speed_4_.Update(0, gm6020_4_.rpm());
    } else {
      // GM6020电机的位置环PID数据更新
      gm6020_pid_position_1_.Update(motor_target_value_.gm6020_set_position[0], gm6020_1_.encoder());
      gm6020_pid_position_2_.Update(motor_target_value_.gm6020_set_position[1], gm6020_2_.encoder());
      gm6020_pid_position_3_.Update(motor_target_value_.gm6020_set_position[2], gm6020_3_.encoder());
      gm6020_pid_position_4_.Update(motor_target_value_.gm6020_set_position[3], gm6020_4_.encoder());
      // GM6020电机的速度环PID数据更新
      gm6020_pid_speed_1_.Update(gm6020_pid_position_1_.value(), gm6020_1_.rpm());
      gm6020_pid_speed_2_.Update(gm6020_pid_position_2_.value(), gm6020_2_.rpm());
      gm6020_pid_speed_3_.Update(gm6020_pid_position_3_.value(), gm6020_3_.rpm());
      gm6020_pid_speed_4_.Update(gm6020_pid_position_4_.value(), gm6020_4_.rpm());
    }
    // M3508电机的速度环PID数据更新
    m3508_pid_speed_1_.Update(motor_target_value_.m3508_set_speed[0], m3508_1_.rpm());
    m3508_pid_speed_2_.Update(motor_target_value_.m3508_set_speed[1], m3508_2_.rpm());
    m3508_pid_speed_3_.Update(motor_target_value_.m3508_set_speed[2], m3508_3_.rpm());
    m3508_pid_speed_4_.Update(motor_target_value_.m3508_set_speed[3], m3508_4_.rpm());

    // 设置电机电流
    // GM6020电机电流设置
    gm6020_1_.SetCurrent(gm6020_pid_speed_1_.value() * chassis_key_value_.kpower_buffer_low_speed_limit);
    gm6020_2_.SetCurrent(gm6020_pid_speed_2_.value() * chassis_key_value_.kpower_buffer_low_speed_limit);
    gm6020_3_.SetCurrent(gm6020_pid_speed_3_.value() * chassis_key_value_.kpower_buffer_low_speed_limit);
    gm6020_4_.SetCurrent(gm6020_pid_speed_4_.value() * chassis_key_value_.kpower_buffer_low_speed_limit);
    // M3508电机电流设置
    m3508_1_.SetCurrent(m3508_pid_speed_1_.value() * chassis_key_value_.kpower_buffer_low_speed_limit);
    m3508_2_.SetCurrent(m3508_pid_speed_2_.value() * chassis_key_value_.kpower_buffer_low_speed_limit);
    m3508_3_.SetCurrent(m3508_pid_speed_3_.value() * chassis_key_value_.kpower_buffer_low_speed_limit);
    m3508_4_.SetCurrent(m3508_pid_speed_4_.value() * chassis_key_value_.kpower_buffer_low_speed_limit);
  }
  // 单轮上坡模式
  else if (chassis_mode_data_.chassis_mode == SINGLE_WHEEL) {
    // GM6020电机的位置环PID数据更新
    gm6020_single_wheel_pid_position_1_.Update(motor_target_value_.gm6020_set_position[0], gm6020_1_.encoder());
    gm6020_single_wheel_pid_position_2_.Update(motor_target_value_.gm6020_set_position[1], gm6020_2_.encoder());
    gm6020_single_wheel_pid_position_3_.Update(motor_target_value_.gm6020_set_position[2], gm6020_3_.encoder());
    gm6020_single_wheel_pid_position_4_.Update(motor_target_value_.gm6020_set_position[3], gm6020_4_.encoder());
    // GM6020电机的速度环PID数据更新
    gm6020_single_wheel_pid_speed_1_.Update(gm6020_single_wheel_pid_position_1_.value(), gm6020_1_.rpm());
    gm6020_single_wheel_pid_speed_2_.Update(gm6020_single_wheel_pid_position_2_.value(), gm6020_2_.rpm());
    gm6020_single_wheel_pid_speed_3_.Update(gm6020_single_wheel_pid_position_3_.value(), gm6020_3_.rpm());
    gm6020_single_wheel_pid_speed_4_.Update(gm6020_single_wheel_pid_position_4_.value(), gm6020_4_.rpm());

    // M3508电机的速度环PID数据更新
    m3508_single_wheel_pid_speed_1_.Update(motor_target_value_.m3508_set_speed[0], m3508_1_.rpm());
    m3508_single_wheel_pid_speed_2_.Update(motor_target_value_.m3508_set_speed[1], m3508_2_.rpm());
    m3508_single_wheel_pid_speed_3_.Update(motor_target_value_.m3508_set_speed[2], m3508_3_.rpm());
    m3508_single_wheel_pid_speed_4_.Update(motor_target_value_.m3508_set_speed[3], m3508_4_.rpm());

    // 设置电机电流
    // GM6020电机电流设置
    gm6020_1_.SetCurrent(gm6020_single_wheel_pid_speed_1_.value() * chassis_key_value_.kpower_buffer_low_speed_limit);
    gm6020_2_.SetCurrent(gm6020_single_wheel_pid_speed_2_.value() * chassis_key_value_.kpower_buffer_low_speed_limit);
    gm6020_3_.SetCurrent(gm6020_single_wheel_pid_speed_3_.value() * chassis_key_value_.kpower_buffer_low_speed_limit);
    gm6020_4_.SetCurrent(gm6020_single_wheel_pid_speed_4_.value() * chassis_key_value_.kpower_buffer_low_speed_limit);
    // M3508电机电流设置
    m3508_1_.SetCurrent(m3508_single_wheel_pid_speed_1_.value() * chassis_key_value_.kpower_buffer_low_speed_limit);
    m3508_2_.SetCurrent(m3508_single_wheel_pid_speed_2_.value() * chassis_key_value_.kpower_buffer_low_speed_limit);
    m3508_3_.SetCurrent(m3508_single_wheel_pid_speed_3_.value() * chassis_key_value_.kpower_buffer_low_speed_limit);
    m3508_4_.SetCurrent(m3508_single_wheel_pid_speed_4_.value() * chassis_key_value_.kpower_buffer_low_speed_limit);
  }
  // 无力模式
  else {
    // 设置电机电流
    // GM6020电机电流设置
    gm6020_1_.SetCurrent(0);
    gm6020_2_.SetCurrent(0);
    gm6020_3_.SetCurrent(0);
    gm6020_4_.SetCurrent(0);
    // M3508电机电流设置
    m3508_1_.SetCurrent(0);
    m3508_2_.SetCurrent(0);
    m3508_3_.SetCurrent(0);
    m3508_4_.SetCurrent(0);
  }
  // 发送控制数据
  DjiMotor<>::SendCommand();
}

void Chassis::DataUpdateAndHandle() {
  WheelYawAngleChange();
  DataUpdate();
  RemainBulletCount();
  SpeedLimitUpdate();
  PowerLimitLoop();
  SpeedModeChangeJudge();
  ChassisMoveCalculate();
  WheelSpeedAngleCalculate();
  UpSlopeSpeedAdjust();
  ChassisCurrentUpdate();
}

extern "C" {
void ChassisTask(const void *argument) {
  chassis = new Chassis();
  chassis->ChassisInit();
  while (1) {
    chassis->DataUpdateAndHandle();
    osDelay(1);
  }
}
}
