#ifndef __COMMUNICATE_TASK_HPP__
#define __COMMUNICATE_TASK_HPP__

#include "can.h"
#include "usart.h"
#include "cmsis_os.h"

#include "librm.hpp"

using namespace rm;
using namespace rm::hal;
using namespace rm::device;

#ifdef __cplusplus
extern "C" {
#endif

extern void CommunicateTask(const void *argument);

#ifdef __cplusplus
}
#endif

class GimbalCommunicator : public CanDevice {
  explicit GimbalCommunicator(CanInterface &can);
  GimbalCommunicator() = delete;
  ~GimbalCommunicator() override = default;

 public:
  i8 remote_speed_x() { return gimbal_data_feedback_.remote_speed_x; }
  i8 remote_speed_y() { return gimbal_data_feedback_.remote_speed_y; }
  u8 chassis_mode() { return gimbal_data_feedback_.chassis_mode; }
  u8 UI_show_flag() { return gimbal_data_feedback_.UI_show_flag; }
  u8 get_target_flag() { return gimbal_data_feedback_.get_target_flag; }
  u8 suggest_fire_flag() { return gimbal_data_feedback_.suggest_fire_flag; }
  i8 aim_speed_change() { return gimbal_data_feedback_.aim_speed_change; }

 public:
  void RxCallback(const CanMsg *msg) override;  // 接收数据
  void SendGimbalData();                        // 发送云台数据
  void SendSuperCapData();                      // 发送超级电容数据
  void comunicate_data_update();                // 数据更新

 private:
  // 发送给超级电容的信息
  struct TxSuperCapData {
    i16 chassis_power_limit;           // 底盘功率上限值
    i16 chassis_power_buffer;          // 底盘功率缓冲值
    u8 chassis_power_flag;             // 底盘供电情况
    bool super_cap_power_switch;       // 电容开关
    bool super_cap_enable_log_switch;  // 记录开关
  } tx_super_cap_data_;

  // 发送给云台的信息
  struct TxGimbalData {
    u8 tx_buf_gimbal[8]{0};    // 发送给云台的缓冲区1
    u8 robot_id;               // 机器人基本数据
    u8 power_state : 4;        // 电源状态
    u16 heat_limit;            // 枪口热量上限值
    u16 current_heat;          // 当前枪口热量
    u16 current_cooling_heat;  // 当前枪口冷却
    u8 cooling_mag;            // 枪口热量冷却倍率
  } tx_gimbal_data_;

  // 接收云台信息
  struct RxGimbalData {
    i8 remote_speed_x;     // 遥控器发送的x轴速度
    i8 remote_speed_y;     // 遥控器发送的y轴速度
    u8 chassis_mode;       // 底盘模式
    u8 UI_show_flag;       // UI显示标志
    u8 get_target_flag;    // 自瞄是否已启动
    u8 suggest_fire_flag;  // 建议开火
    i8 aim_speed_change;   // 射击弹速调整
    u8 reserve[2];         // 保留位
  } gimbal_data_feedback_;
};

#endif /* __COMMUNICATE_TASK_HPP__ */