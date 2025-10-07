#ifndef __REFEREE_TASK_HPP__
#define __REFEREE_TASK_HPP__

#include "usart.h"

#include "UI.hpp"
#include "queue.hpp"

#include "librm.hpp"

using namespace rm;
using namespace rm::hal;
using namespace rm::device;

class UIData {
 public:
  UIData() = default;
  ~UIData() = default;

  void UIDataInit();
  void UIDataUpdate();
  void UIDataSend();

 private:
  u32 irq;
  u32 tmp_send[7];
  // 电容电压
  Float_Data super_cap_energy, ammo_speed_jugde, remain_bullet;
  // 模式
  Graph_Data chassis_mode_flag, buff_mode_flag, speed_mode_flag, get_target_flag, suggest_fire_flag;
  String_Data mode, aimbot;
  // 瞄准线
  Graph_Data imagey, imagex;
  // UI数据发送
  void DataSend(Serial msg, u8 *data, u8 len);
};

#ifdef __cplusplus
extern "C" {
#endif

extern void RefereeTask(void const *argument);

#ifdef __cplusplus
}
#endif

#endif /* __REFEREE_TASK_HPP__ */