#include "referee_task.hpp"
#include "rc_referee_data.hpp"
#include "communicate_task.hpp"
#include "chassis_task.hpp"

static UIData *UI_data;
static Serial referee_uart(huart6, 128, rm::hal::stm32::UartMode::kDma, rm::hal::stm32::UartMode::kDma);

extern Chassis *chassis;
extern GimbalCommunicator *gimbal_communicator;

extern UI ui;
extern u16 robot_id;

extern Referee<rm::device::RefereeRevision::kV170> referee_data_buffer;

void UIData::UIDataInit() {
  Line_Draw(&imagex, "x", UI_Graph_ADD, 0, UI_Color_Orange, 2, 918, 515, 978, 515);
  Line_Draw(&imagey, "y", UI_Graph_ADD, 0, UI_Color_Orange, 2, 948, 465, 948, 565);

  Float_Draw(&super_cap_energy, "cms", UI_Graph_ADD, 2, UI_Color_Green, 27, 2, 5, 900, 270,
             (f32)chassis->super_cap_voltage() * 1000.0f);
  Float_Draw(&ammo_speed_jugde, "asj", UI_Graph_ADD, 2, UI_Color_White, 25, 2, 2, 360, 850,
             (f32)gimbal_communicator->aim_speed_change() * 1000.0f);
  Float_Draw(&remain_bullet, "rbn", UI_Graph_ADD, 2, UI_Color_White, 25, 2, 2, 1362, 475,
             (f32)chassis->remain_bullet_number() * 1000.0f);

  Rectangle_Draw(&get_target_flag, "gtf", UI_Graph_ADD, 2, UI_Color_Purplish_red, 0, 360, 800, 420, 750);
  Rectangle_Draw(&suggest_fire_flag, "sff", UI_Graph_ADD, 2, UI_Color_Purplish_red, 0, 360, 750, 420, 700);
  Rectangle_Draw(&chassis_mode_flag, "cmf", UI_Graph_ADD, 2, UI_Color_Purplish_red, 0, 6200, 730, 6350, 700);
  Rectangle_Draw(&buff_mode_flag, "bmf", UI_Graph_ADD, 2, UI_Color_Purplish_red, 0, 6200, 730, 6350, 700);
  Rectangle_Draw(&speed_mode_flag, "smf", UI_Graph_ADD, 2, UI_Color_Purplish_red, 0, 6200, 770, 6350, 730);

  Char_Draw(&aimbot, "aim", UI_Graph_ADD, 1, UI_Color_Green, 25, 22, 2, 360, 800, "GETTARGET\nSUGGESTFIRE");
  Char_Draw(&mode, "mod", UI_Graph_ADD, 1, UI_Color_Green, 25, 20, 2, 1300, 800, "F R N U S D X\nH N S");

  irq = (u32)&aimbot;
  EnQueue(&UI_send_buffer[1], (u8 *)&irq, 4);
  irq = (u32)&mode;
  EnQueue(&UI_send_buffer[1], (u8 *)&irq, 4);
  irq = (u32)&imagex;
  EnQueue(&UI_send_buffer[1], (u8 *)&irq, 4);
  irq = (u32)&imagey;
  EnQueue(&UI_send_buffer[1], (u8 *)&irq, 4);
  irq = (u32)&super_cap_energy;
  EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
  irq = (u32)&ammo_speed_jugde;
  EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
  irq = (u32)&remain_bullet;
  EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
  irq = (u32)&get_target_flag;
  EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
  irq = (u32)&suggest_fire_flag;
  EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
  irq = (u32)&chassis_mode_flag;
  EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
  irq = (u32)&buff_mode_flag;
  EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
  irq = (u32)&speed_mode_flag;
  EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
}
void UIData::UIDataUpdate() {
  // 电容电压
  if (chassis->chassis_speed_mode() == chassis->HIGH_SPEED) {
    Float_Draw(&super_cap_energy, "cms", UI_Graph_Change, 2, UI_Color_Green, 27, 1, 5, 900, 270,
               (f32)chassis->super_cap_voltage() * 1000.0f);
  } else {
    Float_Draw(&super_cap_energy, "cms", UI_Graph_Change, 2, UI_Color_Main, 27, 1, 5, 900, 270,
               (f32)chassis->super_cap_voltage() * 1000.0f);
  }
  // 弹速调节
  if (gimbal_communicator->aim_speed_change() > 0) {
    Float_Draw(&ammo_speed_jugde, "asj", UI_Graph_Change, 2, UI_Color_Green, 25, 2, 2, 360, 850,
               (f32)gimbal_communicator->aim_speed_change() * 1000.0f);
  } else if (gimbal_communicator->aim_speed_change() < 0) {
    Float_Draw(&ammo_speed_jugde, "asj", UI_Graph_Change, 2, UI_Color_Pink, 25, 2, 2, 360, 850,
               (f32)gimbal_communicator->aim_speed_change() * 1000.0f);
  } else {
    Float_Draw(&ammo_speed_jugde, "asj", UI_Graph_Change, 2, UI_Color_White, 25, 2, 2, 360, 850,
               (f32)gimbal_communicator->aim_speed_change() * 1000.0f);
  }
  // 剩余子弹
  if (chassis->remain_bullet_number() < 0) {
    Float_Draw(&remain_bullet, "rbn", UI_Graph_Change, 2, UI_Color_Pink, 25, 2, 2, 1362, 475,
               (f32)chassis->remain_bullet_number() * 1000.0f);
  } else if (chassis->remain_bullet_number() < 100) {
    Float_Draw(&remain_bullet, "rbn", UI_Graph_Change, 2, UI_Color_Orange, 25, 2, 2, 1362, 475,
               (f32)chassis->remain_bullet_number() * 1000.0f);
  } else {
    Float_Draw(&remain_bullet, "rbn", UI_Graph_Change, 2, UI_Color_Green, 25, 2, 2, 1362, 475,
               (f32)chassis->remain_bullet_number() * 1000.0f);
  }
  // 自瞄模式
  if (gimbal_communicator->get_target_flag() == 1) {
    Rectangle_Draw(&get_target_flag, "gtf", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 350, 807, 580, 768);
  } else {
    Rectangle_Draw(&get_target_flag, "gtf", UI_Graph_Change, 2, UI_Color_Purplish_red, 0, 350, 807, 580, 768);
  }

  if (gimbal_communicator->suggest_fire_flag() == 1) {
    Rectangle_Draw(&suggest_fire_flag, "sff", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 350, 766, 630, 731);
  } else {
    Rectangle_Draw(&suggest_fire_flag, "sff", UI_Graph_Change, 2, UI_Color_Purplish_red, 0, 350, 766, 630, 731);
  }

  // 底盘模式
  if (chassis->chassis_mode() == chassis->FOLLOW) {
    Rectangle_Draw(&chassis_mode_flag, "cmf", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 1292, 808, 1322, 770);
  } else if (chassis->chassis_mode() == chassis->ROTATE) {
    Rectangle_Draw(&chassis_mode_flag, "cmf", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 1342, 808, 1372, 770);
  } else if (chassis->chassis_mode() == chassis->NOFORCE) {
    Rectangle_Draw(&chassis_mode_flag, "cmf", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 1392, 808, 1422, 770);
  } else if (chassis->chassis_mode() == chassis->SINGLE_WHEEL) {
    Rectangle_Draw(&chassis_mode_flag, "cmf", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 1492, 808, 1522, 770);
  } else {
    Rectangle_Draw(&chassis_mode_flag, "cmf", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 1442, 808, 1472, 770);
  }
  if (chassis->buff_state() == chassis->DAFU) {
    Rectangle_Draw(&buff_mode_flag, "bmf", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 1542, 808, 1572, 770);
  } else if (chassis->buff_state() == chassis->XIAOFU) {
    Rectangle_Draw(&buff_mode_flag, "bmf", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 1592, 808, 1622, 770);
  } else {
    Rectangle_Draw(&buff_mode_flag, "bmf", UI_Graph_Change, 2, UI_Color_Purplish_red, 0, 1542, 808, 1572, 770);
  }

  // 底盘速度模式
  if (chassis->chassis_speed_mode() == chassis->HIGH_SPEED) {
    Rectangle_Draw(&speed_mode_flag, "smf", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 1292, 766, 1322, 728);
  } else {
    Rectangle_Draw(&speed_mode_flag, "smf", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 1342, 766, 1372, 728);
  }
  if (referee_data_buffer.data().power_heat_data.buffer_energy < 40) {
    Rectangle_Draw(&speed_mode_flag, "smf", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 1392, 766, 1422, 728);
  }

  irq = (u32)&super_cap_energy;
  EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
  irq = (u32)&ammo_speed_jugde;
  EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
  irq = (u32)&remain_bullet;
  EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
  irq = (u32)&get_target_flag;
  EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
  irq = (u32)&suggest_fire_flag;
  EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
  irq = (u32)&chassis_mode_flag;
  EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
  irq = (u32)&buff_mode_flag;
  EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
  irq = (u32)&speed_mode_flag;
  EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
}

void UIData::UIDataSend() {
  while (!IsEmpty(&UI_send_buffer[0])) {
    if (UI_send_buffer[0].counter / 4 >= 7) {
      for (u8 i = 0; i < 7; i++) {
        UI_Pop(&UI_send_buffer[0], (u8 *)&tmp_send[i]);
      }
      UI_ReFresh(7, *(Graph_Data *)tmp_send[0], *(Graph_Data *)tmp_send[1], *(Graph_Data *)tmp_send[2],
                 *(Graph_Data *)tmp_send[3], *(Graph_Data *)tmp_send[4], *(Graph_Data *)tmp_send[5],
                 *(Graph_Data *)tmp_send[6]);
    } else if (UI_send_buffer[0].counter / 4 >= 5) {
      for (u8 i = 0; i < 5; i++) {
        UI_Pop(&UI_send_buffer[0], (u8 *)&tmp_send[i]);
      }
      UI_ReFresh(5, *(Graph_Data *)tmp_send[0], *(Graph_Data *)tmp_send[1], *(Graph_Data *)tmp_send[2],
                 *(Graph_Data *)tmp_send[3], *(Graph_Data *)tmp_send[4]);
    } else if (UI_send_buffer[0].counter / 4 >= 2) {
      for (u8 i = 0; i < 2; i++) {
        UI_Pop(&UI_send_buffer[0], (u8 *)&tmp_send[i]);
      }
      UI_ReFresh(2, *(Graph_Data *)tmp_send[0], *(Graph_Data *)tmp_send[1]);
    } else {
      UI_Pop(&UI_send_buffer[0], (u8 *)&tmp_send[0]);
      UI_ReFresh(1, *(Graph_Data *)tmp_send[0]);
    }
    DataSend(referee_uart, ui.info_arr, ui.len);
    osDelay(50);
  }

  while (!IsEmpty(&UI_send_buffer[1])) {
    UI_Pop(&UI_send_buffer[1], (u8 *)&tmp_send[0]);
    Char_ReFresh(*(String_Data *)tmp_send[0]);
    DataSend(referee_uart, ui.info_arr, ui.len);
    osDelay(50);
  }
}

void UIData::DataSend(Serial msg, u8 *data, u8 len) { msg.Write(data, len); }

extern "C" {
void RefereeTask(void const *argument) {
  // Serial referee_uart(huart6, 128, rm::hal::stm32::UartMode::kDma, rm::hal::stm32::UartMode::kDma);
  RcTcRefereeData referee_data_rc_tc(referee_uart);
  referee_data_rc_tc.Begin();
  // 初始化队列
  QueueInit(&UI_data->UI_send_buffer[0]);
  QueueInit(&UI_data->UI_send_buffer[1]);
  while (1) {
    robot_id = referee_data_buffer.data().robot_status.robot_id;
    // if (gimbal_communicator->UI_show_flag == 1) {
    //   UI_data->UIDataInit();
    //   UI_data->UIDataSend();
    // } else {
    //   UI_data->UIDataUpdate();
    //   UI_data->UIDataSend();
    // }
  }
}
}
