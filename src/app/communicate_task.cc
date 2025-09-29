#include "communicate_task.hpp"
#include "chassis_task.hpp"

SuperCap *super_cap;
GimbalCommunicator *gimbal_communicator;
GimbalCommunicator::GimbalCommunicator(CanInterface &can) : CanDevice(can, 0x120) {}

extern Chassis *chassis;
extern Can can1;
extern Referee<RefereeRevision::kV170> referee_data_buffer;

void GimbalCommunicator::comunicate_data_update() {
  // 接收裁判系统数据
  tx_gimbal_data_.robot_id = referee_data_buffer.data().robot_status.robot_id;
  tx_gimbal_data_.power_state = referee_data_buffer.data().robot_status.power_management_gimbal_output |
                                referee_data_buffer.data().robot_status.power_management_chassis_output << 1 |
                                referee_data_buffer.data().robot_status.power_management_shooter_output << 2;
  tx_gimbal_data_.heat_limit = referee_data_buffer.data().robot_status.shooter_barrel_heat_limit;
  tx_gimbal_data_.current_heat = referee_data_buffer.data().power_heat_data.shooter_17mm_1_barrel_heat;
  tx_gimbal_data_.current_cooling_heat = referee_data_buffer.data().robot_status.shooter_barrel_cooling_value;
  tx_gimbal_data_.cooling_mag = referee_data_buffer.data().buff.cooling_buff;
  // 接收超级电容数据
  tx_super_cap_data_.chassis_power_buffer = referee_data_buffer.data().power_heat_data.buffer_energy;
  tx_super_cap_data_.chassis_power_limit = referee_data_buffer.data().robot_status.chassis_power_limit;
}

void GimbalCommunicator::SendSuperCapData() {
  super_cap->UpdateChassisBuffer(tx_super_cap_data_.chassis_power_buffer);
  if (chassis->chassis_mode() != 5) {
    if (tx_super_cap_data_.chassis_power_buffer >= 50) {
      super_cap->UpdateSettings(tx_super_cap_data_.chassis_power_limit, 300, tx_super_cap_data_.chassis_power_limit, 1,
                                1);
    } else if (tx_super_cap_data_.chassis_power_buffer >= 30) {
      super_cap->UpdateSettings(tx_super_cap_data_.chassis_power_limit, 300, tx_super_cap_data_.chassis_power_limit / 2,
                                1, 1);
    } else {
      super_cap->UpdateSettings(tx_super_cap_data_.chassis_power_limit, 300, 0, 1, 1);
    }
  } else {
    super_cap->UpdateSettings(tx_super_cap_data_.chassis_power_limit, 300, 0, 0, 0);
  }
}

void GimbalCommunicator::SendGimbalData() {
  tx_gimbal_data_.tx_buf_gimbal[0] = tx_gimbal_data_.current_heat >> 8;
  tx_gimbal_data_.tx_buf_gimbal[1] = tx_gimbal_data_.current_heat;
  tx_gimbal_data_.tx_buf_gimbal[2] = tx_gimbal_data_.current_cooling_heat >> 8;
  tx_gimbal_data_.tx_buf_gimbal[3] = tx_gimbal_data_.current_cooling_heat;
  tx_gimbal_data_.tx_buf_gimbal[4] = tx_gimbal_data_.heat_limit >> 8;
  tx_gimbal_data_.tx_buf_gimbal[5] = tx_gimbal_data_.heat_limit;
  tx_gimbal_data_.tx_buf_gimbal[6] = tx_gimbal_data_.cooling_mag;
  tx_gimbal_data_.tx_buf_gimbal[7] = tx_gimbal_data_.power_state << 4 | ((tx_gimbal_data_.robot_id < 100) ? 0 : 1);
  this->can_->Write(0x100, tx_gimbal_data_.tx_buf_gimbal, 8);
}

void GimbalCommunicator::RxCallback(const hal::CanMsg *msg) {
  if (msg->rx_std_id == 0x120) {
    gimbal_data_feedback_.remote_speed_x = static_cast<i8>(msg->data[0]);
    gimbal_data_feedback_.remote_speed_y = static_cast<i8>(msg->data[1]);
    gimbal_data_feedback_.chassis_mode = static_cast<u8>(msg->data[2]);
    gimbal_data_feedback_.UI_show_flag = static_cast<u8>(msg->data[3]);
    gimbal_data_feedback_.get_target_flag = static_cast<u8>(msg->data[4]);
    gimbal_data_feedback_.suggest_fire_flag = static_cast<u8>(msg->data[5]);
    gimbal_data_feedback_.aim_speed_change = static_cast<i8>(msg->data[6]);
  }
}

extern "C" {
void CommunicateTask(const void *argument) {
  // 创建超级电容容对象
  super_cap = new SuperCap(can1);
  while (1) {
    // 接收CAN数据
    gimbal_communicator->comunicate_data_update();
    // 发送CAN数据
    // 发送云台数据
    gimbal_communicator->SendGimbalData();
    // 发送超级电容数据
    gimbal_communicator->SendSuperCapData();
    osDelay(10);
  }
}
}