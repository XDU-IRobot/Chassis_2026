#ifndef __RC_REFEREE_DATA_HPP__
#define __RC_REFEREE_DATA_HPP__

#include <queue>

#include "librm.hpp"

using namespace rm;
using namespace rm::hal;
using namespace rm::device;

class RcTcRefereeData {
 public:
  RcTcRefereeData() = delete;
  explicit RcTcRefereeData(SerialInterface &serial);

  void Begin();
  void RxCallback(const std::vector<u8> &data, u16 rx_len);

 private:
  SerialInterface *serial_;
};

#endif /* __RC_REFEREE_DATA_HPP__ */