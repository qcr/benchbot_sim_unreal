#include "ping_codelet.hpp"

namespace isaac {

  void Pinger::start() {
    tickPeriodically();
  }

  void Pinger::tick() {
    LOG_INFO(get_message().c_str());
  }

} // namespace isaac
