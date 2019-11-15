#include "isaac_pinger.hh"

namespace isaac {
void IsaacPinger::start() { tickPeriodically(); }

void IsaacPinger::tick() {
  // LOG_INFO(get_message().c_str());
  LOG_INFO("Hello world!");
}
}  // namespace isaac
