#pragma

#include <string>

#include "engine/alice/alice.hpp"

namespace isaac {

  class Pinger : public alice::Codelet {
    void start() override;
    void tick() override;

    ISAAC_PARAM(std::string, message, "Hello DEMO!");
  };

} // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::Pinger);
