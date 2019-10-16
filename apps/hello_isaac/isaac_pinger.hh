#include <string>

#include "engine/alice/alice.hpp"

namespace isaac {

  class IsaacPinger : public alice::Codelet {
    public:
      void start() override;
      void tick() override;

      // ISAAC_PARAM(::std::string, message, "Hello Isaac!");
  };

} // namespace isaac


ISAAC_ALICE_REGISTER_CODELET(isaac::IsaacPinger);
