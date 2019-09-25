#include <string>

#include "ros/ros.h"

#include "engine/alice/alice.hpp"

namespace isaac {

  class RosPinger : public alice::Codelet {
    public:
      RosPinger();
      virtual ~RosPinger();

      void start() override;
      void stop() override;
      void tick() override;

    private:
      struct RosData;
      std::unique_ptr<RosData> ros_data_;
  };

} // namespace isaac


ISAAC_ALICE_REGISTER_CODELET(isaac::RosPinger);
