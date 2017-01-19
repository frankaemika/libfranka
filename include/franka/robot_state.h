#pragma once

#include <cstdint>

namespace franka{

struct RobotState {
    std::uint32_t timestamp;
    double q[7];
    double dq[7];
    double tau_J[7];
    double dtau_J[7];
};

}