#pragma once

#include <stddef.h>
#include <sys/types.h>

namespace robot_service {

enum class RIFunctionId : uint32_t {
  kConnect = 0
};

struct RIConnectRequest {
  RIFunctionId function_id;
  uint16_t ri_library_version;
  uint16_t udp_port;
};

struct RIConnectReply {
  enum class StatusCode : uint32_t {
    kSuccess = 0,
    kIncompatibleLibraryVersion = 1
  };
  uint32_t status_code;
  uint16_t ri_version;
};
}