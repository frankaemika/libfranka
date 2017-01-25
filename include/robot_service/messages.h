#pragma once

#include <cstdint>

namespace robot_service {

enum class RIFunctionId : uint32_t {
  kConnect = 0
};

enum class StatusCode : uint32_t {
  kSuccess = 0,
  kIncompatibleLibraryVersion = 1,
  kUnknownFunction = 2
};

struct RIConnectRequest {
  RIFunctionId function_id;
  uint16_t ri_library_version;
  uint16_t udp_port;
};

struct RIConnectReply {
  StatusCode status_code;
  uint16_t ri_version;
};

struct RIErrorReply {
  StatusCode status_code;
};

}