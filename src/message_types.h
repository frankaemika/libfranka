#pragma once

#include <cstdint>

namespace message_types {

constexpr uint16_t kRiLibraryVersion = 1;

enum class FunctionId : uint32_t {
  kConnect = 0,
  kPing = 1,
};

struct ConnectRequest {
  FunctionId function_id;
  uint16_t ri_library_version;
  uint16_t udp_port;
};

struct ConnectReply {
  enum class StatusCode : uint32_t {
    kSuccess = 0,
    kIncompatibleLibraryVersion = 1
  };
  StatusCode status_code;
  uint16_t ri_version;
};

struct PingRequest {
  FunctionId function_id;
};

struct PingReply {
  enum class StatusCode : uint32_t { kSuccess = 0, kFail = 1 };
  StatusCode status_code;
};

}  // namespace message_types
