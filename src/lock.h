#pragma once

#include <atomic>
#include <mutex>

namespace franka {

struct Lock {
  std::mutex mutex{};
  std::atomic<uint64_t> command_id{0};
};

}  // namespace franka
