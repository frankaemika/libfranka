// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/exception.h>

#include <string>
#include <utility>

// `using std::string_literals::operator""s` produces a GCC warning that cannot be disabled, so we
// have to use `using namespace ...`.
// See https://gcc.gnu.org/bugzilla/show_bug.cgi?id=65923#c0
using namespace std::string_literals;  // NOLINT(google-build-using-namespace)

namespace franka {

ControlException::ControlException(const std::string& what,
                                   std::vector<franka::Record> log) noexcept
    : Exception(what), log(std::move(log)) {}

IncompatibleVersionException::IncompatibleVersionException(uint16_t server_version,
                                                           uint16_t library_version) noexcept
    : Exception("libfranka: Incompatible library version (server version: "s +
                std::to_string(server_version) + ", library version: "s +
                std::to_string(library_version) +
                "). Please check https://frankaemika.github.io for Panda system updates "
                "or use a different version of libfranka."s),
      server_version(server_version),
      library_version(library_version) {}

}  // namespace franka
