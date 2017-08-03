#pragma once

#include <string>

#include <Poco/TemporaryFile.h>

#include "network.h"

namespace franka {

class LibraryDownloader {
 public:
  LibraryDownloader(Network& network);

  const std::string& path() const noexcept;

 private:
  Poco::TemporaryFile model_library_file_{};
};

};  // namespace franka
