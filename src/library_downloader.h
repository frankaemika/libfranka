// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <string>

#include <Poco/TemporaryFile.h>

#include "network.h"

namespace franka {

class LibraryDownloader {
 public:
  LibraryDownloader(Network& network);
  ~LibraryDownloader();

  const std::string& path() const noexcept;

 private:
  Poco::File model_library_file_;
};

};  // namespace franka
