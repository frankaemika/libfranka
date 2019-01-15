// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "library_downloader.h"

#include <exception>
#include <fstream>
#include <vector>
#include <iostream>

#include <franka/exception.h>
#include <franka/platform_type.h>
#include <research_interface/robot/service_types.h>

namespace franka {

LibraryDownloader::LibraryDownloader(Network& network) {
  using research_interface::robot::LoadModelLibrary;
  LoadModelLibrary::Architecture architecture;
  LoadModelLibrary::System operation_system;

  #if defined(X64)
      architecture = LoadModelLibrary::Architecture::kX64;
      std::cout << "use x64" << std::endl;
  #elif defined(X86)
      architecture = LoadModelLibrary::Architecture::kx86;
      std::cout << "use x86" << std::endl;
  #elif defined(ARM)
      architecture = LoadModelLibrary::Architecture::kARM;
      std::cout << "use ARM" << std::endl;
  #elif defined(ARM64)
      architecture = LoadModelLibrary::Architecture::kARM64;
      std::cout << "use ARM64" << std::endl;
  #else
      throw ModelException("libfranka: Unsupported architecture!");
  #endif

  #if defined(WINDOWS)
    operation_system = LoadModelLibrary::System::kWindows;
    std::cout << "use Windows" << std::endl;
  #elif defined(LINUX)
    std::cout << "use Linux" << std::endl;
  #else
    throw ModelException("libfranka: Unsupported operation system!");
  #endif

  uint32_t command_id = network.tcpSendRequest<LoadModelLibrary>(
    architecture, operation_system);
  std::vector<uint8_t> buffer;
  LoadModelLibrary::Response response =
      network.tcpBlockingReceiveResponse<LoadModelLibrary>(command_id, &buffer);
  if (response.status != LoadModelLibrary::Status::kSuccess) {
    throw ModelException("libfranka: Server reports error when loading model library.");
  }

  try {
    std::ofstream model_library_stream(path().c_str(), std::ios_base::out | std::ios_base::binary);
    model_library_stream.write(reinterpret_cast<char*>(buffer.data()), buffer.size());
  } catch (const std::exception& ex) {
    throw ModelException("libfranka: Cannot save model library.");
  }
}

const std::string& LibraryDownloader::path() const noexcept {
  return model_library_file_.path();
}

};  // namespace franka
