#include "model_library_downloader.h"

#include <fstream>
#include <vector>

#include <research_interface/robot/service_types.h>

namespace franka {

ModelLibraryDownloader::ModelLibraryDownloader(Network& network) {
  research_interface::robot::LoadModelLibrary::Request request(
      research_interface::robot::LoadModelLibrary::Architecture::kX64,
      research_interface::robot::LoadModelLibrary::Platform::kLinux);
  network.tcpSendRequest<research_interface::robot::LoadModelLibrary::Request>(request);
  auto response = network.tcpBlockingReceiveResponse<research_interface::robot::LoadModelLibrary>();
  std::vector<char> buffer(response.size);
  // TODO(FWA): assert(buffer.size() <= std::numeric_limits<int>::max());
  network.tcpReceiveIntoBuffer(buffer.data(), buffer.size());
  std::ofstream model_library_stream(path().c_str(), std::ios_base::out | std::ios_base::binary);
  model_library_stream.write(buffer.data(), buffer.size());
}

const std::string& ModelLibraryDownloader::path() const noexcept {
  return model_library_file_.path();
}

};  // namespace franka
