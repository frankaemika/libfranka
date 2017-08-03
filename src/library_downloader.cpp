#include "library_downloader.h"

#include <exception>
#include <fstream>
#include <vector>

#include <research_interface/robot/service_types.h>

#include <franka/exception.h>

namespace franka {

template <>
typename research_interface::robot::LoadModelLibrary::Response Network::tcpBlockingReceiveResponse<
    research_interface::robot::LoadModelLibrary>(uint32_t command_id) {
  using research_interface::robot::LoadModelLibrary;

  std::array<uint8_t, sizeof(typename LoadModelLibrary::Response)> buffer;

  // Wait until we receive a packet with the right header.
  std::unique_lock<std::mutex> lock(tcp_mutex_, std::defer_lock);
  typename LoadModelLibrary::Header header;
  while (true) {
    lock.lock();
    tcp_socket_.poll(Poco::Timespan(0, 1e4), Poco::Net::Socket::SELECT_READ);
    if (tcpPeekHeaderUnsafe(&header) && header.command == LoadModelLibrary::kCommand &&
        header.command_id == command_id) {
      break;
    }
    lock.unlock();
  }
  tcpReceiveIntoBufferUnsafe(buffer.data(), buffer.size());
  return *reinterpret_cast<const typename LoadModelLibrary::Response*>(buffer.data());
}

LibraryDownloader::LibraryDownloader(Network& network) {
  using research_interface::robot::LoadModelLibrary;

  LoadModelLibrary::Response response = network.executeCommand<LoadModelLibrary>(
      LoadModelLibrary::Architecture::kX64, LoadModelLibrary::System::kLinux);
  if (response.status != LoadModelLibrary::Status::kSuccess) {
    throw ModelException("libfranka: Server reports error when loading model library.");
  }

  try {
    std::vector<uint8_t> buffer(response.size);
    network.tcpReceiveIntoBuffer(buffer.data(), static_cast<int>(buffer.size()));
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
