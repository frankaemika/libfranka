#pragma once

#include <Poco/SharedLibrary.h>

namespace franka {

/*
 * Wraps library loading and unloading with RAII.
 */
class LibraryLoader {
 public:
  LibraryLoader(const std::string& filepath);
  ~LibraryLoader();

  void* getSymbol(const std::string& symbol_name);

 private:
  Poco::SharedLibrary library_;
};

}  // namespace franka
