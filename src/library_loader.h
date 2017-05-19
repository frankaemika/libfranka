#pragma once

#include <Poco/SharedLibrary.h>

#include <franka/exception.h>

namespace franka {

/*
 * Main purpose of this class is to allow for forward declaration in model.h.
 * Additionally it wraps it using RAII.
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