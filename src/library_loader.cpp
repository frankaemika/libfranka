// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "library_loader.h"

#include <Poco/Exception.h>

#include <franka/exception.h>

using namespace std::string_literals;  // NOLINT(google-build-using-namespace)

namespace franka {

LibraryLoader::LibraryLoader(const std::string& filepath) try {
  library_.load(filepath);
} catch (const Poco::LibraryAlreadyLoadedException& e) {
  throw ModelException("libfranka: Model library already loaded"s);
} catch (const Poco::LibraryLoadException& e) {
  throw ModelException("libfranka: Cannot load model library: "s + e.what());
} catch (const Poco::Exception& e) {
  throw ModelException("libfranka: Error while loading library: "s + e.what());
}

LibraryLoader::~LibraryLoader() {
  try {
    library_.unload();
  } catch (...) {
  }
}

void* LibraryLoader::getSymbol(const std::string& symbol_name) try {
  return library_.getSymbol(symbol_name);
} catch (const Poco::NotFoundException& e) {
  throw ModelException("libfranka: Symbol cannot be found: "s + e.what());
} catch (const Poco::Exception& e) {
  throw ModelException("libfranka: Error while fetching symbols: "s + e.what());
}

}  // namespace franka
