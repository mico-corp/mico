//-----------------------------------------------------------------------------
//  FLOW
//-----------------------------------------------------------------------------
//  Copyright 2020 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//-----------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to
//  deal in the Software without restriction, including without limitation the
//  rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
//  sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//  IN THE SOFTWARE.
//-----------------------------------------------------------------------------

#include <flow/plugins/PluginsLoader.h>

#ifdef _WIN32
#include <dlfcn.h>
#include <windows.h>
#endif

#ifdef linux
#include <dlfcn.h>
#endif

#include <boost/filesystem.hpp>

namespace flow {

PluginNodeCreator::ListCreators
PluginsLoader::parseFolder(const std::string &_path) {
  PluginNodeCreator::ListCreators list;

  std::vector<std::string> files;
  readDirectory(_path, files);
  if (files.size() > 0) {
    for (auto file : files) {
      auto tempList = parseLibrary(file);
      list.insert(list.end(), tempList.begin(), tempList.end());
    }
  }
  return list;
}

PluginNodeCreator::ListCreators
PluginsLoader::parseLibrary(const std::string &_path) {
  if (_path.find("dll") == std::string::npos &&
      _path.find("so") == std::string::npos)
    return {};
  if (_path.find("mico-") == std::string::npos)
    return {};

  void *handle = getHandle(_path);
  if (handle) {
    auto creator = retrieveFactoryBlocks(_path, handle);
    if (creator)
      return creator->get();
  }
  return {};
}

struct PathLeafString {
  std::string
  operator()(const boost::filesystem::directory_entry &entry) const {
    return entry.path().string();
  }
};

void PluginsLoader::readDirectory(const std::string &_path,
                                  std::vector<std::string> &_files) {
  boost::filesystem::path p(_path);
  if (boost::filesystem::exists(p)) {
    boost::filesystem::directory_iterator start(p);
    boost::filesystem::directory_iterator end;
    std::transform(start, end, std::back_inserter(_files), PathLeafString());
  }
}

void *PluginsLoader::getHandle(const std::string &_mpluginPath) {
  void *hndl = dlopen(_mpluginPath.c_str(), RTLD_NOW);
  if (hndl) {
    dlerror();
    return hndl;
  } else {
    std::cerr << "[Warning]: " << dlerror() << std::endl;
    return nullptr;
  }
}

PluginNodeCreator *
PluginsLoader::retrieveFactoryBlocks(const std::string &_mpluginPath,
                                     void *_handle) {
  typedef PluginNodeCreator *(*Factory)(fs::path);

  void *mkr = dlsym(_handle, "factory");
  if (mkr == nullptr) {
    std::cerr << "[Warning] Pluging " << _mpluginPath
              << " does not have factory" << std::endl;
    std::cerr << "[Warning]: " << dlerror() << std::endl;
    return nullptr;
  } else {
    Factory factory = (Factory)mkr;

    const char *dlsym_error = dlerror();
    if (dlsym_error) {
      std::cerr << "[Warning] Cannot load symbol 'factory': " << dlsym_error
                << '\n';
      return nullptr;
    }

    return factory(_mpluginPath);
  }
}
} // namespace flow
