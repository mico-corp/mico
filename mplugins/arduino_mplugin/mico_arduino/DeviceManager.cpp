//---------------------------------------------------------------------------------------------------------------------
//  Arduino MICO plugin
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2020 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------


#include "DeviceManager.h"
#include "DeviceDigitalPin.h"

std::map<std::string, DeviceBackend*> DeviceManager::devices_;


void DeviceManager::registerDevice(JsonObject &_json){
    std::string key = _json["id"];
    if(devices_.find(key) == devices_.end()){
      std::string backend = _json["bknd"];
  
      auto device = createDevice(backend, _json);
      if(device){
        devices_[key] = device;
      }
    }
}

void DeviceManager::unregisterDevice(JsonObject &_json){
    std::string key = _json["id"];
    devices_.erase(key);
}

void DeviceManager::runDevices(){
    for(auto &device : devices_){
       device.second->execute();
    }
}
void DeviceManager::processMessage(JsonObject &_json){
  std::string key = _json["id"];
  auto it = devices_.find(key);
  if(it != devices_.end())
    it->second->process(_json);
}

DeviceBackend* DeviceManager::createDevice(std::string _backend, JsonObject &_config){
  if(_backend == DeviceDigitalOutPin::backendName()){
    return DeviceDigitalOutPin::createPin(_config);
  }else if(_backend == DeviceDigitalInPin::backendName()){
    return DeviceDigitalInPin::createPin(_config);
  }
}

void DeviceManager::listDevices(){
    for(auto &device : devices_){
       Serial.println(device.first.c_str());
    }
}
