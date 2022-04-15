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
#include "DeviceMpu6050.h"
#include "DeviceUltrasonicSensor.h"

std::map<std::string, DeviceBackend*> DeviceManager::devices_;


bool DeviceManager::registerDevice(const std::string &_id, std::string _data){
    int idx = _data.find('@');
    if(idx == _data.npos) return false;
    std::string backend = _data.substr(0,idx);
    _data = _data.substr(idx+1);
    
    if(devices_.find(_id) == devices_.end()){
      auto device = createDevice(_id, backend, _data);
      if(device){
        devices_[_id] = device;
  Serial.println("Device created");
      }
    }
}

void DeviceManager::unregisterDevice(const std::string &_id){
    devices_.erase(_id);
}

void DeviceManager::runDevices(){
  //Serial.println("start run");
    for(auto &device : devices_){
      device.second->execute();
    }
}
void DeviceManager::processMessage(const std::string &_id, const std::string &_data){
  auto it = devices_.find(_id);
  if(it != devices_.end())
    it->second->process(_data);
}

DeviceBackend* DeviceManager::device(const std::string &_devName){
  if(devices_.find(_devName) != devices_.end()){
    return devices_[_devName];
  }else{
    return nullptr;
  }
  
}
    
DeviceBackend* DeviceManager::createDevice(const std::string &_id, const std::string &_backend, const std::string &_config){
  if(_backend == DeviceDigitalPin::backendName()){
    return DeviceDigitalPin::createPin(_id, _config);
  }else if(_backend == DeviceMpu6050::backendName()){
    return DeviceMpu6050::create(_id);
  }else if(_backend == DeviceUltrasonicSensor::backendName()){
    return DeviceUltrasonicSensor::create(_id, _config);
  }
}

void DeviceManager::listDevices(){
    for(auto &device : devices_){
//       Serial.println(device.first.c_str());
    }
}
