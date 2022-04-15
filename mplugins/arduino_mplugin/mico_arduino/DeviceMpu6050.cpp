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

#include "DeviceMpu6050.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "sstream"

DeviceMpu6050 *DeviceMpu6050::create(const std::string &_id){ 
    if(!isFree(A4) && !isFree(A5)){
      registerPin(A4);
      registerPin(A5);
      return new DeviceMpu6050(_id);
    }else{
        return nullptr;
    }
}

void DeviceMpu6050::execute(){
  sensors_event_t a, g, temp;
  mpu_.getEvent(&a, &g, &temp);
  std::stringstream ss;
  ss << "2@"<<id_<<"@"<< a.acceleration.x<< "," << a.acceleration.y<< "," <<a.acceleration.z<<"," << g.gyro.x <<"," <<g.gyro.y << "," << g.gyro.z;
  Serial.println(ss.str().c_str());
}

void DeviceMpu6050::deinitialize(){
    unregisterPin(A4);
    unregisterPin(A5);
}

DeviceMpu6050::DeviceMpu6050(const std::string &_id){
  Serial.println("Beginning");
  id_ = _id;
  mpu_.begin();
  mpu_.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu_.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu_.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("started");
}
