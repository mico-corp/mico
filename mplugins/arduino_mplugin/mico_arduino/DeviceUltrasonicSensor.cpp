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

#include "DeviceUltrasonicSensor.h"
#include "sstream"

DeviceUltrasonicSensor *DeviceUltrasonicSensor::create(const std::string &_id, std::string _config){
    int idx = _config.find(',');
    if(idx == _config.npos) return nullptr;
    
    int pinIn =  atoi(_config.substr(0,idx).c_str());
    int pinOut =  atoi(_config.substr(idx+1).c_str());
    
    if(!isFree(pinIn) && !isFree(pinOut)){
      registerPin(pinIn);
      registerPin(pinOut);
        return new DeviceUltrasonicSensor(_id, pinIn, pinOut);
    }else{
        return nullptr;
    }
}

void DeviceUltrasonicSensor::deinitialize(){
   unregisterPin(pinIn_);
   unregisterPin(pinOut_);
}

void DeviceUltrasonicSensor::execute(){
  long t; //timepo que demora en llegar el eco
  long d; //distancia en centimetros
  
  digitalWrite(pinOut_, HIGH);
  delayMicroseconds(10);          //Enviamos un pulso de 10us
  digitalWrite(pinOut_, LOW);
  
  t = pulseIn(pinIn_, HIGH); //obtenemos el ancho del pulso
  d = t/59;             //escalamos el tiempo a una distancia en cm
  
  std::stringstream ss;
  ss << "2@" << id_<<"@"<<d;
  Serial.println(ss.str().c_str());
}

DeviceUltrasonicSensor::DeviceUltrasonicSensor(const std::string &_id, int _pinIn, int _pinOut){
    id_ = _id;
    pinIn_ = _pinIn;
    pinOut_ = _pinOut;
    pinMode(pinOut_, OUTPUT); //pin como salida
    pinMode(pinIn_, INPUT);  //pin como entrada
    digitalWrite(pinOut_, LOW);//Inicializamos el pin con 0
}
