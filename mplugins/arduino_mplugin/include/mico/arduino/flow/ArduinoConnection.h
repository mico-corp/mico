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

#ifndef MICO_ARDUINO_FLOW_ARDUINOCONNECTION_H_
#define MICO_ARDUINO_FLOW_ARDUINOCONNECTION_H_

#include <mutex>
#include <unordered_map>
#include <thread>
#include <boost/any.hpp>
#include <functional>

class SerialPort;

// MESSAGES PROTOCOL
//
// Register device
//		{	"type": 0, 
//			"id": 1234,
//			"bknd": "type_device",
//			"config": { "":""...}
//		}
// 
// Unregister device
//		{	"type": 1, 
//			"id": 1234
//		}
//
// Data to device
//		{	"type": 2, 
//			"id": 1234,
//			"data": ...
//		}

namespace mico {
	namespace arduino {
		class ArduinoDevice;

		class ArduinoConnection {
		public:
			typedef std::function<void(boost::any &)> ReadCallback;

			ArduinoConnection(const std::string& _port, int baudrate);
			~ArduinoConnection();

			bool registerCallback(int _id, ReadCallback _readCb);
			
			void unregisterCallback(int _id);

			template<typename T_>
			void write(int _id, const T_ &_dat);

			void close();

		private:
			std::unordered_map<int, ArduinoDevice*> devices_;
			std::shared_ptr<SerialPort> port_;

			inline static int uniqueId_ = 0;
			
			std::unordered_map<int, ReadCallback> callbacks_;
			std::thread readThread_;
			bool run_ = true;

			std::mutex mutex_;
		};
	}
}

#include <mico/arduino/flow/ArduinoConnection.inl>

#endif