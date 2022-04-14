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

#include <mico/arduino/flow/ArduinoConnection.h>
#include <mico/arduino/SerialPort.h>

namespace mico {
	namespace arduino {
		ArduinoConnection::ArduinoConnection(const std::string& _port, int baudrate) {
			port_ = std::shared_ptr<SerialPort>(new SerialPort(_port, 115200));
			run_ = true;
			readThread_ = std::thread([&]() {
				while (run_) {
					/*auto str = port_->readLine();
					
					nlohmann::json msg(str);
					if (msg.is_object()) {
						std::string id = msg["id"];
						if (callbacks_.find(id) == callbacks_.end()) {
							callbacks_[id](msg);
						}
					}*/

				}
			});
		}
		
		ArduinoConnection::~ArduinoConnection() {
			run_ = false;
			if (readThread_.joinable())
				readThread_.join();
		}

		std::string ArduinoConnection::createUniqueKey() {
			return std::to_string(uniqueId_++);
		}

		void ArduinoConnection::registerDevice(nlohmann::json _config, ReadCallback _readCb ) {
			std::lock_guard<std::mutex> lock(mutex_);
			port_->writeString(_config.dump() + "\n");	// 666 It might be good to have a confirmation here
			std::string id = _config["id"];
			if (_readCb) {
				if (callbacks_.find(id) == callbacks_.end()) {
					callbacks_[id] = _readCb;
				}
				else {
					throw std::invalid_argument("Id of device already registered");
				}
			}
		}

		void ArduinoConnection::unregisterDevice(nlohmann::json _config) {
			std::lock_guard<std::mutex> lock(mutex_);
			port_->writeString(_config.dump() + "\n");	// 666 It might be good to have a confirmation here
		}

		void ArduinoConnection::write(nlohmann::json _config) {
			std::lock_guard<std::mutex> lock(mutex_);
			port_->writeString(_config.dump() + "\n");	// 666 It might be good to have a confirmation here
		}


		void ArduinoConnection::close() {
			std::lock_guard<std::mutex> lock(mutex_);
			port_->close();
		}
	}
}