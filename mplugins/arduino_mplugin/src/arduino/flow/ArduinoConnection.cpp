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
#include <QMessageBox>

namespace mico {
	namespace arduino {
		ArduinoConnection::ArduinoConnection(const std::string& _port, int baudrate) {
			port_ = std::shared_ptr<SerialPort>(new SerialPort(_port, 115200));
			run_ = true;
			readThread_ = std::thread([&]() {
				while (run_) {
					auto str = port_->readLine();
					
					int idx = str.find("@");
					if (idx != str.npos) {
						boost::any data;
						try {
							int id = std::stoi(str.substr(0, idx));
							if (callbacks_.find(id) != callbacks_.end()) {
								auto dataRaw = str.substr(idx + 1);
								if (dataRaw.find(",") == dataRaw.npos) { // Scalar
									if (dataRaw.find(".") == dataRaw.npos) { // int
										data = std::stoi(dataRaw);
									} else { // Float
										data = std::stof(dataRaw);
									}
								} else { // Vector
									std::istringstream iss(dataRaw);
									std::string token;
									if (dataRaw.find(".") == dataRaw.npos) { // int
										std::vector<int> vdata;
										while (std::getline(iss, token, ',')) {
											vdata.push_back(std::stoi(token));
										}
										data = vdata;
									} else { // Float
										std::vector<float> vdata;
										while (std::getline(iss, token, ',')) {
											vdata.push_back(std::stof(token));
										}
										data = vdata;
									}
								}
								std::lock_guard<std::mutex> lock(mutex_);
								callbacks_[id](data);
							}
						} catch (std::invalid_argument& _e) {

						}
					}
				}
			});
		}
		
		ArduinoConnection::~ArduinoConnection() {
			run_ = false;
			if (readThread_.joinable())
				readThread_.join();
		}

		bool ArduinoConnection::registerCallback(int _id, ReadCallback _readCb ) {
			std::lock_guard<std::mutex> lock(mutex_);
			if (callbacks_.find(_id) == callbacks_.end()) {
				callbacks_[_id] = _readCb;
				return true;
			}
			else {
				QMessageBox::warning(nullptr, "Bad Id", "Cannot register the given ID, please select a different one.");
				return false;
			}
		}

		void ArduinoConnection::unregisterCallback(int _id) {
			std::lock_guard<std::mutex> lock(mutex_);
			if (callbacks_.find(_id) != callbacks_.end()) {
				callbacks_.erase(_id);
			}
		}


		void ArduinoConnection::close() {
			std::lock_guard<std::mutex> lock(mutex_);
			callbacks_.clear();	// 666 Notify blocks to put as unconfigured.
			port_->close();
		}
	}
}