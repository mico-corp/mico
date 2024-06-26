//-----------------------------------------------------------------------------
//  Arduino MICO plugin
//-----------------------------------------------------------------------------
//  Copyright 2020 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//-----------------------------------------------------------------------------
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
//-----------------------------------------------------------------------------

#include <mico/arduino/SerialPort.h>

namespace mico {
	namespace arduino {
		template<>
		inline void ArduinoConnection::write<int>(int _id, const int& _data) {
			std::string msg = std::to_string(_id) + "@" + std::to_string(_data);
			port_->writeString(msg + "\n");
		}

		template<>
		inline void ArduinoConnection::write<float>(int _id, const float& _data) {
			std::string msg = std::to_string(_id) + "@" + std::to_string(_data);
			port_->writeString(msg + "\n");
		}

		template<>
		inline void ArduinoConnection::write<std::vector<int>>(int _id, const std::vector<int>& _data) {
			std::string msg = std::to_string(_id) + "@";
			for (auto& e : _data) {
				msg += std::to_string(e) + ",";
			}
			port_->writeString(msg + "\n");
		}

		template<>
		inline void ArduinoConnection::write<std::vector<float>>(int _id, const std::vector<float>& _data) {
			std::string msg = std::to_string(_id) + "@";
			for (auto& e : _data) {
				msg += std::to_string(e) + ",";
			}
			port_->writeString(msg + "\n");
		}
	}
}