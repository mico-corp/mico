/* 
 * File:   SimpleSerial.h
 * Author: Terraneo Federico
 * Distributed under the Boost Software License, Version 1.0.
 *
 * Created on September 10, 2009, 12:12 PM
 * Modified on June 10, 2020, 11:19 PM by Pablo Ramon Soria
 */

#include <mico/arduino/SerialPort.h>

SerialPort::SerialPort(std::string _port, unsigned int _baudRate) 
    #if defined(__linux__)
        : io_(), serial_(io_) 
    #endif
    {

    #if defined(_WIN32)
        this->connected = false;
        this->handler = CreateFileA(static_cast<LPCSTR>(_port.c_str()),
                                    GENERIC_READ | GENERIC_WRITE,
                                    0,
                                    NULL,
                                    OPEN_EXISTING,
                                    FILE_ATTRIBUTE_NORMAL,
                                    NULL);
        if (this->handler == INVALID_HANDLE_VALUE) {
            if (GetLastError() == ERROR_FILE_NOT_FOUND) {
                std::cerr << "ERROR: Handle was not attached.Reason : " << _port << " not available\n";
            } else {
                std::cerr << "ERROR!!!\n";
            }
        }
        else {
            DCB dcbSerialParameters = {0};

            if (!GetCommState(this->handler, &dcbSerialParameters)) {
                std::cerr << "Failed to get current serial parameters\n";
            } else {
                dcbSerialParameters.BaudRate = _baudRate;
                dcbSerialParameters.ByteSize = 8;
                dcbSerialParameters.StopBits = ONESTOPBIT;
                dcbSerialParameters.Parity = NOPARITY;
                dcbSerialParameters.fDtrControl = DTR_CONTROL_ENABLE;

                if (!SetCommState(handler, &dcbSerialParameters)) {
                    std::cout << "ALERT: could not set serial port parameters\n";
                } else {
                    this->connected = true;
                    PurgeComm(this->handler, PURGE_RXCLEAR | PURGE_TXCLEAR);
                    Sleep(1);
                }
            }
        }
    #elif defined(__linux__)
        serial_.open(_port);
        serial_.set_option(boost::asio::serial_port_base::baud_rate(_baudRate));
    #endif
}

bool SerialPort::isOpen(){

    #if defined(_WIN32)
        if (!ClearCommError(this->handler, &this->errors, &this->status)) {
            this->connected = false;
        }
        return this->connected;
    #elif defined(__linux__)
        return serial_.is_open();
    #endif
}

void SerialPort::close() {
    #if defined(_WIN32)
        if (this->connected) {
            this->connected = false;
            CloseHandle(this->handler);
        }
    #elif defined(__linux__)
        serial_.close();
    #endif
}

size_t SerialPort::writeString(std::string _s) {
    
    #if defined(_WIN32)
        DWORD bytesSend;
        if (!WriteFile(this->handler, (void*) _s.c_str(), _s.size(), &bytesSend, 0)) {
            ClearCommError(this->handler, &this->errors, &this->status);
            return false;
        }
        
        return true;
    #elif defined(__linux__)
        return boost::asio::write(serial_,boost::asio::buffer(_s.c_str(),_s.size()));
    #endif
}

std::string SerialPort::readLine() {
    
       
        char c;
        std::string result;
        int value = 0;

        for(;;) {
            std::string c = readBytes(1);
            if(c.size() == 0)
                return "";
                
            switch(c[0]) {
                case '\r':
                    break;
                case '\n':
                    return result;
                default:
                    result+=c;
            }
        }
}

std::string SerialPort::readBytes(unsigned _bytes) {
    
    #if defined(_WIN32)
        DWORD bytesRead{};
        ClearCommError(this->handler, &this->errors, &this->status);
        char buffer[1024];
        if (ReadFile(this->handler, (void*) buffer, _bytes, &bytesRead, NULL)) {
            return std::string(buffer, buffer + bytesRead);
        }else{
            return "";
        }

    #elif defined(__linux__)
        //Reading data char by char, code is optimized for simplicity, not speed
        using namespace boost;
        char c;
        std::string result;
        for(unsigned i = 0 ; i < _bytes ; i++) {
            asio::read(serial_,asio::buffer(&c,1));
            result+=c;
        }
        return result;
    #endif
}
