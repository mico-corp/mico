/*
 * File:   SimpleSerial.h
 * Author: Terraneo Federico
 * Distributed under the Boost Software License, Version 1.0.
 *
 * Created on September 10, 2009, 12:12 PM
 * Modified on June 10, 2020, 11:19 PM by Pablo Ramon Soria
 */


#include <mico/arduino/SerialPort.h>
#include <boost/asio/basic_serial_port.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/impl/io_context.hpp>
#include <boost/asio/impl/io_context.ipp>
#include <boost/asio/impl/read.hpp>
#include <boost/asio/impl/serial_port_base.hpp>
#include <boost/asio/impl/serial_port_base.ipp>
#include <boost/asio/impl/write.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/serial_port_base.hpp>
#include <boost/config/detail/suffix.hpp>
#include <chrono>
#include <thread>

SerialPort::SerialPort(std::string _port, unsigned int _baudRate) : io_(), serial_(io_, _port) {
    serial_.set_option(boost::asio::serial_port_base::baud_rate(_baudRate));
}

bool SerialPort::isOpen() const {
    return serial_.is_open();
}

void SerialPort::close() {
    serial_.close();
}

size_t SerialPort::writeString(std::string _s) {
    return boost::asio::write(serial_, boost::asio::buffer(_s.c_str(), _s.size()));
}

std::string SerialPort::readLine() {
    //Reading data char by char, code is optimized for simplicity, not speed
    using namespace boost;
    char c;
    std::string result;

    for (;;) {
        asio::read(serial_, asio::buffer(&c, 1));
        switch (c)
        {
        case '\r':
            break;
        case '\n':
            return result;
        default:
            result += c;
        }
    }
}

std::string SerialPort::readBytes(unsigned _bytes) {
    //Reading data char by char, code is optimized for simplicity, not speed
    using namespace boost;
    char c;
    std::string result;
    for (unsigned i = 0; i < _bytes; i++) {
        asio::read(serial_, asio::buffer(&c, 1));
        result += c;
    }
    return result;
}