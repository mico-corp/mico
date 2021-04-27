/* 
 * File:   SimpleSerial.h
 * Author: Terraneo Federico
 * Distributed under the Boost Software License, Version 1.0.
 *
 * Created on September 10, 2009, 12:12 PM
 * Modified on June 10, 2020, 11:19 PM by Pablo Ramon Soria
 */

#ifndef _SIMPLESERIAL_H
#define	_SIMPLESERIAL_H

#include <boost/asio.hpp>

class SerialPort
{
public:
    /**
     * Constructor.
     * \param port device name, example "/dev/ttyUSB0" or "COM4"
     * \param baud_rate communication speed, example 9600 or 115200
     * \throws boost::system::system_error if cannot open the
     * serial device
     */
    SerialPort(std::string port, unsigned int baud_rate) : io(), serial(io,port) {
        serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    }


    /**
     * Check if port is opened
     * \throws boost::system::system_error on failure
     */
    bool isOpen() {
        return serial.is_open();
    }

    /**
     * Close connection
     * \throws boost::system::system_error on failure
     */
    void close() {
        serial.close();
    }

    /**
     * Write a string to the serial device.
     * \param s string to write
     * \throws boost::system::system_error on failure
     */
    void writeString(std::string s) {
        boost::asio::write(serial,boost::asio::buffer(s.c_str(),s.size()));
    }

    /**
     * Blocks until a line is received from the serial device.
     * Eventual '\n' or '\r\n' characters at the end of the string are removed.
     * \return a string containing the received line
     * \throws boost::system::system_error on failure
     */
    std::string readLine() {
        //Reading data char by char, code is optimized for simplicity, not speed
        using namespace boost;
        char c;
        std::string result;
        for(;;)
        {
            asio::read(serial,asio::buffer(&c,1));
            switch(c)
            {
                case '\r':
                    break;
                case '\n':
                    return result;
                default:
                    result+=c;
            }
        }
    }

    /**
     * Blocks until a N bytes are received from the serial device.
     * \return a string containing the bytes line
     * \throws boost::system::system_error on failure
     */
    std::string readBytes(unsigned _bytes) {
        //Reading data char by char, code is optimized for simplicity, not speed
        using namespace boost;
        char c;
        std::string result;
        for(unsigned i = 0 ; i < _bytes ; i++) {
            asio::read(serial,asio::buffer(&c,1));
            result+=c;
        }
        return result;
    }

    /**
     * Blocks until a N bytes are received from the serial device.
     * \return a string containing the bytes line
     * \throws boost::system::system_error on failure
     */
    uint8_t readByte() {
        //Reading data char by char, code is optimized for simplicity, not speed
        using namespace boost;
        char c;
        asio::read(serial,asio::buffer(&c,1));
            
        return c;
    }

private:
    boost::asio::io_service io;
    boost::asio::serial_port serial;
};

#endif	/* _SIMPLESERIAL_H */
