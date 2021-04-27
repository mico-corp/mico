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

#if defined(_WIN32)
#   include <windows.h>
#elif defined(__linux__)
#   include <boost/asio.hpp>
#endif

#include <iostream>

class SerialPort {
    public:
        /**
         * Constructor.
         * \param port device name, example "/dev/ttyUSB0" or "COM4"
         * \param baud_rate communication speed, example 9600 or 115200
         * \throws boost::system::system_error if cannot open the
         * serial device
         */
        SerialPort(std::string _port, unsigned int _baudRate);

        /**
         * Check if port is opened
         * \throws boost::system::system_error on failure
         */
        bool isOpen();

        /**
         * Close connection
         * \throws boost::system::system_error on failure
         */
        void close();

        /**
         * Write a string to the serial device.
         * \param s string to write
         * \throws boost::system::system_error on failure
         */
        size_t writeString(std::string s);

        /**
         * Blocks until a line is received from the serial device.
         * Eventual '\n' or '\r\n' characters at the end of the string are removed.
         * \return a string containing the received line
         * \throws boost::system::system_error on failure
         */
        std::string readLine();

        /**
         * Blocks until a N bytes are received from the serial device.
         * \return a string containing the bytes line
         * \throws boost::system::system_error on failure
         */
        std::string readBytes(unsigned _bytes);

    private:
        #if defined(_WIN32)
            HANDLE handler;
            bool connected;
            COMSTAT status;
            DWORD errors;
        #elif defined(__linux__)
            boost::asio::io_service io_;
            boost::asio::serial_port serial_;
        #endif
};

#endif	/* _SIMPLESERIAL_H */
