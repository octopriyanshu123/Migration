/**
 * -------------------------------------------- COPYRIGHT NOTICE ---------------------------------------------------\n
 * Copyright (C) 2023 Octobotics Tech Pvt. Ltd. All Rights Reserved.\n
 * Do not remove this copyright notice.\n
 * Do not use, reuse, copy, merge, publish, sub-license, sell, distribute or modify this code - except without\n
 * explicit, written permission from Octobotics Tech Pvt. Ltd.\n
 * Contact connect@octobotics.tech for full license information.\n
 * -------------------------------------------- COPYRIGHT NOTICE ---------------------------------------------------
 *
 *
 * @file serial_comm.h
 * @author vaibhav
 * @brief
 * @date 2023-04-07
 *
 *
 */

#pragma once
#ifndef _COMMON_SERIAL_COMM_H_
#define _COMMON_SERIAL_COMM_H_

#include <string>
#include <vector>
#include <stdint.h>

#include "CppLinuxSerial/SerialPort.hpp"
using namespace mn::CppLinuxSerial;


class SerialComm
{
protected:
private:
    std::string port_;
    unsigned int baudrate_;
    unsigned int timeout_;
    std::string receive_data_;
public:
    SerialComm();
    ~SerialComm();

    void get_port(std::string &port);
    void serial_init();
    void serial_shutdown();

    //string
    void send_serial_s(const std::string &send);
    void receive_serial_s(std::string &receive);
    void send_receive_serial_s(const std::string &send, std::string &receive);

    //binary
    void send_serial_b(std::vector<uint8_t> &send);
    void receive_serial_b(std::vector<uint8_t> &receive);
    void send_receive_serial_b(std::vector<uint8_t> &send, std::vector<uint8_t> &receive);
    SerialPort serialPort_;

};

#endif //_COMMON_SERIAL_COMM_H_
