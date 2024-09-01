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
 * @file x10_api_base.h
 * @author vaibhav
 * @brief
 * @date 2023-04-07
 *
 *
 */

#pragma once
#ifndef BASE_X10_API_BASE_H
#define BASE_X10_API_BASE_H



#include "x10_reg.hpp"
#include "crc16.hpp"
#include "serial_comm.hpp"

#include <stdint.h>
#include <string>
#include <vector>
#include <cstdio>

class X10ApiBase
{
protected:
    X10_REG x10_reg_;
    Crc crc_;

private:
    /* data */
    void RS485_send_driver(uint8_t id, uint8_t data_length, uint8_t *p_buffer);
    std::vector<uint8_t> send_data_serial_;
    std::vector<uint8_t> receive_data_serial_;
    int8_t error_check();

public:
    SerialComm serial_;
    X10ApiBase();
    ~X10ApiBase();
    void get_port_address(std::string &port);
    uint16_t Crc_check(uint8_t *DATA);

    // Implemented:
    void rmdX10_init();
    void rmdX10_shut_down();

    int8_t Motor_read_pid(uint8_t id, uint8_t *data_arr);                                                                         // 2.1
    int8_t Write_pid_RAM(uint8_t id, uint8_t CurrKP, uint8_t CurrKI, uint8_t SpdKP, uint8_t SpdKI, uint8_t PosKP, uint8_t PosKI); // 2.2
    int8_t Write_pid_ROM(uint8_t id, uint8_t CurrKP, uint8_t CurrKI, uint8_t SpdKP, uint8_t SpdKI, uint8_t PosKP, uint8_t PosKI); // 2.3
    int8_t Motor_read_accel(uint8_t id, int32_t &get_accel);                                                                      // 2.4
    int8_t Write_accel_ALL(uint8_t id, uint8_t index, uint32_t accel);                                                            // 2.5
    int8_t multi_turn_current_pos(uint8_t id, int32_t &get_current_enc_pose);                                                     // 2.6
    int8_t multi_turn_original_pos(uint8_t id, int32_t &get_original_enc_pose);                                                   // 2.7
    int8_t multi_turn_zero_off_pos(uint8_t id, int32_t get_zero_off_pose);                                                        // 2.8
    int8_t set_multi_turn_zero(uint8_t id, int32_t offset);                                                                       // 2.9
    int8_t set_current_turn_zero(uint8_t id);                                                                                     // 2.10
    int8_t get_single_enc(uint8_t id, int16_t *data_arr);                                                                         // 2.11
    int8_t get_multi_angle(uint8_t id, int32_t motorAngle);                                                                       // 2.12
    int8_t get_single_angle(uint8_t id, int16_t motorAngle);                                                                      // 2.13
    int8_t Motor_state1(uint8_t id, int16_t *data_arr);                                                                           // 2.14
    int8_t Motor_state2(uint8_t id, int16_t *data_arr);                                                                           // 2.15
    int8_t Motor_state3(uint8_t id, int16_t *data_arr);                                                                           // 2.16
    int8_t Motor_shut_down(uint8_t id);                                                                                           // 2.17
    int8_t Motor_stop(uint8_t id);                                                                                                // 2.18
    int8_t torqueControl(uint8_t id, int32_t iqControl);                                                                          // 2.19
    int8_t speedControl(uint8_t id, int32_t speed);                                                                               // 2.20
    int8_t speedControl(uint8_t id, int32_t speed, int16_t &get_speed);                                                           // 2.20
    int8_t abs_pose_control(uint8_t id, uint16_t maxSpeed, int32_t angle);                                                        // 2.21
    int8_t single_turn(uint8_t id, uint8_t spinDir, uint16_t maxSpeed, uint16_t angle);                                           // 2.22
    int8_t increment_control(uint8_t id, uint16_t maxSpeed, int32_t angle);                                                       // 2.23
    int8_t Motor_mode(uint8_t id, uint8_t &mode);                                                                                 // 2.24
    int8_t get_motor_power(uint8_t id);                                                                                           // 2.25
    void Motor_reset(uint8_t id);                                                                                                 // 2.26
    int8_t Motor_brake_release(uint8_t id);                                                                                       // 2.27
    int8_t Motor_brake_lock(uint8_t id);                                                                                          // 2.28

    int8_t Motor_runtime(uint8_t id, uint32_t &runTime);                                                                          // 2.29
    int8_t Motor_edition(uint8_t id, uint32_t &sysDate);                                                                          // 2.30
    int8_t Motor_comm_protect(uint8_t id, uint32_t commProtect);                                                                  // 2.31
    void Motor_baudrate(uint8_t id, uint8_t baudrate);                                                                            // 2.32
    int8_t Motor_model(uint8_t id, uint8_t *data_arr);                                                                             // 2.33
    int8_t Motor_function(uint8_t id, uint8_t index, uint32_t Value);                                                             // 2.34                                                             // 6.1

    int8_t set_Motor_id(uint8_t id, uint8_t newID);                                                                               // 7.0a
    int8_t get_Motor_id(uint8_t id, uint16_t &getID);                                                                             // 7.0b

};



#endif // BASE_X10_API_BASE_H
