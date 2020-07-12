#pragma once

#include "ros/ros.h"
#include <serial/serial.h>
#include <vector>

namespace imu_sensor
{
  class IMUData {
    public:
      double yaw;
      double pitch;
      double roll;
      double x_acc; // m/s^2 
      double y_acc; // m/s^2 
      double z_acc; // m/s^2 
      int16_t x_acc_mg;  
      int16_t y_acc_mg;  
      int16_t z_acc_mg;  
  };

    class IMUSerial
    {
    public:
        IMUSerial(const char *port, int baud);
        ~IMUSerial();
        bool connect();
        void closeConnection();
        void readAndParse();
        bool hasNewData() const;
        const IMUData& getData();

    private:
        void parse(uint8_t byte);
        uint8_t calcCS(const std::vector<uint8_t>& data);
        void processData(const std::vector<uint8_t>& data);
        double convertMgToMsSquared(int16_t mg);

    private:
        const char *port_;
        int baud_;
        serial::Serial *serial_;

        enum class State {
            WaitHeader, 
            WaitData,
            WaitCS
        };

        State state_;
        uint32_t n_csum_miss_;
        uint32_t n_msg_misses_;
        IMUData data_;
        bool has_new_msg_;

        const uint16_t HEADER = 0xAAAA;

        const int32_t MESSAGE_LEN = 19;
        const int32_t HEADER_LEN = 2;
        const int32_t CS_LEN = 1;
        const int32_t DATA_FIELD_LEN = MESSAGE_LEN - HEADER_LEN - CS_LEN;
    };
}
