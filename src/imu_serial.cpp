#include "imu_serial.h"

using namespace std;
using namespace imu_sensor;

// namespace imu_sensor
// {
    /**
     * 
     */
    IMUSerial::IMUSerial(const char *port, int baud) :
        port_(port),
        baud_(baud),
        serial_(nullptr),
        state_(State::WaitHeader),
        n_csum_miss_(0),
        n_msg_misses_(0)
    {
        serial_ = new serial::Serial();

        serial::Timeout to(serial::Timeout::simpleTimeout(500));
        serial_->setTimeout(to);
        serial_->setPort(port_);
        serial_->setBaudrate(baud_);
    }

    /**
     * 
     */
    IMUSerial::~IMUSerial()
    {
        if (serial_) {
            delete serial_;
        }
    }

    /**
     * [IMUSeria::connect description]
     * @return [description]
     */
    bool IMUSerial::connect()
    {
        const int N_ATTEMPTS = 10;
        
        /* Do N attemps */
        for (int n = 0; n < N_ATTEMPTS; n++) {
            try {
                serial_->open();
            } catch (serial::IOException) {
                ROS_WARN("serial IO Exception");
            }

            if (serial_->isOpen()) {
                ROS_INFO("Serial connection successful on port:  %s", port_);
                return true;
            } else {
                ROS_WARN("Bad Connection with serial port Error %s, try number %i", port_, n);
            }
        }
        
        ROS_WARN("Unable to establish serial connection");
        return false; 
    }

    /**
     * [IMUSeria::closeConnection description]
     */
    void IMUSerial::closeConnection()
    {
        if (serial_->isOpen()) {
            serial_->close();
        }
    }

    /**
     * [IMUSerial::readAndParse description]
     */
    void IMUSerial::readAndParse()
    {
        size_t n_bytes_to_read = serial_->available();
        if(!n_bytes_to_read) {
          return;
        }
       
        /* Read */
        /* Read method inserts new data at the end */
        vector<uint8_t> bytes;
        try {
            if (serial_->read(bytes, n_bytes_to_read) != n_bytes_to_read) {
                ROS_INFO("Timeout occured during serial reading");
            }
        } catch (serial::SerialException e) {
            ROS_WARN("Serial exception during reading. What: %s", e.what());
        }

        /* Debug-print read data. Uncomment if you need it */
        /*
        cout << "Received data (hex): ";
        for (auto b : bytes) {
          cout << hex <<"0x" << b;
        }
        cout << endl;
        */

        /* Parse */
        for (auto b : bytes) {
          parse(b);
        }
    }


    /**
     * [IMUSerial::parse description]
     * @param byte [description]
     */
    void IMUSerial::parse(uint8_t byte)
    {
        static vector<uint8_t> data_field(DATA_FIELD_LEN);
        static uint32_t data_i = 0;
        static uint16_t header = 0;

        switch(state_) {
            case State::WaitHeader:
                header = (header << 8) | byte; // Does the operation cast variables to int32 or int16?
                if (header == HEADER) {
                    state_ = State::WaitData;
                    data_i = 0;
                    header = 0;
                }
                break;

            case State::WaitData:
                data_field[data_i++] = byte;
                if (data_i >= DATA_FIELD_LEN) {
                    data_i = 0;
                    state_ = State::WaitCS;
                }
                break;

            case State::WaitCS:
                if (byte == calcCS(data_field)) {
                    processData(data_field);
                } else {
                    n_csum_miss_++;
                    ROS_INFO("Message CS miss: %i", n_csum_miss_);
                    /* Print anyway. Uncomment if you still need it */
                    /*
                    std::cout << "Corrupted data: "; 
                    processData(data_field);
                    std::cout << "**********" << std::endl;
                    */
                }
                state_ = State::WaitHeader;
                break;

            default:
                state_ = State::WaitHeader;
                break;
        }
    }

    /**
     * [IMUSerial::calcCS description]
     * @param  data [description]
     * @return      [description]
     */
    uint8_t IMUSerial::calcCS(const std::vector<uint8_t>& data)
    {
        uint8_t csum = 0;
        for (uint8_t byte : data) {
            csum += byte;
        }
        return csum;
    }

    void IMUSerial::processData(const std::vector<uint8_t>& data)
    {
        static uint8_t prev_msg_index = 0;

        uint8_t msg_index = data.at(0);
 
        data_.yaw     = -(static_cast<int16_t>(static_cast<int16_t>(data.at(1)) | static_cast<int16_t>(data.at(2)) << 8) / 100.0);
        data_.roll    = static_cast<int16_t>(static_cast<int16_t>(data.at(3)) | static_cast<int16_t>(data.at(4)) << 8) / 100.0;
        data_.pitch   = static_cast<int16_t>(static_cast<int16_t>(data.at(5)) | static_cast<int16_t>(data.at(6)) << 8) / 100.0;

        data_.x_acc_mg   = static_cast<int16_t>(data.at(7)) | static_cast<int16_t>(data.at(8)) << 8;
        data_.y_acc_mg   = static_cast<int16_t>(data.at(9)) | static_cast<int16_t>(data.at(10)) << 8;
        data_.z_acc_mg   = static_cast<int16_t>(data.at(11)) | static_cast<int16_t>(data.at(12)) << 8;

        data_.x_acc = convertMgToMsSquared(data_.x_acc_mg);
        data_.y_acc = convertMgToMsSquared(data_.y_acc_mg);
        data_.z_acc = -convertMgToMsSquared(data_.z_acc_mg);

        has_new_msg_ = true;
        ROS_INFO("Index: %i. Yaw, degree: %.2f. Pitch: %.2f. Roll: %.2f. X acc., m/s^2: %.2f. Y acc.: %.2f. Z acc.: %.2f",
            msg_index, data_.yaw, data_.pitch, data_.roll, data_.x_acc, data_.y_acc, data_.z_acc);

        /* Check if there is no any missed packages */
        uint32_t diff = msg_index - prev_msg_index;
        if (diff > 1 && diff != -255) {
            ROS_INFO("Messages missed: %i", n_msg_misses_);
        }
        prev_msg_index = msg_index;    

    }
        
double IMUSerial::convertMgToMsSquared(int16_t mg)
{
  return (9.80665 * static_cast<double>(mg) / 1000.0);
}

bool IMUSerial::hasNewData() const 
{
  return has_new_msg_; 
}

const IMUData& IMUSerial::getData() 
{
  has_new_msg_ = false;
  return data_;
}