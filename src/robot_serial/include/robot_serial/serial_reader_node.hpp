#ifndef SERIAL_READER_NODE_HPP
#define SERIAL_READER_NODE_HPP

#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <libserial/SerialPort.h>

#define FRAME_SIZE 24

class SerialReaderNode : public rclcpp::Node {
public:
    SerialReaderNode();
    ~SerialReaderNode();

private:
    void readThread();
    void parseThread();
    bool checkBCC(std::vector<uint8_t>::iterator first, std::vector<uint8_t>::iterator last);
    void processFrame(std::vector<uint8_t>& buffer);
    LibSerial::SerialPort serial_port_;
    std::mutex buffer_mutex_;
    std::condition_variable buffer_cond_;
    std::thread read_thread_;
    std::thread parse_thread_;
    std::atomic<bool> running_;
    std::atomic<bool> data_ready_;
    std::vector<uint8_t> shared_buffer_;
    
};

#endif