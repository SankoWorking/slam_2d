#ifndef SERIAL_READER_NODE_HPP
#define SERIAL_READER_NODE_HPP

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <libserial/SerialPort.h>
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

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

    /**
     * @brief 将两个字节转换为x、y、z轴速度(m/s)
     * @param high 帧中的第2、4、6字节，对应x、y、z轴速度
     * @param low 帧中的第3、5、7字节，对应x、y、z轴速度
     * @return 转换后的线速度整数值
     */
    int16_t calVel(uint8_t high, uint8_t low);

    /**
     * @brief 将两个字节转换为加速度(m/s^2)
     * @param high 帧中的第8、10、12字节，对应x、y、z轴加速度
     * @param low 帧中的第9、11、13字节，对应x、y、z轴加速度
     * @return 转换后打加速度浮点值
     */
    float calAcc(uint8_t high, uint8_t low);

    /**
     * @brief 将两个字节转换为角速度(rad/s)
     * @param high 帧中的第14、16、18字节，对应x、y、z轴角速度
     * @param low 帧中的第15、17、19字节，对应x、y、z轴加速度
     * @return 转换后打加速度浮点值
     */
    float calAng(uint8_t high, uint8_t low);
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void publishStatus();
    std::string jsonEscape(const std::string& value);

    LibSerial::SerialPort serial_port_;
    std::string serial_port_name_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::mutex serial_mutex_;
    std::mutex status_mutex_;
    std::mutex buffer_mutex_;
    std::condition_variable buffer_cond_;
    std::thread read_thread_;
    std::thread parse_thread_;
    std::atomic<bool> running_;
    std::atomic<bool> data_ready_;
    std::vector<uint8_t> shared_buffer_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::Clock::SharedPtr node_clock_;
    bool serial_connected_;
    bool last_write_ok_;
    bool has_last_cmd_;
    bool has_last_read_;
    double last_cmd_time_;
    double last_read_time_;
    double last_cmd_linear_x_;
    double last_cmd_linear_y_;
    double last_cmd_angular_z_;
    std::string last_error_;
};

#endif
