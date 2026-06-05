#include "robot_serial/serial_reader_node.hpp"

#include <sstream>

SerialReaderNode::SerialReaderNode () : Node("serial_reader_node"){
    running_ = true;
    data_ready_ = false;
    serial_connected_ = false;
    last_write_ok_ = false;
    has_last_cmd_ = false;
    has_last_read_ = false;
    last_cmd_time_ = 0.0;
    last_read_time_ = 0.0;
    last_cmd_linear_x_ = 0.0;
    last_cmd_linear_y_ = 0.0;
    last_cmd_angular_z_ = 0.0;
    shared_buffer_.reserve(1024);
    node_clock_ = this->get_clock();
    serial_port_name_ = this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("velocity", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/chassis/status", 10);
    status_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&SerialReaderNode::publishStatus, this));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&SerialReaderNode::cmdVelCallback, this, std::placeholders::_1));
    try {
        serial_port_.Open(serial_port_name_);
        serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        {
            std::lock_guard<std::mutex> status_lock(status_mutex_);
            serial_connected_ = true;
            last_write_ok_ = true;
            last_error_.clear();
        }
        RCLCPP_INFO(this->get_logger(), "Serial started: %s", serial_port_name_.c_str());
    } catch (const std::exception &e) {
        {
            std::lock_guard<std::mutex> status_lock(status_mutex_);
            serial_connected_ = false;
            last_write_ok_ = false;
            last_error_ = e.what();
        }
        RCLCPP_ERROR(
            this->get_logger(),
            "Can't open serial port %s: %s",
            serial_port_name_.c_str(),
            e.what());
        running_ = false;
    }
    if (running_){
        read_thread_ = std::thread(&SerialReaderNode::readThread, this);
        parse_thread_ = std::thread(&SerialReaderNode::parseThread, this);
    }
}

SerialReaderNode::~SerialReaderNode () {
    running_ = false;
    buffer_cond_.notify_all();
    if (read_thread_.joinable()) read_thread_.join();
    if (parse_thread_.joinable()) parse_thread_.join();
    {
        std::lock_guard<std::mutex> serial_lock(serial_mutex_);
        if (serial_port_.IsOpen()) serial_port_.Close();
    }
    RCLCPP_DEBUG(this->get_logger(), "Node closed");
}



/**
 * @brief cmd_vel订阅者的回调函数。
 * @param TODO
 */
void SerialReaderNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        has_last_cmd_ = true;
        last_cmd_time_ = node_clock_->now().seconds();
        last_cmd_linear_x_ = msg->linear.x;
        last_cmd_linear_y_ = msg->linear.y;
        last_cmd_angular_z_ = msg->angular.z;
    }

    int16_t x_speed = static_cast<int16_t>(msg->linear.x * 1000.0);
    int16_t y_speed = static_cast<int16_t>(msg->linear.y * 1000.0);
    int16_t z_speed = static_cast<int16_t>(msg->angular.z * 1000.0);

    std::vector<uint8_t> frame(11);
    frame[0] = 0x7B;
    frame[1] = 0x00;
    frame[2] = 0x00;

    frame[3] = (x_speed >> 8) & 0xFF;
    frame[4] = x_speed & 0xFF;

    frame[5] = (y_speed >> 8) & 0xFF;
    frame[6] = y_speed & 0xFF;

    frame[7] = (z_speed >> 8) & 0xFF;
    frame[8] = z_speed & 0xFF;

    uint8_t bcc_check = 0;
    for (int i = 0; i < 9; i++) {
        bcc_check ^= frame[i];
    }
    frame[9] = bcc_check;

    frame[10] = 0x7D;
    try {
        std::lock_guard<std::mutex> serial_lock(serial_mutex_);
        if (!serial_port_.IsOpen()) {
            std::lock_guard<std::mutex> status_lock(status_mutex_);
            serial_connected_ = false;
            last_write_ok_ = false;
            last_error_ = "serial port is not open";
            return;
        }
        serial_port_.Write(frame);
    } catch (const std::exception &e) {
        {
            std::lock_guard<std::mutex> status_lock(status_mutex_);
            serial_connected_ = false;
            last_write_ok_ = false;
            last_error_ = e.what();
        }
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *node_clock_,
            2000,
            "Failed to write cmd_vel to serial: %s",
            e.what());
        return;
    }
    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        serial_connected_ = true;
        last_write_ok_ = true;
        last_error_.clear();
    }
    RCLCPP_DEBUG(this->get_logger(), "Sent cmd_vel frame to chassis");
}



/**
 * @brief 串口读取线程回调函数
 */
void SerialReaderNode::readThread() {
    RCLCPP_DEBUG(this->get_logger(), "Serial read thread started");
    LibSerial::DataBuffer temp_buffer;
    temp_buffer.reserve(128);
    while (rclcpp::ok() && running_) {
        try {
            {
                std::lock_guard<std::mutex> serial_lock(serial_mutex_);
                temp_buffer.clear();
                serial_port_.Read(temp_buffer, 24, 10);
            }
            if (!temp_buffer.empty()) {
                {
                    std::lock_guard<std::mutex> status_lock(status_mutex_);
                    has_last_read_ = true;
                    last_read_time_ = node_clock_->now().seconds();
                    serial_connected_ = true;
                }
                {
                    std::lock_guard<std::mutex> lock(buffer_mutex_);
                    shared_buffer_.insert(shared_buffer_.end(), temp_buffer.begin(), temp_buffer.end());
                }
                data_ready_ = true;
                buffer_cond_.notify_one();
            }

        } catch (const LibSerial::ReadTimeout&) {
            continue;
        } catch (const std::exception &e) {
            {
                std::lock_guard<std::mutex> status_lock(status_mutex_);
                serial_connected_ = false;
                last_write_ok_ = false;
                last_error_ = e.what();
            }
            RCLCPP_WARN(this->get_logger(), "Serial read stopped: %s", e.what());
            running_ = false;
            buffer_cond_.notify_all();
        }
    }
}

std::string SerialReaderNode::jsonEscape(const std::string& value) {
    std::ostringstream out;
    for (char c : value) {
        switch (c) {
            case '"':
                out << "\\\"";
                break;
            case '\\':
                out << "\\\\";
                break;
            case '\n':
                out << "\\n";
                break;
            case '\r':
                out << "\\r";
                break;
            case '\t':
                out << "\\t";
                break;
            default:
                out << c;
                break;
        }
    }
    return out.str();
}

void SerialReaderNode::publishStatus() {
    const double now = node_clock_->now().seconds();
    bool port_open;
    bool connected;
    bool last_write_ok;
    bool has_last_cmd;
    bool has_last_read;
    double last_cmd_age;
    double last_read_age;
    double last_cmd_linear_x;
    double last_cmd_linear_y;
    double last_cmd_angular_z;
    std::string last_error;

    {
        std::lock_guard<std::mutex> serial_lock(serial_mutex_);
        port_open = serial_port_.IsOpen();
    }

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        connected = serial_connected_ && port_open;
        last_write_ok = last_write_ok_;
        has_last_cmd = has_last_cmd_;
        has_last_read = has_last_read_;
        last_cmd_age = has_last_cmd_ ? now - last_cmd_time_ : -1.0;
        last_read_age = has_last_read_ ? now - last_read_time_ : -1.0;
        last_cmd_linear_x = last_cmd_linear_x_;
        last_cmd_linear_y = last_cmd_linear_y_;
        last_cmd_angular_z = last_cmd_angular_z_;
        last_error = last_error_;
    }

    std::ostringstream data;
    data << "{"
         << "\"port\":\"" << jsonEscape(serial_port_name_) << "\","
         << "\"port_open\":" << (port_open ? "true" : "false") << ","
         << "\"connected\":" << (connected ? "true" : "false") << ","
         << "\"running\":" << (running_.load() ? "true" : "false") << ","
         << "\"last_write_ok\":" << (last_write_ok ? "true" : "false") << ","
         << "\"has_last_cmd\":" << (has_last_cmd ? "true" : "false") << ","
         << "\"has_last_read\":" << (has_last_read ? "true" : "false") << ","
         << "\"last_cmd_age\":" << last_cmd_age << ","
         << "\"last_read_age\":" << last_read_age << ","
         << "\"last_cmd\":{"
         << "\"linear_x\":" << last_cmd_linear_x << ","
         << "\"linear_y\":" << last_cmd_linear_y << ","
         << "\"angular_z\":" << last_cmd_angular_z << "},"
         << "\"last_error\":\"" << jsonEscape(last_error) << "\""
         << "}";

    auto status_msg = std_msgs::msg::String();
    status_msg.data = data.str();
    status_pub_->publish(status_msg);
}



/**
 * @brief 串口解析线程回调函数
 */
void SerialReaderNode::parseThread() {
    std::vector<uint8_t> work_buffer;
    work_buffer.reserve(1024);

    while (rclcpp::ok() && running_) {
        std::vector<uint8_t> incomming_data;
        {
            std::unique_lock<std::mutex> lock(buffer_mutex_);
            buffer_cond_.wait(lock, [this] {return data_ready_ || !running_;});
            
            if (!running_) break;

            incomming_data.swap(shared_buffer_);
            data_ready_ = false;
        }
        if (!incomming_data.empty()) {
            work_buffer.insert(work_buffer.end(), incomming_data.begin(), incomming_data.end());
        }

        const size_t FRAME_LEN = FRAME_SIZE;
        while (work_buffer.size() >= FRAME_LEN) {
            if (work_buffer[0] == 0x7B && work_buffer[23] == 0x7D) {
                if (checkBCC(work_buffer.begin(), work_buffer.begin() + FRAME_SIZE - 2)) {
                    processFrame(work_buffer);
                }else {
                    work_buffer.erase(work_buffer.begin());
                }
            }else {
                 work_buffer.erase(work_buffer.begin());
            }
        }
    }
}

bool SerialReaderNode::checkBCC(std::vector<uint8_t>::iterator first, std::vector<uint8_t>::iterator last) {
    uint8_t bcc = 0x00;
    for (auto i = first; i != last; ++i) {
        bcc ^= *i;
    }
    if (bcc == *last) {
        return true;
    }else {
        return false;
    }
}

int16_t SerialReaderNode::calVel(uint8_t high, uint8_t low) {
    return (int16_t)((high << 8) | low);
}

float SerialReaderNode::calAcc(uint8_t high, uint8_t low) {
    int16_t raw_value = static_cast<int16_t>((high << 8) | low);
    return raw_value / 1672.0f;
}

float SerialReaderNode::calAng(uint8_t high, uint8_t low) {
    int16_t raw_value = static_cast<int16_t>((high << 8) | low);
    return raw_value / 3753.0f;
}

void SerialReaderNode::processFrame(std::vector<uint8_t>& buffer) {
    auto vel_msg = geometry_msgs::msg::TwistStamped();
    auto imu_msg = sensor_msgs::msg::Imu();
    auto now = node_clock_->now();
    vel_msg.header.stamp = now;
    vel_msg.twist.linear.x = calVel(buffer[2], buffer[3]) / 1000.0;
    vel_msg.twist.linear.y = calVel(buffer[4], buffer[5]) / 1000.0;
    vel_msg.twist.linear.z = calVel(buffer[6], buffer[7]) / 1000.0;
    vel_msg.twist.angular.x = calAng(buffer[14], buffer[15]);
    vel_msg.twist.angular.y = calAng(buffer[16], buffer[17]);
    vel_msg.twist.angular.z = calAng(buffer[18], buffer[19]);

    imu_msg.header.stamp = now;
    imu_msg.header.frame_id = "imu_link";
    imu_msg.linear_acceleration.x = calAcc(buffer[8], buffer[9]);
    imu_msg.linear_acceleration.y = calAcc(buffer[10], buffer[11]);
    imu_msg.linear_acceleration.z = calAcc(buffer[12], buffer[13]);
    imu_msg.angular_velocity.x = calAng(buffer[14], buffer[15]);
    imu_msg.angular_velocity.y = calAng(buffer[16], buffer[17]);
    imu_msg.angular_velocity.z = calAng(buffer[18], buffer[19]);


    vel_pub_->publish(vel_msg);
    imu_pub_->publish(imu_msg);

    buffer.erase(buffer.begin(), buffer.begin() + FRAME_SIZE);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialReaderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
