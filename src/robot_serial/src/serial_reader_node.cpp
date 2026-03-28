#include "robot_serial/serial_reader_node.hpp"

SerialReaderNode::SerialReaderNode () : Node("serial_reader_node"){
    running_ = true;
    data_ready_ = false;
    shared_buffer_.reserve(1024);
    node_clock_ = this->get_clock();
    vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("velocity", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&SerialReaderNode::cmdVelCallback, this, std::placeholders::_1));
    try {
        serial_port_.Open("/dev/ttyACM0");
        serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        RCLCPP_INFO(this->get_logger(), "Serial Started:/dev/ttyACM0");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Can't open serial. %s", e.what());
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
    if (serial_port_.IsOpen()) serial_port_.Close();
    RCLCPP_INFO(this->get_logger(), "Node Closed!");
}



/**
 * @brief cmd_vel订阅者的回调函数。
 * @param TODO
 */
void SerialReaderNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
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
        {
            std::lock_guard<std::mutex> serial_lock(serial_mutex_);
            serial_port_.Write(frame); 
        }
        RCLCPP_INFO(this->get_logger(), "Sent: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X...", frame[0], frame[1], frame[2], frame[3], frame[4], frame[5], frame[6], frame[7], frame[8], frame[9], frame[10]);
}



/**
 * @brief 串口读取线程回调函数
 */
void SerialReaderNode::readThread() {
    RCLCPP_INFO(this->get_logger(), "Read Thread Started");
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
                    std::lock_guard<std::mutex> lock(buffer_mutex_);
                    shared_buffer_.insert(shared_buffer_.end(), temp_buffer.begin(), temp_buffer.end());
                }
                data_ready_ = true;
                buffer_cond_.notify_one();
            }

        } catch (const LibSerial::ReadTimeout&) {
            continue;
        }
    }
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