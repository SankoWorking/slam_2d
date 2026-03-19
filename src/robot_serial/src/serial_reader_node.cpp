#include "robot_serial/serial_reader_node.hpp"

SerialReaderNode::SerialReaderNode () : Node("serial_reader_node"){
    running_ = true;
    data_ready_ = false;
    shared_buffer_.reserve(1024);
    node_clock_ = this->get_clock();
    vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("velocity", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
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

void SerialReaderNode::readThread() {
    RCLCPP_INFO(this->get_logger(), "Read Thread Started");
    LibSerial::DataBuffer temp_buffer;
    temp_buffer.reserve(128);
    while (rclcpp::ok() && running_) {
        try {
            temp_buffer.clear();
            serial_port_.Read(temp_buffer, 24, 100);
            
            if (!temp_buffer.empty()) {
                printf("Raw: ");
                for (auto b : temp_buffer) printf("%02X", b);
                printf("\n");
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

void SerialReaderNode::parseThread() {
    RCLCPP_INFO(this->get_logger(), "Parse Thread Started");
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
        RCLCPP_INFO(this->get_logger(), "Parse Thread Started!");
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
    vel_msg.twist.linear.x = calVel(buffer[2], buffer[3]);
    vel_msg.twist.linear.y = calVel(buffer[4], buffer[5]);
    vel_msg.twist.linear.z = calVel(buffer[6], buffer[7]);

    imu_msg.header.stamp = now;
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