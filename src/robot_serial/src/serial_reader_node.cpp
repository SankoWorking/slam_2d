#include "robot_serial/serial_reader_node.hpp"

SerialReaderNode::SerialReaderNode () : Node("serial_reader_node"){
    running_ = true;
    data_ready_ = false;
    shared_buffer_.reserve(1024);

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
    uint8_t bcc = 0x00;

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
                auto bcc_end = work_buffer.begin() + FRAME_SIZE - 2;
                for (auto i = work_buffer.begin(); i != bcc_end; ++i) {
                    bcc ^= *i;
                }

                if (bcc == *(work_buffer.begin() + FRAME_SIZE - 2)) {
                    std::stringstream ss;
                    ss << "Frame: [ ";
                    for (size_t i = 0; i<24; ++i) {
                        ss << std::hex << std::setw(2) << std::setfill('0') << (int)work_buffer[i] << " ";
                    }
                    ss << "]";
                    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

                    work_buffer.erase(work_buffer.begin(), work_buffer.begin() + FRAME_SIZE);
                }else {
                    work_buffer.erase(work_buffer.begin());
                }
                bcc = 0x00;
            }else {
                 work_buffer.erase(work_buffer.begin());
            }
        }
    }
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialReaderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}