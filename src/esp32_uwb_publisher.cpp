#include "esp32_uwb/esp32_uwb_publisher.hpp"
#include <thread>
#include <mutex>
#include <atomic>

UwbPublisher::UwbPublisher()
    : Node("esp32_uwb_publisher"),
      range_pub_(),
      header_pub_(),
      timer_(),
      ser_(),
      integer_regex_("from: (\\d+)"),
      range_regex_("Range: ([\\d\\.]+) m"),
      init_okay_(false),
      is_pub_once_(false),
      stop_serial_thread_(false) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Start to initialize");

    this->declare_parameter("port", rclcpp::ParameterType::PARAMETER_STRING);
    this->declare_parameter("baud_rate", rclcpp::ParameterType::PARAMETER_INTEGER);
    this->declare_parameter("rate", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("anchor_id", rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY);

    std::vector<int64_t> anchor_id_long;
    if (!this->get_parameter("port", port_) ||
        !this->get_parameter("baud_rate", baud_rate_) ||
        !this->get_parameter("rate", timer_rate_) ||
        !this->get_parameter("anchor_id", anchor_id_long)) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Parameter missing");
        return;
    }

    anchor_id_.clear();
    anchor_id_.reserve(anchor_id_long.size());
    for (int64_t id : anchor_id_long) {
        anchor_id_.push_back(static_cast<int>(id));
    }
    anchor_num_ = anchor_id_.size();
    RCLCPP_INFO_STREAM(this->get_logger(), "num of anchors: " << anchor_num_);
    range_ = std::vector<float>(anchor_num_, 0.0);

    range_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("uwb_range", 10);
    header_pub_ = this->create_publisher<std_msgs::msg::Header>("uwb_header", 10);

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / timer_rate_),
        std::bind(&UwbPublisher::timer_callback, this));
    timer_->cancel();

    RCLCPP_INFO_STREAM(this->get_logger(), "Complete initialization");
    init_okay_ = true;
}

UwbPublisher::~UwbPublisher() {
    stop_serial_thread_ = true;
    if (serial_thread_.joinable()) {
        serial_thread_.join();
    }
    ser_.close();
    timer_->cancel();
}

bool UwbPublisher::init_okay() {
    return init_okay_;
}

bool UwbPublisher::init_serial() {
    try {
        ser_.setPort(port_);
        ser_.setBaudrate(baud_rate_);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser_.setTimeout(to);
        ser_.open();
    } catch (serial::IOException& e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to open port");
        return false;
    }

    if (ser_.isOpen()) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Serial Port initialized");
        return true;
    } else {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot open Serial Port");
        return false;
    }
}

void UwbPublisher::start() {
    stop_serial_thread_ = false;
    serial_thread_ = std::thread(&UwbPublisher::serial_read_loop, this);
    timer_->reset();
}

void UwbPublisher::serial_read_loop() {
    while (!stop_serial_thread_ && rclcpp::ok()) {
        if (!ser_.available()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        std::string data = ser_.readline(ser_.available());
        int id;
        double range;

        if (std::regex_search(data, match_, integer_regex_)) {
            id = std::stoi(match_[1]);
            if (std::regex_search(data, match_, range_regex_)) {
                range = std::stod(match_[1]);
                auto it = std::find(anchor_id_.begin(), anchor_id_.end(), id);
                if (it != anchor_id_.end()) {
                    int index = std::distance(anchor_id_.begin(), it);
                    std::lock_guard<std::mutex> lock(range_mutex_);
                    range_[index] = range;
                }
            }
        }
    }
}

void UwbPublisher::timer_callback() {
    check_and_pub();
}

void UwbPublisher::check_and_pub() {
    std_msgs::msg::Float32MultiArray msg_range;
    {
        std::lock_guard<std::mutex> lock(range_mutex_);
        msg_range.data = range_;
    }

    range_pub_->publish(msg_range);

    std_msgs::msg::Header msg_header;
    msg_header.stamp = this->now();
    header_pub_->publish(msg_header);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UwbPublisher>();
    if (!node->init_okay() || !node->init_serial()) {
        return -1;
    }
    node->start();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

