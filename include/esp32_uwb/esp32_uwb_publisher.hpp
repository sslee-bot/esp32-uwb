#ifndef ESP32_UWB_PUBLISHER_HPP
#define ESP32_UWB_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <serial/serial.h>
#include <regex>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>

class UwbPublisher : public rclcpp::Node {
public:
    UwbPublisher();
    ~UwbPublisher();

    bool init_okay();
    bool init_serial();
    void start();

private:
    void timer_callback();
    void check_and_pub();
    void serial_read_loop();

    // ROS interfaces
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr range_pub_;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr header_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // serial interface
    serial::Serial ser_;

    // parameters
    std::string port_;
    int baud_rate_;
    double timer_rate_;
    std::vector<int> anchor_id_;
    int anchor_num_;

    // data
    std::vector<float> range_;
    std::regex integer_regex_;
    std::regex range_regex_;
    std::smatch match_;

    // status
    bool init_okay_;
    bool is_pub_once_;

    // thread
    std::thread serial_thread_;
    std::atomic<bool> stop_serial_thread_;
    std::mutex range_mutex_;
};

#endif  // ESP32_UWB_PUBLISHER_HPP

