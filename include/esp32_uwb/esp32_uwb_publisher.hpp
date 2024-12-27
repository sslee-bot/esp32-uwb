#ifndef SSLEE_BOT_ESP32_UWB
#define SSLEE_BOT_ESP32_UWB

#include <serial/serial.h>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <sstream>
#include <string>
#include <vector>
#include "std_msgs/msg/header.hpp"
// #include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std;
using namespace std::chrono_literals;
using std::placeholders::_1;

class UwbPublisher : public rclcpp::Node {
public:
    UwbPublisher();
    virtual ~UwbPublisher();
    virtual bool init_serial();
    virtual bool init_okay();
    virtual void start();

private:
    virtual void timer_callback();
    virtual void check_and_pub();

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr range_pub_;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr header_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    serial::Serial ser_;
    string port_;
    int baud_rate_;
    double timer_rate_;  // Timer callback rate
    double pub_rate_;
    vector<int64_t> anchor_id_;

    int anchor_num_;
    vector<float> range_;
    bool init_okay_;

    rclcpp::Time last_pub_time_;
    bool is_pub_once_;
};

#endif