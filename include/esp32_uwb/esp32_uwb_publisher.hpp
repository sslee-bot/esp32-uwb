#ifndef SSLEE_BOT_ESP32_UWB
#define SSLEE_BOT_ESP32_UWB

#include <serial/serial.h>
#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std;
using namespace std::chrono_literals;
using std::placeholders::_1;

class UwbPublisher : public rclcpp::Node
{
public:
    UwbPublisher();
    virtual ~UwbPublisher();
    virtual bool init_serial();
    virtual bool init_okay();
    virtual void start();

private:
    virtual void timer_callback();

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr dist_pub_;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr header_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    serial::Serial ser_;
    string port_;
    int baud_rate_;
    double timer_rate_; // Timer callback rate
    vector<string> anchor_id_;
    int anchor_num_;
    vector<int> dist_;
    bool init_okay_;
};

#endif