#include "esp32_uwb/esp32_uwb_publisher.hpp"

UwbPublisher::UwbPublisher()
    : Node("esp32_uwb_publisher"),
      dist_pub_(),
      header_pub_(),
      timer_(),
      ser_(),
      init_okay_(false)
{
    // Log
    RCLCPP_INFO_STREAM(this->get_logger(), "Start to initialize");

    // parameters
    this->declare_parameter("port", rclcpp::ParameterType::PARAMETER_STRING);
    this->declare_parameter("baud_rate", rclcpp::ParameterType::PARAMETER_INTEGER);
    this->declare_parameter("timer_rate", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("anchor_id", rclcpp::ParameterType::PARAMETER_STRING_ARRAY);

    if (!this->get_parameter("port", port_)) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "no parameter: port");
        return;
    }
    if (!this->get_parameter("baud_rate", baud_rate_)) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "no parameter: baud_rate");
        return;
    }
    if (!this->get_parameter("timer_rate", timer_rate_)) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "no parameter: timer_rate");
        return;
    }
    if (!this->get_parameter("anchor_id", anchor_id_)) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "no parameter: anchor_id");
        return;
    }

    anchor_num_ = anchor_id_.size();
    RCLCPP_INFO_STREAM(this->get_logger(), "num of anchors: " << anchor_num_);
    dist_ = vector<int>(anchor_num_, 0);

    // port_ = this->declare_parameter("port", string("/dev/ttyACM0"));
    // baud_rate_ = this->declare_parameter("baud_rate", int(115200));
    // timer_rate_ = this->declare_parameter("timer_rate", double(10));

    // publisher
    dist_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("uwb_dist", 1000);
    header_pub_ = this->create_publisher<std_msgs::msg::Header>("header_uwb", 1000);

    // timer
    timer_ = this->create_wall_timer(
        1s / timer_rate_, std::bind(&UwbPublisher::timer_callback, this));
    timer_->cancel();

    RCLCPP_INFO_STREAM(this->get_logger(), "Complete initialization");
    init_okay_ = true;
}

UwbPublisher::~UwbPublisher()
{
    ser_.close();
    timer_->cancel();
}

bool UwbPublisher::init_okay()
{
    return init_okay_;
}

bool UwbPublisher::init_serial()
{
    try
    {
        ser_.setPort(port_);
        ser_.setBaudrate(baud_rate_);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser_.setTimeout(to);
        ser_.open();
    }
    catch (serial::IOException &e)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to open port");
        return false;
    }

    if (ser_.isOpen())
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Serial Port initialized");
        return true;
    }
    else
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot open Serial Port");
        return false;
    }
}

void UwbPublisher::start()
{
    timer_->reset();
}

void UwbPublisher::timer_callback()
{
    if (ser_.available()) {
        // TODO
    }

    // For debug
    RCLCPP_WARN_STREAM_THROTTLE(
              this->get_logger(), *(this->get_clock()), 2000,
              "debug");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<UwbPublisher>();
    bool init_okay = node->init_okay();
    if (init_okay == false)
    {
        return -1;
    }

    bool ret = node->init_serial();
    if (ret == false)
    {
        return -1;
    }
    node->start();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}