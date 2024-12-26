#include "esp32_uwb/esp32_uwb_publisher.hpp"

UwbPublisher::UwbPublisher()
    : Node("esp32_uwb_publisher"),
      range_pub_(),
      header_pub_(),
      timer_(),
      ser_(),
      init_okay_(false),
      is_pub_once_(false)
{
    // Log
    RCLCPP_INFO_STREAM(this->get_logger(), "Start to initialize");

    // parameters
    this->declare_parameter("port", rclcpp::ParameterType::PARAMETER_STRING);
    this->declare_parameter("baud_rate", rclcpp::ParameterType::PARAMETER_INTEGER);
    this->declare_parameter("timer_rate", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("pub_rate", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("anchor_id", rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY);

    if (!this->get_parameter("port", port_))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "no parameter: port");
        return;
    }
    if (!this->get_parameter("baud_rate", baud_rate_))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "no parameter: baud_rate");
        return;
    }
    if (!this->get_parameter("timer_rate", timer_rate_))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "no parameter: timer_rate");
        return;
    }
    if (!this->get_parameter("pub_rate", pub_rate_))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "no parameter: pub_rate");
        return;
    }
    if (!this->get_parameter("anchor_id", anchor_id_))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "no parameter: anchor_id");
        return;
    }

    anchor_num_ = anchor_id_.size();
    RCLCPP_INFO_STREAM(this->get_logger(), "num of anchors: " << anchor_num_);
    range_ = vector<float>(anchor_num_, 0.0);

    // port_ = this->declare_parameter("port", string("/dev/ttyACM0"));
    // baud_rate_ = this->declare_parameter("baud_rate", int(115200));
    // timer_rate_ = this->declare_parameter("timer_rate", double(10));

    // publisher
    range_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("uwb_range", 1000);
    header_pub_ = this->create_publisher<std_msgs::msg::Header>("uwb_header", 1000);

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
    if (is_pub_once_ == false)
    {
        last_pub_time_ = this->now();
    }

    if (ser_.available())
    {
        string data = ser_.read(ser_.available());

        regex integer_regex("from: (\\d+)");
        regex range_regex("Range: ([\\d\\.]+) m");
        smatch match;
        int id;
        double range;

        if (regex_search(data, match, integer_regex))
        {
            id = stoi(match[1]);
            if (regex_search(data, match, range_regex))
            {
                range = stod(match[1]);
            }
            else
            {
                RCLCPP_WARN_STREAM(
                    this->get_logger(), "no range value is included: " << data);
                return;
            }
        }
        else
        {
            RCLCPP_WARN_STREAM(
                this->get_logger(), "no anchor ID value is included: " << data);
            return;
        }

        // for debug
        // RCLCPP_WARN_STREAM(this->get_logger(), "id: " << id << ", range: " << range);

        // update range vector
        auto it = find(anchor_id_.begin(), anchor_id_.end(), id);
        int index = 0;
        if (it != anchor_id_.end())
        {
            index = distance(anchor_id_.begin(), it);
            range_[index] = range;
        }
        else
        {
            // it is currently unused anchor, ignore it
            return;
        }

        // check duration
        auto now = this->now();
        auto duration = now - last_pub_time_;
        // RCLCPP_WARN_STREAM(this->get_logger(), duration.seconds());
        if (!is_pub_once_ || duration.seconds() > 1.0 / pub_rate_)
        {
            // publish
            std_msgs::msg::Float32MultiArray msg_range;
            msg_range.data = range_;
            range_pub_->publish(msg_range);

            std_msgs::msg::Header msg_header;
            msg_header.stamp = now;
            header_pub_->publish(msg_header);

            is_pub_once_ = true;
            last_pub_time_ = now;
        }

        // for debug
        // RCLCPP_WARN_STREAM(this->get_logger(), data);
    }
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