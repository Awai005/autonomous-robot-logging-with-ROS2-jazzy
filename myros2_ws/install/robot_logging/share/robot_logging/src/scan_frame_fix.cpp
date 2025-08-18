#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class ScanFrameFix : public rclcpp::Node {
public:
  ScanFrameFix() : rclcpp::Node("scan_frame_fix")
  {
    input_   = this->declare_parameter<std::string>("input", "/scan_raw");
    output_  = this->declare_parameter<std::string>("output", "/scan");
    frame_id_= this->declare_parameter<std::string>("frame_id", "lidar_link");

    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      output_, rclcpp::SensorDataQoS());

    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      input_, rclcpp::SensorDataQoS(),
      std::bind(&ScanFrameFix::callback, this, std::placeholders::_1));
  }

private:
  void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg_in)
  {
    auto msg = *msg_in;              // copy
    msg.header.frame_id = frame_id_; // fix the frame
    pub_->publish(std::move(msg));
  }

  std::string input_, output_, frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanFrameFix>());
  rclcpp::shutdown();
  return 0;
}
