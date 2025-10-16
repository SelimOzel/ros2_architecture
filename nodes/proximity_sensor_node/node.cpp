#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class ProximitySensorNode : public rclcpp::Node
{
public:
  ProximitySensorNode()
  : Node("ProximitySensorNode")
  {
    frequency_hz_ = this->declare_parameter<double>("frequency_hz", 0.5);     // Hz
    amplitude_mm_ = this->declare_parameter<double>("amplitude_mm", 1200.0);  // peak (max) in mm
    start_time_ = this->get_clock()->now();
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("proximity_sensor", 10);
    timer_ = this->create_wall_timer(
      100ms, std::bind(&ProximitySensorNode::timer_callback, this));
  }

private:
  double generate_sine_mm(double t_sec) const
  {
    constexpr double PI = 3.14159265358979323846;
    // Base sine is [-1, +1]; map to [0, amplitude_mm_]
    double omega = 2.0 * PI * frequency_hz_;
    double normalized = (std::sin(omega * t_sec) + 1.0) * 0.5;  // [0,1]
    return normalized * amplitude_mm_;                          // [0, amplitude_mm_]
  }

  void timer_callback()
  {
    const rclcpp::Time now = this->get_clock()->now();
    const double t_sec = (now - start_time_).seconds();
    std_msgs::msg::Float64 msg;
    msg.data = generate_sine_mm(t_sec);
    RCLCPP_INFO(this->get_logger(),
                "Publishing sine(mm): %.3f  (freq=%.3f Hz, peak=%.1f mm, t=%.2f s)",
                msg.data, frequency_hz_, amplitude_mm_, t_sec);
    publisher_->publish(msg);
  }

  rclcpp::Time start_time_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  
  double frequency_hz_;
  double amplitude_mm_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProximitySensorNode>());
  rclcpp::shutdown();
  return 0;
}
