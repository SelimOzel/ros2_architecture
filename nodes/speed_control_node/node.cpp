#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;

class SpeedControlNode : public rclcpp::Node
{
public:
  SpeedControlNode()
  : Node("SpeedControlNode"),
    estop_active_(false),
    last_mode_("<unset>"),
    last_proximity_mm_(std::nan(""))
  {
    // Subscribe to proximity in mm (Float64)
    proximity_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "proximity_sensor", 10, std::bind(&SpeedControlNode::proximity_callback, this, _1));

    // Subscribe to emergency stop (Bool)
    estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "emergency_stop", 10, std::bind(&SpeedControlNode::estop_callback, this, _1));

    // Publisher for current max speed command (String)
    max_speed_pub_ = this->create_publisher<std_msgs::msg::String>("max_speed", 10);

    RCLCPP_INFO(this->get_logger(),
      "SpeedControlNode ready. Subscribing to 'proximity'(Float64) and 'emergency_stop'(Bool); "
      "publishing 'max_speed'(String).");
  }

private:
  // Decide max speed based on proximity in mm:
  //   0..400   => STOP
  //   400..800 => SLOW
  //   >800     => FULL_SPEED
  static std::string decide_speed(double proximity_mm)
  {
    if (proximity_mm <= 400.0) {
      return "STOP";
    } else if (proximity_mm <= 800.0) {
      return "SLOW";
    } else {
      return "FULL_SPEED";
    }
  }

  // Publish only when mode changes
  void maybe_publish_mode(const std::string &mode)
  {
    if (mode == last_mode_) return;
    std_msgs::msg::String out;
    out.data = mode;
    max_speed_pub_->publish(out);
    last_mode_ = mode;
    RCLCPP_INFO(this->get_logger(), "max_speed -> %s", mode.c_str());
  }

  void apply_and_publish()
  {
    // Base mode from proximity (if we have one)
    std::string base_mode = "SLOW";
    if (!std::isnan(last_proximity_mm_)) {
      base_mode = decide_speed(last_proximity_mm_);
    }

    // E-stop overrides to STOP when active
    const std::string mode = estop_active_ ? std::string("STOP") : base_mode;
    maybe_publish_mode(mode);
  }

  void proximity_callback(const std_msgs::msg::Float64 & msg)
  {
    last_proximity_mm_ = msg.data;
    apply_and_publish();
  }

  void estop_callback(const std_msgs::msg::Bool & msg)
  {
    estop_active_ = msg.data;
    apply_and_publish();
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr proximity_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr max_speed_pub_;

  bool estop_active_;
  std::string last_mode_;
  double last_proximity_mm_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpeedControlNode>());
  rclcpp::shutdown();
  return 0;
}
