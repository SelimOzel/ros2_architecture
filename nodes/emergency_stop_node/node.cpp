#include <chrono>
#include <functional>
#include <memory>
#include <cstdio>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

// RAII helper to put stdin into noncanonical, non-echo, nonblocking mode
class StdinRawMode {
public:
  StdinRawMode() : active_(false) {
    if (!isatty(STDIN_FILENO)) return;

    if (tcgetattr(STDIN_FILENO, &orig_) == -1) return;

    termios raw = orig_;
    raw.c_lflag &= ~(ICANON | ECHO);   // no canonical mode, no echo
    raw.c_cc[VMIN]  = 0;               // non-blocking read
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == -1) return;

    // Also set O_NONBLOCK so read() doesn't block
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags != -1) {
      fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }
    active_ = true;
  }

  ~StdinRawMode() {
    if (active_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &orig_);
      // best-effort: restore blocking flag
      int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
      if (flags != -1) fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
    }
  }

private:
  termios orig_{};
  bool active_;
};

class EmergencyStopNode : public rclcpp::Node
{
public:
  EmergencyStopNode()
  : Node("EmergencyStopNode"), estop_state_(false)
  {
    // Put stdin into raw nonblocking so we can poll keys
    raw_mode_ = std::make_unique<StdinRawMode>();
    publisher_ = this->create_publisher<std_msgs::msg::Bool>("emergency_stop", 10);
    timer_ = this->create_wall_timer(
      50ms, std::bind(&EmergencyStopNode::timer_callback, this));
    RCLCPP_INFO(this->get_logger(),
      "EmergencyStopNode started. Press 'E' to toggle emergency stop (topic: 'emergency_stop').");
  }

private:
  // Read all available chars from stdin; return true if any 'E'/'e' was seen.
  bool poll_for_E_key()
  {
    bool saw_e = false;
    unsigned char buf[64];
    ssize_t n = 0;

    // Drain available bytes this tick
    while ((n = ::read(STDIN_FILENO, buf, sizeof(buf))) > 0) {
      for (ssize_t i = 0; i < n; ++i) {
        if (buf[i] == 'e' || buf[i] == 'E') {
          saw_e = true;
        }
      }
    }
    // n == -1 with EAGAIN is fine (no data)
    return saw_e;
  }

  void publish_state(bool state)
  {
    std_msgs::msg::Bool msg;
    msg.data = state;
    publisher_->publish(msg);
    RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP %s",
                state ? "ENGAGED (true)" : "RELEASED (false)");
  }

  void timer_callback()
  {
    if (poll_for_E_key()) {
      estop_state_ = !estop_state_;
      publish_state(estop_state_);
    }
  }

  std::unique_ptr<StdinRawMode> raw_mode_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
  
  bool estop_state_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EmergencyStopNode>());
  rclcpp::shutdown();
  return 0;
}
