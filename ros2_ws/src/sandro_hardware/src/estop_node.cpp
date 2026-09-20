#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <cerrno>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "sandro_hardware/protocol.hpp"

namespace sandro_hardware
{
class EstopNode final : public rclcpp::Node
{
public:
  EstopNode()
  : Node("gim6010_estop")
  {
    interface_ = declare_parameter<std::string>("can_interface", "can0");
    nodes_ = declare_parameter<std::vector<int64_t>>("node_ids", {1, 2, 3, 4, 5, 6});
    service_ = create_service<std_srvs::srv::Trigger>(
      "~/trigger",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        try {
          send_command(protocol::CanCommand::estop);
          response->success = true;
          response->message = "CAN ESTOP sent to configured GIM6010 nodes";
          RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        } catch (const std::exception & error) {
          response->success = false;
          response->message = error.what();
          RCLCPP_ERROR(get_logger(), "ESTOP transmission failed: %s", error.what());
        }
      });
    // The drives latch ESTOP_REQUESTED after the command above. Re-arming the
    // hardware component clears that bit itself; this service is the manual
    // equivalent for inspection without a relaunch. It clears every error, so
    // read the heartbeat first and use it deliberately.
    clear_service_ = create_service<std_srvs::srv::Trigger>(
      "~/clear_errors",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        try {
          send_command(protocol::CanCommand::clear_errors);
          response->success = true;
          response->message = "CAN Clear_Errors sent to configured GIM6010 nodes";
          RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
        } catch (const std::exception & error) {
          response->success = false;
          response->message = error.what();
          RCLCPP_ERROR(get_logger(), "Clear_Errors transmission failed: %s", error.what());
        }
      });
  }

private:
  void send_command(protocol::CanCommand command) const
  {
    const int socket = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket < 0) {
      throw std::runtime_error(std::string("cannot create CAN socket: ") + std::strerror(errno));
    }
    ifreq request{};
    std::strncpy(request.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
    if (::ioctl(socket, SIOCGIFINDEX, &request) < 0) {
      ::close(socket);
      throw std::runtime_error("CAN interface not found: " + interface_);
    }
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = request.ifr_ifindex;
    if (::bind(socket, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
      ::close(socket);
      throw std::runtime_error("cannot bind CAN interface: " + interface_);
    }
    for (const auto node : nodes_) {
      if (node < 0 || node > 63) {
        ::close(socket);
        throw std::runtime_error("node_ids must be in [0, 63]");
      }
      can_frame frame{};
      frame.can_id = protocol::can_id(static_cast<uint8_t>(node), command);
      frame.can_dlc = 8;
      if (::write(socket, &frame, sizeof(frame)) != sizeof(frame)) {
        ::close(socket);
        throw std::runtime_error("failed to send CAN command");
      }
    }
    ::close(socket);
  }

  std::string interface_;
  std::vector<int64_t> nodes_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
};
}  // namespace sandro_hardware

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sandro_hardware::EstopNode>());
  rclcpp::shutdown();
  return 0;
}
