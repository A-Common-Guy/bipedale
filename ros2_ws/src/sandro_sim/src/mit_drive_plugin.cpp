// mujoco_ros2_control plugin that runs the GIM6010 MIT drive law on every
// physics step. LatencySystem hands it the command frames through drive_bus();
// before each mj_step it overwrites the joint's actuator control with
//   kp (q_ref - q) + kd (v_ref - v) + effort
// computed from the live joint state, as the drive computes it in its own
// loop from its own encoder. Without this plugin LatencySystem computes the
// same law once per control cycle from the cycle's state snapshot and the
// backend holds that torque until the next cycle: a delayed, sampled loop the
// physical drive does not have.

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>
#include <mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "sandro_sim/drive_bus.hpp"
#include "sandro_sim/mit_drive.hpp"

namespace sandro_sim {

class MitDrivePlugin final
    : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase {
public:
  bool init(rclcpp::Node::SharedPtr node, const mjModel *model,
            mjData * /*data*/) override {
    node_ = std::move(node);
    model_ = model;
    drive_bus().attach();
    RCLCPP_INFO(node_->get_logger(),
                "MIT drive law runs on every physics step (timestep %.3f ms)",
                1e3 * model_->opt.timestep);
    return true;
  }

  void pre_step(mjData *data) override {
    auto &bus = drive_bus();
    const auto generation = bus.generation();
    if (generation != generation_) {
      resolve(bus);
      generation_ = generation;
    }
    bus.due(data->time, frames_);
    for (std::size_t i = 0; i < joints_.size() && i < frames_.size(); ++i) {
      const auto &joint = joints_[i];
      if (!joint || !frames_[i]) {
        continue;
      }
      data->ctrl[joint->actuator] =
          mit_torque(*frames_[i], data->qpos[joint->qpos], data->qvel[joint->dof],
                     torque_limit_nm_);
    }
  }

  void on_reset(mjData * /*data*/) override { drive_bus().clear(); }

  void cleanup() override { node_.reset(); }

private:
  struct Joint {
    int qpos{};
    int dof{};
    int actuator{};
  };

  // Map the bus's joint names onto the model: the hinge's state addresses and
  // the actuator that transmits to it. A name the model lacks is left to
  // LatencySystem's per-cycle torque.
  void resolve(const DriveBus &bus) {
    joints_.clear();
    torque_limit_nm_ = bus.torque_limit_nm();
    for (const auto &name : bus.joints()) {
      std::optional<Joint> resolved;
      const int joint = mj_name2id(model_, mjOBJ_JOINT, name.c_str());
      if (joint >= 0 && model_->jnt_type[joint] == mjJNT_HINGE) {
        for (int actuator = 0; actuator < model_->nu; ++actuator) {
          if (model_->actuator_trntype[actuator] == mjTRN_JOINT &&
              model_->actuator_trnid[2 * actuator] == joint) {
            resolved = Joint{model_->jnt_qposadr[joint], model_->jnt_dofadr[joint], actuator};
            break;
          }
        }
      }
      if (!resolved) {
        RCLCPP_ERROR(node_->get_logger(),
                     "joint '%s' has no hinge with a joint actuator in the model; "
                     "its drive law stays once per control cycle", name.c_str());
      }
      joints_.push_back(resolved);
    }
  }

  rclcpp::Node::SharedPtr node_;
  const mjModel *model_{nullptr};
  std::uint64_t generation_{0};
  double torque_limit_nm_{};
  std::vector<std::optional<Joint>> joints_;
  std::vector<std::optional<MitCommand>> frames_;
};

} // namespace sandro_sim

PLUGINLIB_EXPORT_CLASS(sandro_sim::MitDrivePlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
