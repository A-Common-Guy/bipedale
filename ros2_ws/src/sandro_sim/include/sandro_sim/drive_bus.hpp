#pragma once

// Hand-off of MIT command frames from the control thread to the physics
// thread. LatencySystem::write() pushes each joint's frame stamped with the
// controller time it was written at; MitDrivePlugin::pre_step() asks, before
// every MuJoCo step, which frame each drive is executing at that simulation
// time and computes the drive law from the live joint state, the way a
// GIM6010 runs it in its own loop from its own encoder. A frame becomes due
// command_delay_s after its stamp (the CAN and USB transport), so the command
// latency is resolved at the physics step, not at the control cycle.
//
// Header-only and free of ROS and MuJoCo so it can be unit-tested. The
// process-wide instance shared by both plugins is drive_bus().

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "sandro_sim/mit_drive.hpp"

namespace sandro_sim {

class DriveBus {
public:
  // Frames waiting for their delay; at 300 Hz and a few milliseconds of delay
  // only a handful are ever pending.
  static constexpr std::size_t kQueueCapacity = 64;

  void configure(std::vector<std::string> joints, const double command_delay_s,
                 const double torque_limit_nm) {
    const std::lock_guard<std::mutex> lock(mutex_);
    names_ = std::move(joints);
    joints_.assign(names_.size(), Joint{});
    command_delay_s_ = command_delay_s > 0.0 ? command_delay_s : 0.0;
    torque_limit_nm_ = torque_limit_nm;
    ++generation_;
  }

  // Bumped by every configure(), so a consumer knows to re-resolve joints.
  std::uint64_t generation() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return generation_;
  }

  std::vector<std::string> joints() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return names_;
  }

  double torque_limit_nm() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return torque_limit_nm_;
  }

  void push(const std::size_t joint, const double stamp_s, const MitCommand &frame) {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (joint >= joints_.size()) {
      return;
    }
    auto &pending = joints_[joint].pending;
    if (pending.size() >= kQueueCapacity) {
      pending.pop_front();
    }
    pending.emplace_back(stamp_s, frame);
  }

  // The frame every drive executes at now_s: the newest whose delay has
  // elapsed, held until a newer one is due. Empty for a drive that has not
  // received one yet.
  void due(const double now_s, std::vector<std::optional<MitCommand>> &frames) {
    const std::lock_guard<std::mutex> lock(mutex_);
    frames.resize(joints_.size());
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      auto &joint = joints_[i];
      while (!joint.pending.empty() &&
             joint.pending.front().first + command_delay_s_ <= now_s + 1e-9) {
        joint.current = joint.pending.front().second;
        joint.pending.pop_front();
      }
      frames[i] = joint.current;
    }
  }

  // Forget every frame (activation, deactivation, world reset): the drives
  // then execute nothing until the controller writes again.
  void clear() {
    const std::lock_guard<std::mutex> lock(mutex_);
    for (auto &joint : joints_) {
      joint = Joint{};
    }
  }

  // Set by the physics-side plugin when it is loaded, so the control side can
  // report which drive emulation is in effect.
  void attach() {
    const std::lock_guard<std::mutex> lock(mutex_);
    attached_ = true;
  }

  bool attached() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return attached_;
  }

private:
  struct Joint {
    std::deque<std::pair<double, MitCommand>> pending;
    std::optional<MitCommand> current;
  };

  mutable std::mutex mutex_;
  std::vector<std::string> names_;
  std::vector<Joint> joints_;
  double command_delay_s_{};
  double torque_limit_nm_{};
  std::uint64_t generation_{};
  bool attached_{false};
};

DriveBus &drive_bus();

} // namespace sandro_sim
