// Closed-loop simulation of the whole-body controller without ROS.
//
// Runs the same pieces as `sandro.launch.py hardware:=mujoco
// controller:=whole_body_controller` in one deterministic process:
//   - MuJoCo on scene_ros2.xml, reset to the stand keyframe;
//   - the controller manager's wake-up rule on the simulation clock (first
//     physics step at or past last wake + 1/update_rate);
//   - mujoco_ros2_control's sensor read (raw qpos/qvel, IMU sensors with the
//     MJCF noise added per read);
//   - sandro_sim's LatencySystem (DelayLine per state interface, DriveBus for
//     the MIT frames, the per-cycle fallback torque) and MitDrivePlugin (the
//     MIT law before every physics step);
//   - sandro_controllers' WholeBodyCore, unchanged.
// The output CSV has drive_test.py's layout, so compare_runs.py compares it
// with hardware recordings directly. Driven by ../closed_loop.py.

#include <mujoco/mujoco.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "sandro_controllers/whole_body_core.hpp"
#include "sandro_controllers/whole_body_parameters.hpp"
#include "sandro_sim/delay_line.hpp"
#include "sandro_sim/drive_bus.hpp"
#include "sandro_sim/mit_drive.hpp"

namespace {

using sandro_controllers::WholeBodyCommand;
using sandro_controllers::WholeBodyCore;
using sandro_controllers::WholeBodyOutput;
using sandro_controllers::WholeBodyParameters;
using sandro_controllers::WholeBodySettings;
using sandro_controllers::WholeBodyState;
using sandro_sim::DelayLine;
using sandro_sim::DriveBus;
using sandro_sim::MitCommand;

// ros2_control description order (sandro.ros2_control.xacro), which is also
// the controller's joint order.
constexpr std::array<const char *, 6> kJoints{
    "right_motor_inner", "right_motor_outer", "right_wheel",
    "left_motor_inner",  "left_motor_outer",  "left_wheel"};

struct Phase {
  double duration_s{};
  double forward_m_s{};
  double yaw_rad_s{};
  double height_rate_m_s{};
};

struct Push {
  double start_s{};
  double force_x_n{};
  double duration_s{};
};

struct Options {
  std::string model;
  std::string output;
  std::vector<std::pair<std::string, double>> parameters;
  std::vector<Phase> phases;
  std::vector<Push> pushes;
  int update_rate_hz{300};
  double timestep_s{0.0};  // 0: the model's
  double imu_delay_s{0.010};
  double imu_rate_hz{200.0};
  double joint_delay_s{0.001};
  double joint_rate_hz{200.0};
  bool joint_stagger{true};
  double command_delay_s{0.001};
  double mit_torque_limit_nm{10.0};
  bool drive_plugin{true};
  bool imu_noise{true};
  double unpowered_s{0.03};
  double settle_s{3.0};
  double engage_timeout_s{10.0};
  double command_rate_hz{20.0};
  double diagnostics_rate_hz{50.0};
  unsigned seed{1};
};

[[noreturn]] void fail(const std::string &message) {
  std::cerr << "closed_loop: " << message << "\n";
  std::exit(2);
}

std::vector<double> numbers(const std::string &text) {
  std::vector<double> result;
  std::stringstream stream(text);
  std::string item;
  while (std::getline(stream, item, ',')) {
    result.push_back(std::stod(item));
  }
  return result;
}

bool truthy(const std::string &text) {
  if (text == "true" || text == "1") return true;
  if (text == "false" || text == "0") return false;
  fail("expected true/false, got '" + text + "'");
}

Options parse(int argc, char **argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (i + 1 >= argc) fail("missing value for " + key);
    const std::string value = argv[++i];
    if (key == "--model") options.model = value;
    else if (key == "--output") options.output = value;
    else if (key == "--param") {
      const auto eq = value.find('=');
      if (eq == std::string::npos) fail("--param needs name=value");
      options.parameters.emplace_back(value.substr(0, eq), std::stod(value.substr(eq + 1)));
    } else if (key == "--phase") {
      auto v = numbers(value);
      if (v.size() == 3) v.push_back(0.0);
      if (v.size() != 4) fail("--phase needs duration,forward,yaw[,height_rate]");
      options.phases.push_back({v[0], v[1], v[2], v[3]});
    } else if (key == "--push") {
      const auto v = numbers(value);
      if (v.size() != 3) fail("--push needs start_s,force_x_n,duration_s (phase-0 relative)");
      options.pushes.push_back({v[0], v[1], v[2]});
    } else if (key == "--update-rate") options.update_rate_hz = std::stoi(value);
    else if (key == "--timestep") options.timestep_s = std::stod(value);
    else if (key == "--imu-delay") options.imu_delay_s = std::stod(value);
    else if (key == "--imu-rate") options.imu_rate_hz = std::stod(value);
    else if (key == "--joint-delay") options.joint_delay_s = std::stod(value);
    else if (key == "--joint-rate") options.joint_rate_hz = std::stod(value);
    else if (key == "--joint-stagger") options.joint_stagger = truthy(value);
    else if (key == "--command-delay") options.command_delay_s = std::stod(value);
    else if (key == "--drive-plugin") options.drive_plugin = truthy(value);
    else if (key == "--imu-noise") options.imu_noise = truthy(value);
    else if (key == "--unpowered") options.unpowered_s = std::stod(value);
    else if (key == "--settle") options.settle_s = std::stod(value);
    else if (key == "--seed") options.seed = static_cast<unsigned>(std::stoul(value));
    else fail("unknown option " + key);
  }
  if (options.model.empty() || options.output.empty()) fail("--model and --output are required");
  if (options.phases.empty()) fail("at least one --phase is required");
  return options;
}

struct JointAddress {
  int qpos{};
  int dof{};
  int actuator{};
};

class Simulation {
public:
  explicit Simulation(const Options &options) : options_(options), rng_(options.seed) {
    char error[1000] = "";
    model_ = mj_loadXML(options.model.c_str(), nullptr, error, sizeof(error));
    if (!model_) fail(std::string("cannot load model: ") + error);
    if (options.timestep_s > 0.0) model_->opt.timestep = options.timestep_s;
    data_ = mj_makeData(model_);
    const int key = mj_name2id(model_, mjOBJ_KEY, "stand");
    if (key < 0) fail("model has no 'stand' keyframe");
    mj_resetDataKeyframe(model_, data_, key);
    mj_forward(model_, data_);
    for (std::size_t i = 0; i < kJoints.size(); ++i) {
      const int joint = mj_name2id(model_, mjOBJ_JOINT, kJoints[i]);
      if (joint < 0) fail(std::string("joint not in model: ") + kJoints[i]);
      int actuator = -1;
      for (int a = 0; a < model_->nu; ++a) {
        if (model_->actuator_trntype[a] == mjTRN_JOINT && model_->actuator_trnid[2 * a] == joint) {
          actuator = a;
        }
      }
      if (actuator < 0) fail(std::string("no actuator for ") + kJoints[i]);
      joints_[i] = {model_->jnt_qposadr[joint], model_->jnt_dofadr[joint], actuator};
    }
    for (const auto *suffix : {"imu_quat", "imu_gyro", "imu_accel"}) {
      const int sensor = mj_name2id(model_, mjOBJ_SENSOR, suffix);
      if (sensor < 0) fail(std::string("sensor not in model: ") + suffix);
      sensors_.push_back(sensor);
    }
    torso_ = mj_name2id(model_, mjOBJ_BODY, "torso");
    imu_site_ = mj_name2id(model_, mjOBJ_SITE, "imu");
    if (torso_ < 0 || imu_site_ < 0) fail("model needs the 'torso' body and the 'imu' site");
    staged_ctrl_.assign(static_cast<std::size_t>(model_->nu), 0.0);

    // LatencySystem: one channel per state interface. Joint channels share
    // their joint's broadcast phase; the IMU channels share one clock.
    const double joint_period = options.joint_rate_hz > 0.0 ? 1.0 / options.joint_rate_hz : 0.0;
    for (std::size_t i = 0; i < kJoints.size(); ++i) {
      const double phase = options.joint_stagger
                               ? joint_period * static_cast<double>(i) / static_cast<double>(kJoints.size())
                               : 0.0;
      for (auto &line : joint_lines_[i]) {
        line.configure(options.joint_delay_s, options.joint_rate_hz, phase, true);
      }
    }
    for (auto &line : imu_lines_) {
      line.configure(options.imu_delay_s, options.imu_rate_hz, 0.0, true);
    }
    for (auto &channel : fallback_lines_) {
      for (auto &line : channel) {
        line.configure(options.command_delay_s, 0.0);
      }
    }
    std::vector<std::string> names(kJoints.begin(), kJoints.end());
    bus_.configure(names, options.command_delay_s, options.mit_torque_limit_nm);
  }

  ~Simulation() {
    mj_deleteData(data_);
    mj_deleteModel(model_);
  }

  double time() const { return data_->time; }
  double timestep() const { return model_->opt.timestep; }

  // One physics step as mujoco_ros2_control's physics thread runs it: staged
  // controls in, plugin pre_step, external push, mj_step.
  void step(const double push_force_x_n) {
    for (int a = 0; a < model_->nu; ++a) {
      data_->ctrl[a] = staged_ctrl_[static_cast<std::size_t>(a)];
    }
    if (options_.drive_plugin) {
      bus_.due(data_->time, frames_);
      for (std::size_t i = 0; i < joints_.size(); ++i) {
        if (frames_[i]) {
          data_->ctrl[joints_[i].actuator] = sandro_sim::mit_torque(
              *frames_[i], data_->qpos[joints_[i].qpos], data_->qvel[joints_[i].dof],
              options_.mit_torque_limit_nm);
        }
      }
    }
    if (torso_ >= 0) {
      mjtNum *wrench = data_->xfrc_applied + 6 * torso_;
      std::fill(wrench, wrench + 6, 0.0);
      if (push_force_x_n != 0.0) {
        // Along the torso's forward axis projected on the ground.
        const mjtNum *r = data_->xmat + 9 * torso_;
        const double fx = r[0], fy = r[3];
        const double norm = std::hypot(fx, fy);
        if (norm > 1e-6) {
          wrench[0] = push_force_x_n * fx / norm;
          wrench[1] = push_force_x_n * fy / norm;
        }
      }
    }
    mj_step(model_, data_);
  }

  // mujoco_ros2_control read() plus LatencySystem::read().
  WholeBodyState read(const double now, std::array<double, 10> &imu_out,
                      std::array<double, 6> &effort_out) {
    WholeBodyState state;
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      state.position[i] = joint_lines_[i][0].update(now, data_->qpos[joints_[i].qpos]);
      state.velocity[i] = joint_lines_[i][1].update(now, data_->qvel[joints_[i].dof]);
      effort_out[i] = joint_lines_[i][2].update(now, data_->actuator_force[joints_[i].actuator]);
    }
    std::array<double, 10> raw{};
    const mjtNum *quat = data_->sensordata + model_->sensor_adr[sensors_[0]];
    const mjtNum *gyro = data_->sensordata + model_->sensor_adr[sensors_[1]];
    const mjtNum *accel = data_->sensordata + model_->sensor_adr[sensors_[2]];
    raw = {quat[1], quat[2], quat[3], quat[0], gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]};
    if (options_.imu_noise) {
      const std::array<int, 10> sensor_of{0, 0, 0, 0, 1, 1, 1, 2, 2, 2};
      for (std::size_t i = 0; i < raw.size(); ++i) {
        const double sigma = model_->sensor_noise[sensors_[static_cast<std::size_t>(sensor_of[i])]];
        if (sigma > 0.0) {
          raw[i] += std::normal_distribution<double>(0.0, sigma)(rng_);
        }
      }
    }
    for (std::size_t i = 0; i < raw.size(); ++i) {
      imu_out[i] = imu_lines_[i].update(now, raw[i]);
    }
    state.quaternion_x = imu_out[0];
    state.quaternion_y = imu_out[1];
    state.quaternion_z = imu_out[2];
    state.quaternion_w = imu_out[3];
    state.angular_velocity_rad_s = {imu_out[4], imu_out[5], imu_out[6]};
    state.imu_age_s = 0.0;
    return state;
  }

  // LatencySystem::write(): the frame onto the bus, and the per-cycle torque
  // the backend holds when MitDrivePlugin is absent.
  void write(const double now, const WholeBodyOutput &output) {
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      const std::array<double, 5> raw{output.position_command_rad[i], output.velocity_command_rad_s[i],
                                      output.kp_nm_rad[i], output.kd_nm_s_rad[i], output.effort_nm[i]};
      std::array<double, 5> delayed{};
      for (std::size_t f = 0; f < raw.size(); ++f) {
        delayed[f] = fallback_lines_[i][f].update(now, raw[f]);
      }
      bus_.push(i, now, MitCommand{raw[0], raw[1], raw[2], raw[3], raw[4]});
      staged_ctrl_[static_cast<std::size_t>(joints_[i].actuator)] = sandro_sim::mit_torque(
          MitCommand{delayed[0], delayed[1], delayed[2], delayed[3], delayed[4]},
          data_->qpos[joints_[i].qpos], data_->qvel[joints_[i].dof], options_.mit_torque_limit_nm);
    }
  }

  void unpower() {
    std::fill(staged_ctrl_.begin(), staged_ctrl_.end(), 0.0);
    bus_.clear();
  }

  // Ground-truth body pitch and forward speed for the CSV (not seen by the
  // controller).
  void truth(double &pitch, double &speed) const {
    // Noise-free, undelayed pitch of the IMU frame (x forward, z up).
    const mjtNum *r = data_->site_xmat + 9 * imu_site_;
    pitch = std::atan2(-r[6], std::hypot(r[0], r[3]));
    speed = std::hypot(data_->qvel[0], data_->qvel[1]);
  }

private:
  const Options &options_;
  mjModel *model_{nullptr};
  mjData *data_{nullptr};
  std::array<JointAddress, 6> joints_{};
  std::vector<int> sensors_;
  int torso_{-1};
  int imu_site_{-1};
  std::vector<double> staged_ctrl_;
  std::array<std::array<DelayLine, 3>, 6> joint_lines_{};
  std::array<DelayLine, 10> imu_lines_{};
  std::array<std::array<DelayLine, 5>, 6> fallback_lines_{};
  DriveBus bus_;
  std::vector<std::optional<MitCommand>> frames_;
  std::mt19937 rng_;
};

std::string format(const double value) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.6g", value);
  return buffer;
}

} // namespace

int main(int argc, char **argv) {
  const Options options = parse(argc, argv);

  WholeBodyParameters parameters;
  for (const auto &[name, value] : options.parameters) {
    if (!sandro_controllers::set_whole_body_parameter(parameters, name, value)) {
      fail("unknown whole-body parameter '" + name + "'");
    }
  }
  const double period_s = 1.0 / static_cast<double>(options.update_rate_hz);
  WholeBodySettings settings;
  std::string reason;
  if (!sandro_controllers::build_whole_body_settings(parameters, period_s, settings, &reason)) {
    fail("invalid whole-body parameters: " + reason);
  }
  const auto gains = settings.gains.at(parameters.height_target_m);
  std::printf("LQR at %.3f m: wheel [x %.2f v %.2f pitch %.2f rate %.2f dcmd %.2f d %.2f] "
              "leg [x %.2f v %.2f pitch %.2f rate %.2f dcmd %.2f d %.2f]; delay margin %.1f ms\n",
              parameters.height_target_m, gains[0][0], gains[0][1], gains[0][2], gains[0][3],
              gains[0][4], gains[0][5], gains[1][0], gains[1][1], gains[1][2], gains[1][3],
              gains[1][4], gains[1][5], 1e3 * settings.delay_margin_s);

  Simulation sim(options);
  std::ofstream csv(options.output);
  if (!csv) fail("cannot write " + options.output);
  const std::vector<std::string> diag_keys{
      "engaged", "status_mask", "pitch_rad", "pitch_reference_rad", "pitch_rate_rad_s",
      "roll_rad", "roll_reference_rad", "forward_velocity_m_s", "velocity_reference_m_s",
      "position_error_m", "yaw_rate_rad_s", "wheel_yaw_rate_rad_s", "yaw_rate_reference_rad_s",
      "leg_height_m", "height_reference_m", "height_differential_m", "height_support_n",
      "leg_offset_m", "leg_offset_reference_m", "leg_offset_command_m", "leg_offset_model_m",
      "leg_offset_state_m", "leg_offset_rate_m_s", "trim_m", "balance_torque_nm",
      "yaw_torque_nm", "yaw_feedforward_nm", "truth_pitch_rad", "truth_speed_m_s"};
  csv << "t_s,stream,phase,cmd_forward,cmd_yaw,cmd_height_rate";
  for (const auto &key : diag_keys) csv << ",diag_" << key;
  for (const auto *prefix : {"pos_", "vel_", "eff_"}) {
    for (const auto *joint : kJoints) csv << "," << prefix << joint;
  }
  csv << ",imu_qx,imu_qy,imu_qz,imu_qw,imu_gx,imu_gy,imu_gz,imu_ax,imu_ay,imu_az\n";

  // Physics runs unpowered on the keyframe while sim_reset re-activates the
  // controller.
  const auto step_ns = [&] { return static_cast<std::int64_t>(sim.time() * 1e9); };
  while (sim.time() < options.unpowered_s - 1e-12) {
    sim.step(0.0);
  }

  WholeBodyCore core;
  const std::int64_t period_ns = static_cast<std::int64_t>(1e9 / options.update_rate_hz);
  std::int64_t last_ns = step_ns();
  std::int64_t wake_ns = last_ns;
  bool activated = false;
  double recording_start = -1.0;
  double phase_start = -1.0;
  double engaged_at = -1.0;
  double last_diag = -1.0;
  double last_command_publish = -1.0;
  double command_receipt = -1.0;
  Phase published{};
  std::size_t phase_index = 0;
  double fall_time = -1.0;
  bool finished = false;
  WholeBodyCommand received{};
  const double total_phase_s = [&] {
    double sum = 0.0;
    for (const auto &p : options.phases) sum += p.duration_s;
    return sum;
  }();

  while (!finished) {
    // Push force for the coming step, phase-0 relative.
    double push = 0.0;
    if (phase_start >= 0.0) {
      for (const auto &p : options.pushes) {
        const double t = sim.time() - phase_start;
        if (t >= p.start_s && t < p.start_s + p.duration_s) push += p.force_x_n;
      }
    }
    sim.step(push);
    const std::int64_t now_ns = step_ns();
    if (now_ns < wake_ns) continue;
    const double now = static_cast<double>(now_ns) * 1e-9;
    const double measured_period = static_cast<double>(now_ns - last_ns) * 1e-9;
    last_ns = now_ns;
    wake_ns = now_ns + period_ns;

    std::array<double, 10> imu{};
    std::array<double, 6> effort{};
    const WholeBodyState state = sim.read(now, imu, effort);
    if (!activated) {
      core.reset(state, settings);
      activated = true;
      continue;  // on_activate writes zero commands
    }

    // drive_test.py: wait for engagement, settle, then the phases at 20 Hz.
    const double command_period = 1.0 / options.command_rate_hz;
    if (last_command_publish < 0.0 || now - last_command_publish >= command_period - 1e-9) {
      last_command_publish = now;
      if (phase_start >= 0.0) {
        double elapsed = now - phase_start;
        std::size_t index = 0;
        while (index < options.phases.size() && elapsed >= options.phases[index].duration_s) {
          elapsed -= options.phases[index].duration_s;
          ++index;
        }
        phase_index = index;
        published = index < options.phases.size() ? options.phases[index] : Phase{};
      } else {
        published = Phase{};
      }
      if (engaged_at >= 0.0) {
        received.forward_velocity_m_s = published.forward_m_s;
        received.yaw_rate_rad_s = published.yaw_rad_s;
        received.height_rate_m_s = published.height_rate_m_s;
        command_receipt = now;
      }
    }
    WholeBodyCommand command = received;
    command.age_s = command_receipt < 0.0 ? std::numeric_limits<double>::infinity()
                                          : std::max(0.0, now - command_receipt);
    const WholeBodyOutput output = core.update(state, command, settings, measured_period);
    sim.write(now, output);

    if (output.engaged && engaged_at < 0.0) {
      engaged_at = now;
      recording_start = now;
    }
    if (engaged_at >= 0.0 && phase_start < 0.0 && now - engaged_at >= options.settle_s) {
      phase_start = now;
    }
    if (engaged_at < 0.0 && now > options.engage_timeout_s) {
      std::printf("never engaged within %.0f s (status %u)\n", options.engage_timeout_s, output.status);
      return 1;
    }
    if (output.fallen && fall_time < 0.0) {
      fall_time = now;
    }
    if (phase_start >= 0.0 && now - phase_start >= total_phase_s) finished = true;
    if (fall_time >= 0.0 && now - fall_time > 0.5) finished = true;

    if (recording_start < 0.0) continue;
    const int phase = phase_start < 0.0 ? -1 : static_cast<int>(phase_index);
    const std::string base = format(now - recording_start) + "," + "%s," + std::to_string(phase) + "," +
                             format(published.forward_m_s) + "," + format(published.yaw_rad_s) + "," +
                             format(published.height_rate_m_s);
    const auto row_start = [&](const char *stream) {
      std::string text = base;
      text.replace(text.find("%s"), 2, stream);
      csv << text;
    };
    const std::size_t blanks_diag = diag_keys.size();
    if (last_diag < 0.0 || now - last_diag >= 1.0 / options.diagnostics_rate_hz - 1e-9) {
      last_diag = now;
      double truth_pitch = 0.0, truth_speed = 0.0;
      sim.truth(truth_pitch, truth_speed);
      const std::vector<double> values{
          output.engaged ? 1.0 : 0.0, static_cast<double>(output.status), output.pitch_rad,
          output.pitch_reference_rad, output.pitch_rate_rad_s, output.roll_rad,
          output.roll_reference_rad, output.forward_velocity_m_s, output.velocity_reference_m_s,
          output.position_m - output.position_reference_m, output.yaw_rate_rad_s,
          output.wheel_yaw_rate_rad_s, output.yaw_rate_reference_rad_s, output.leg_height_m,
          output.height_reference_m, output.height_differential_m, output.height_support_n,
          output.leg_offset_m, output.leg_offset_reference_m, output.leg_offset_command_m,
          output.leg_offset_model_m, output.leg_offset_state_m, output.leg_offset_rate_m_s,
          output.trim_m, output.balance_torque_nm, output.yaw_torque_nm, output.yaw_feedforward_nm,
          truth_pitch, truth_speed};
      row_start("diag");
      for (const double v : values) csv << "," << format(v);
      csv << std::string(18 + 10, ',') << "\n";
    }
    row_start("joint");
    csv << std::string(blanks_diag, ',');
    for (std::size_t i = 0; i < 6; ++i) csv << "," << format(state.position[i]);
    for (std::size_t i = 0; i < 6; ++i) csv << "," << format(state.velocity[i]);
    for (std::size_t i = 0; i < 6; ++i) csv << "," << format(effort[i]);
    csv << std::string(10, ',') << "\n";
    row_start("imu");
    csv << std::string(blanks_diag + 18, ',');
    for (const double v : imu) csv << "," << format(v);
    csv << "\n";
  }
  const bool fell = fall_time >= 0.0;
  std::printf("timestep %.4f ms, controller %d Hz; %s\n", 1e3 * sim.timestep(),
              options.update_rate_hz,
              fell ? "FELL" : "completed all phases");
  return fell ? 3 : 0;
}
