#pragma once

// Sample-and-hold delay line for one scalar channel. Header-only and free of
// ROS so the timing model can be unit-tested.
//
// A real sensor samples on its own clock, every 1/rate seconds at
// phase + k / rate, and its value reaches the consumer `delay` seconds after
// it was sampled; between samples the consumer keeps the last one. This is
// what the BNO085 reports and the drives' CAN encoder broadcasts look like to
// a 300 Hz controller. The sample grid is fixed: a refresh is not "the first
// update at least one period after the previous one", which at 200 Hz against
// a 300 Hz consumer would refresh every second update (150 Hz).
//
// The channel only sees the value at the instants update() is called, so the
// value at a sample instant between two updates is reconstructed: linearly for
// a physical signal (interpolate = true), or as the newest value at or before
// it for a piecewise-constant one such as a command (zero-order hold).

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>

namespace sandro_sim {

class DelayLine {
public:
  static constexpr std::size_t kCapacity = 256;

  // delay_s <= 0 and rate_hz <= 0 mean pass-through. phase_s places the
  // sample grid; only its value modulo the period matters.
  void configure(const double delay_s, const double rate_hz,
                 const double phase_s = 0.0, const bool interpolate = false) {
    delay_s_ = std::isfinite(delay_s) && delay_s > 0.0 ? delay_s : 0.0;
    period_s_ = std::isfinite(rate_hz) && rate_hz > 0.0 ? 1.0 / rate_hz : 0.0;
    phase_s_ = std::isfinite(phase_s) ? phase_s : 0.0;
    interpolate_ = interpolate;
    reset();
  }

  void reset() {
    count_ = 0;
    head_ = 0;
    output_.reset();
    last_sample_index_.reset();
  }

  bool passthrough() const { return delay_s_ == 0.0 && period_s_ == 0.0; }

  // Push the value that is true now and return what the consumer sees now.
  double update(const double now_s, const double value) {
    if (passthrough()) {
      return value;
    }
    samples_[head_] = Sample{now_s, value};
    head_ = (head_ + 1) % kCapacity;
    if (count_ < kCapacity) {
      ++count_;
    }
    double sampled_at = now_s - delay_s_;
    std::optional<long long> index;
    if (period_s_ > 0.0) {
      // Newest grid instant whose sample has arrived by now.
      index = static_cast<long long>(
          std::floor((now_s - delay_s_ - phase_s_) / period_s_ + 1e-9));
      if (output_ && last_sample_index_ == index) {
        return *output_;
      }
      sampled_at = phase_s_ + static_cast<double>(*index) * period_s_;
    }
    const std::optional<double> delayed = value_at(sampled_at);
    if (delayed) {
      output_ = *delayed;
      last_sample_index_ = index;
    } else if (!output_) {
      // Nothing old enough yet: hold the first sample rather than emit zero.
      output_ = value;
    }
    return *output_;
  }

  std::optional<double> current() const { return output_; }

private:
  struct Sample {
    double time_s{};
    double value{};
  };

  const Sample &sample(const std::size_t age_order) const {
    // age_order 0 is the oldest stored sample.
    return samples_[(head_ + kCapacity - count_ + age_order) % kCapacity];
  }

  // The channel's value at time_s, if the history reaches back that far.
  std::optional<double> value_at(const double time_s) const {
    std::optional<std::size_t> before;
    for (std::size_t i = 0; i < count_; ++i) {
      if (sample(i).time_s <= time_s) {
        before = i;
      } else {
        break;
      }
    }
    if (!before) {
      return std::nullopt;
    }
    const Sample &a = sample(*before);
    if (!interpolate_ || *before + 1 >= count_) {
      return a.value;
    }
    const Sample &b = sample(*before + 1);
    const double span = b.time_s - a.time_s;
    if (!(span > 0.0)) {
      return a.value;
    }
    return a.value + (b.value - a.value) * (time_s - a.time_s) / span;
  }

  std::array<Sample, kCapacity> samples_{};
  std::size_t count_{};
  std::size_t head_{};
  std::optional<double> output_;
  std::optional<long long> last_sample_index_;
  double delay_s_{};
  double period_s_{};
  double phase_s_{};
  bool interpolate_{false};
};

} // namespace sandro_sim
