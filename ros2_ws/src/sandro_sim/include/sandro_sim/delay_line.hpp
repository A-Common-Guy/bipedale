#pragma once

// Sample-and-hold delay line for one scalar channel. Header-only and free of
// ROS so the timing model can be unit-tested.
//
// A real sensor path delivers a value that was true `delay` seconds ago and
// only refreshes it every 1/rate seconds; between refreshes the last value is
// held. This is what the BNO085 (200 Hz reports plus fusion latency) and the
// drives' 200 Hz CAN encoder broadcasts look like to a 300 Hz controller, and
// what a 300 Hz controller looks like to the drives on the command side.

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>

namespace sandro_sim {

class DelayLine {
public:
  static constexpr std::size_t kCapacity = 256;

  // delay_s <= 0 and rate_hz <= 0 mean pass-through.
  void configure(const double delay_s, const double rate_hz) {
    delay_s_ = std::isfinite(delay_s) && delay_s > 0.0 ? delay_s : 0.0;
    period_s_ = std::isfinite(rate_hz) && rate_hz > 0.0 ? 1.0 / rate_hz : 0.0;
    reset();
  }

  void reset() {
    count_ = 0;
    head_ = 0;
    output_.reset();
    last_refresh_s_ = -std::numeric_limits<double>::infinity();
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
    if (period_s_ > 0.0 && now_s - last_refresh_s_ < period_s_ - 1e-9) {
      return output_.value_or(value);
    }
    const std::optional<double> delayed = delayed_value(now_s - delay_s_);
    if (delayed) {
      output_ = *delayed;
      last_refresh_s_ = now_s;
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

  // Newest sample stamped at or before `time_s`, if any.
  std::optional<double> delayed_value(const double time_s) const {
    std::optional<double> result;
    for (std::size_t i = 0; i < count_; ++i) {
      // Walk from oldest to newest.
      const std::size_t index = (head_ + kCapacity - count_ + i) % kCapacity;
      if (samples_[index].time_s <= time_s) {
        result = samples_[index].value;
      } else {
        break;
      }
    }
    return result;
  }

  std::array<Sample, kCapacity> samples_{};
  std::size_t count_{};
  std::size_t head_{};
  std::optional<double> output_;
  double last_refresh_s_{-std::numeric_limits<double>::infinity()};
  double delay_s_{};
  double period_s_{};
};

} // namespace sandro_sim
