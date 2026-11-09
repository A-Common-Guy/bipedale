#pragma once

// Diagnostics filled from the control loop without allocating: each key is
// written only when it differs from the one already in that slot, and each
// value is formatted into a stack buffer and copied into the existing string,
// which keeps its capacity across messages. After the first message nothing
// is allocated as long as the set of keys stays the same.

#include <cstdio>
#include <string>
#include <string_view>

namespace sandro_controllers {

// Status: a diagnostic_msgs/DiagnosticStatus (or anything with a `values`
// vector of elements carrying std::string `key` and `value`).
template <class Status> class DiagnosticValues {
public:
  explicit DiagnosticValues(Status &status) : status_(status) {}
  DiagnosticValues(const DiagnosticValues &) = delete;
  DiagnosticValues &operator=(const DiagnosticValues &) = delete;

  // Drops the slots of a longer previous message; call before publishing.
  void finish() { status_.values.resize(count_); }

  void add(std::string_view key, const double value) { put(key, "%.6g", value); }
  void add(std::string_view key, const float value) { put(key, "%.6g", static_cast<double>(value)); }
  void add(std::string_view key, const int value) { put(key, "%d", value); }
  void add(std::string_view key, const unsigned value) { put(key, "%u", value); }
  void add(std::string_view key, const long value) { put(key, "%ld", value); }
  void add(std::string_view key, const bool value) { put(key, "%d", value ? 1 : 0); }

private:
  template <class T> void put(std::string_view key, const char *format, const T value) {
    if (status_.values.size() <= count_) {
      status_.values.resize(count_ + 1);
    }
    auto &item = status_.values[count_++];
    if (item.key != key) {
      item.key.assign(key.data(), key.size());
    }
    char text[32];
    const int length = std::snprintf(text, sizeof(text), format, value);
    item.value.assign(text, length > 0 ? static_cast<std::size_t>(length) : 0U);
  }

  Status &status_;
  std::size_t count_{};
};

// Publishing at a fixed rate from a fixed-rate loop: the next slot advances by
// whole periods, so a 50 Hz rate on a 300 Hz loop stays 50 Hz on average
// instead of rounding every interval up to the next cycle.
class RateGate {
public:
  void reset() { next_s_ = 0.0; }
  // True when a message is due at now_s; call commit() once it was published.
  bool due(const double now_s) const { return now_s >= next_s_; }
  void commit(const double now_s, const double rate_hz) {
    next_s_ += 1.0 / rate_hz;
    if (next_s_ <= now_s) {
      next_s_ = now_s + 1.0 / rate_hz;
    }
  }

private:
  double next_s_{};
};

} // namespace sandro_controllers
