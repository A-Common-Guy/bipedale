#pragma once

#include <algorithm>
#include <cmath>

namespace sandro_hardware
{

// Thermal envelope of one drive: the peak torque in bursts, folding back to
// the rated (continuous) torque as the winding heats. Heat is the torque
// squared low-passed over the winding's thermal time constant, so at the
// rated torque it settles at rated^2, where the limit is the rated torque;
// the fold-back starts at knee * rated^2. The drive's reported temperature
// derates the limit further, from the peak at derate_start_c to the rated
// torque at derate_end_c. sandro_rl/sandro_rl/torque_envelope.py is the same
// law for training.
struct TorqueEnvelopeSettings
{
  double peak_nm{9.0};
  double rated_nm{5.0};
  double time_constant_s{10.0};
  double knee{0.7};
  double derate_start_c{65.0};
  double derate_end_c{80.0};

  bool valid() const
  {
    return peak_nm > 0.0 && rated_nm > 0.0 && rated_nm <= peak_nm && time_constant_s > 0.0 &&
           knee >= 0.0 && knee < 1.0 && derate_end_c > derate_start_c;
  }
};

class TorqueEnvelope
{
public:
  explicit TorqueEnvelope(const TorqueEnvelopeSettings & settings = {})
  : settings_(settings) {}

  // The limit for the coming cycle; a non-finite temperature does not derate.
  double limit(const double temperature_c) const
  {
    const auto & s = settings_;
    const double rated2 = s.rated_nm * s.rated_nm;
    const double budget = std::clamp((rated2 - heat_) / ((1.0 - s.knee) * rated2), 0.0, 1.0);
    const double hot = std::isfinite(temperature_c) ?
      std::clamp((s.derate_end_c - temperature_c) / (s.derate_end_c - s.derate_start_c), 0.0, 1.0) :
      1.0;
    return s.rated_nm + (s.peak_nm - s.rated_nm) * budget * hot;
  }

  // The torque applied over the last period_s.
  void account(const double torque_nm, const double period_s)
  {
    if (!(period_s > 0.0) || !std::isfinite(torque_nm)) {
      return;
    }
    const double alpha = 1.0 - std::exp(-period_s / settings_.time_constant_s);
    heat_ += alpha * (torque_nm * torque_nm - heat_);
  }

  double heat() const {return heat_;}
  const TorqueEnvelopeSettings & settings() const {return settings_;}

private:
  TorqueEnvelopeSettings settings_;
  double heat_{0.0};
};

}  // namespace sandro_hardware
