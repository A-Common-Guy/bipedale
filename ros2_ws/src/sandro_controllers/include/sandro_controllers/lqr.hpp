#pragma once

// Small fixed-size linear-quadratic regulator toolbox for the whole-body
// balance task. Everything is stack-allocated so the schedule can be rebuilt
// from a parameter callback and read from the realtime loop without touching
// the heap.

#include <Eigen/Dense>

namespace sandro_controllers::lqr {

// State: [x, x_dot, pitch, pitch_rate, leg_offset]; input: [total wheel
// torque, leg_offset_rate]. See whole_body_model.hpp for the derivation.
constexpr int kStates = 5;
constexpr int kInputs = 2;

using StateMatrix = Eigen::Matrix<double, kStates, kStates>;
using InputMatrix = Eigen::Matrix<double, kStates, kInputs>;
using GainMatrix = Eigen::Matrix<double, kInputs, kStates>;
using InputWeight = Eigen::Matrix<double, kInputs, kInputs>;
using StateVector = Eigen::Matrix<double, kStates, 1>;
using InputVector = Eigen::Matrix<double, kInputs, 1>;

// exp(M) through scaling and squaring of a Taylor series. Adequate for the
// small, well-conditioned matrices used here; not a general-purpose expm.
template <int N>
Eigen::Matrix<double, N, N> expm(const Eigen::Matrix<double, N, N> &matrix) {
  using Matrix = Eigen::Matrix<double, N, N>;
  const double magnitude = matrix.cwiseAbs().maxCoeff() * N;
  int squarings = 0;
  double scale = 1.0;
  while (magnitude * scale > 0.5 && squarings < 40) {
    scale *= 0.5;
    ++squarings;
  }
  const Matrix scaled = matrix * scale;
  Matrix result = Matrix::Identity();
  Matrix term = Matrix::Identity();
  for (int k = 1; k <= 16; ++k) {
    term = (term * scaled) / static_cast<double>(k);
    result += term;
  }
  for (int i = 0; i < squarings; ++i) {
    result = result * result;
  }
  return result;
}

struct DiscreteModel {
  StateMatrix A{StateMatrix::Zero()};
  InputMatrix B{InputMatrix::Zero()};
};

// Zero-order-hold discretisation of x_dot = A x + B u.
DiscreteModel discretize(const StateMatrix &A, const InputMatrix &B,
                         double period_s);

struct DareSolution {
  StateMatrix P{StateMatrix::Zero()};
  GainMatrix K{GainMatrix::Zero()};
  int iterations{};
  bool converged{};
  // Largest closed-loop eigenvalue magnitude of A - B K; < 1 is stable.
  double spectral_radius{};
};

// Stabilising solution of the discrete-time algebraic Riccati equation by the
// structure-preserving doubling algorithm, with the feedback u = -K x.
DareSolution solve_dare(const StateMatrix &A, const InputMatrix &B,
                        const StateMatrix &Q, const InputWeight &R,
                        int max_iterations = 200, double tolerance = 1e-12);

double spectral_radius(const StateMatrix &matrix);

// Largest number of whole samples of pure input delay the discrete closed
// loop x+ = A x + B u(k - d), u = -K x tolerates while staying stable, up to
// max_samples. Computed by augmenting the state with the delayed inputs; off
// the realtime path only.
int delay_margin_samples(const StateMatrix &A, const InputMatrix &B,
                         const GainMatrix &K, int max_samples);

} // namespace sandro_controllers::lqr
