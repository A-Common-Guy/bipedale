#include "sandro_controllers/lqr.hpp"

#include <algorithm>
#include <cmath>
#include <complex>

namespace sandro_controllers::lqr {

DiscreteModel discretize(const StateMatrix &A, const InputMatrix &B,
                         const double period_s) {
  constexpr int kAugmented = kStates + kInputs;
  Eigen::Matrix<double, kAugmented, kAugmented> augmented =
      Eigen::Matrix<double, kAugmented, kAugmented>::Zero();
  augmented.topLeftCorner<kStates, kStates>() = A * period_s;
  augmented.topRightCorner<kStates, kInputs>() = B * period_s;
  const auto exponential = expm<kAugmented>(augmented);
  DiscreteModel result;
  result.A = exponential.topLeftCorner<kStates, kStates>();
  result.B = exponential.topRightCorner<kStates, kInputs>();
  return result;
}

double spectral_radius(const StateMatrix &matrix) {
  const Eigen::EigenSolver<StateMatrix> solver(matrix, false);
  double radius = 0.0;
  for (int i = 0; i < kStates; ++i) {
    radius = std::max(radius, std::abs(solver.eigenvalues()[i]));
  }
  return radius;
}

DareSolution solve_dare(const StateMatrix &A, const InputMatrix &B,
                        const StateMatrix &Q, const InputWeight &R,
                        const int max_iterations, const double tolerance) {
  DareSolution solution;
  // Structure-preserving doubling (Chu, Fan, Lin, Wang): iterate
  //   A_{k+1} = A_k W_k^{-1} A_k
  //   G_{k+1} = G_k + A_k W_k^{-1} G_k A_k^T
  //   H_{k+1} = H_k + A_k^T H_k W_k^{-1} A_k,   W_k = I + G_k H_k,
  // starting from A_0 = A, G_0 = B R^{-1} B^T, H_0 = Q. H_k converges
  // quadratically to the stabilising Riccati solution, independent of how
  // slow the closed-loop modes are, which plain value iteration is not.
  StateMatrix Ak = A;
  StateMatrix Gk = B * R.inverse() * B.transpose();
  StateMatrix Hk = Q;
  bool converged = false;
  int iterations = 0;
  for (; iterations < max_iterations; ++iterations) {
    const StateMatrix W = StateMatrix::Identity() + Gk * Hk;
    const Eigen::FullPivLU<StateMatrix> lu(W);
    if (!lu.isInvertible()) {
      break;
    }
    const StateMatrix W_inv_A = lu.solve(Ak);
    const StateMatrix W_inv_G = lu.solve(Gk);
    const StateMatrix A_next = Ak * W_inv_A;
    const StateMatrix G_next = Gk + Ak * W_inv_G * Ak.transpose();
    const StateMatrix H_next = Hk + Ak.transpose() * Hk * W_inv_A;
    const double change = (H_next - Hk).cwiseAbs().maxCoeff();
    const double scale = std::max(1.0, H_next.cwiseAbs().maxCoeff());
    Ak = A_next;
    Gk = 0.5 * (G_next + G_next.transpose());
    Hk = 0.5 * (H_next + H_next.transpose());
    if (!Hk.allFinite()) {
      break;
    }
    if (change <= tolerance * scale) {
      converged = true;
      ++iterations;
      break;
    }
  }
  solution.iterations = iterations;
  solution.P = Hk;
  if (!converged || !Hk.allFinite()) {
    solution.converged = false;
    return solution;
  }
  const InputWeight S = R + B.transpose() * Hk * B;
  const Eigen::FullPivLU<InputWeight> lu(S);
  if (!lu.isInvertible()) {
    solution.converged = false;
    return solution;
  }
  solution.K = lu.solve(B.transpose() * Hk * A);
  solution.spectral_radius = spectral_radius(A - B * solution.K);
  solution.converged = solution.K.allFinite() &&
                       std::isfinite(solution.spectral_radius) &&
                       solution.spectral_radius < 1.0;
  return solution;
}

int delay_margin_samples(const StateMatrix &A, const InputMatrix &B,
                         const GainMatrix &K, const int max_samples) {
  if (spectral_radius(A - B * K) >= 1.0) {
    return -1;
  }
  for (int d = 1; d <= max_samples; ++d) {
    // Augmented state [x, u(k-1), ..., u(k-d)]: the plant sees the oldest
    // stored input, each stage shifts, and the newest is -K x.
    const int n = kStates + kInputs * d;
    Eigen::MatrixXd augmented = Eigen::MatrixXd::Zero(n, n);
    augmented.topLeftCorner(kStates, kStates) = A;
    augmented.block(0, kStates + kInputs * (d - 1), kStates, kInputs) = B;
    augmented.block(kStates, 0, kInputs, kStates) = -K;
    for (int i = 1; i < d; ++i) {
      augmented.block(kStates + kInputs * i, kStates + kInputs * (i - 1),
                      kInputs, kInputs) =
          Eigen::MatrixXd::Identity(kInputs, kInputs);
    }
    const Eigen::EigenSolver<Eigen::MatrixXd> solver(augmented, false);
    double radius = 0.0;
    for (int i = 0; i < n; ++i) {
      radius = std::max(radius, std::abs(solver.eigenvalues()[i]));
    }
    if (!std::isfinite(radius) || radius >= 1.0) {
      return d - 1;
    }
  }
  return max_samples;
}

} // namespace sandro_controllers::lqr
