"""
Closed-chain (five-bar) inverse kinematics — symbolic derivation with SymPy.

Geometry:
    +x right, +y down (your convention).
    Motor 1 at (0, 0); motor 2 at (a, 0).
    Upper arms (length l1) at angles th1, th2 from +y.
    Forearms (length l2) at angles b1, b2 from +x.
    th1 cw, th2 ccw, b1 cw, b2 ccw — both arms swing inward,
    both forearms reach into +y (downward, into the workspace).

Closure equations (as the user wrote them):
    (1)  l1*cos(th1) + l2*sin(b1) = l1*cos(th2) + l2*sin(b2)
    (2) -l1*sin(th1) + l2*cos(b1) = a + l1*sin(th2) - l2*cos(b2)
"""

import sympy as sp

# ---------------------------------------------------------------------------
# 1. Symbolic setup
# ---------------------------------------------------------------------------
l1, l2, a = sp.symbols('l1 l2 a', positive=True)
th1, th2 = sp.symbols('theta1 theta2', real=True)
b1, b2 = sp.symbols('b1 b2', real=True)

# Closure equations exactly as given
eq1 = sp.Eq(l1*sp.cos(th1) + l2*sp.sin(b1), l1*sp.cos(th2) + l2*sp.sin(b2))
eq2 = sp.Eq(-l1*sp.sin(th1) + l2*sp.cos(b1), a + l1*sp.sin(th2) - l2*sp.cos(b2))

print("Closure equation 1 (y):")
sp.pprint(eq1)
print("\nClosure equation 2 (x):")
sp.pprint(eq2)

# ---------------------------------------------------------------------------
# 2. Move knowns to RHS:  define P (sin-difference) and Q (cos-sum)
# ---------------------------------------------------------------------------
# eq1  ->  l2*(sin(b1) - sin(b2)) = P
# eq2  ->  l2*(cos(b1) + cos(b2)) = Q
P = l1*(sp.cos(th2) - sp.cos(th1))
Q = a + l1*(sp.sin(th1) + sp.sin(th2))

print("\nP =", P)
print("Q =", Q)

# ---------------------------------------------------------------------------
# 3. Sum-to-product substitution: s = (b1+b2)/2,  d = (b1-b2)/2
#       sin(b1) - sin(b2) = 2 cos(s) sin(d)
#       cos(b1) + cos(b2) = 2 cos(s) cos(d)
# ---------------------------------------------------------------------------
s, d = sp.symbols('s d', real=True)

lhs1 = 2*l2*sp.cos(s)*sp.sin(d)         # = P
lhs2 = 2*l2*sp.cos(s)*sp.cos(d)         # = Q

# tan(d) = P / Q
d_sol = sp.atan2(P, Q)

# cos(s) = sqrt(P^2 + Q^2) / (2 l2)    (taking + branch — assembly mode)
r = sp.sqrt(P**2 + Q**2)
s_sol = sp.acos(r / (2*l2))

b1_expr = s_sol + d_sol
b2_expr = s_sol - d_sol

print("\nClosed-form solution:")
print("  d =", d_sol)
print("  s =", s_sol)
print("  b1 = s + d")
print("  b2 = s - d")

# ---------------------------------------------------------------------------
# 4. Numerical verification: plug values in and check residuals are zero
# ---------------------------------------------------------------------------
import math

def solve_b(th1_v, th2_v, l1_v=1.0, l2_v=1.2, a_v=1.0, sigma=+1):
    P_v = l1_v*(math.cos(th2_v) - math.cos(th1_v))
    Q_v = a_v + l1_v*(math.sin(th1_v) + math.sin(th2_v))
    r_v = math.hypot(P_v, Q_v)
    if r_v > 2*l2_v:
        raise ValueError(f"Out of workspace: r={r_v:.3f} > 2*l2={2*l2_v:.3f}")
    d_v = math.atan2(P_v, Q_v)
    s_v = sigma * math.acos(r_v / (2*l2_v))
    return s_v + d_v, s_v - d_v

print("\n" + "="*60)
print("Numerical verification")
print("="*60)

test_cases = [
    (0.0,  0.0),
    (0.3,  0.3),
    (0.4, -0.2),
    (-0.5, 0.6),
    (1.0,  0.8),
]

L1, L2, A = 1.0, 1.2, 1.0
for t1, t2 in test_cases:
    try:
        bb1, bb2 = solve_b(t1, t2, L1, L2, A)
    except ValueError as e:
        print(f"th1={t1:+.2f} th2={t2:+.2f} -> {e}")
        continue
    # Residuals of the original equations
    res1 = (L1*math.cos(t1) + L2*math.sin(bb1)) - (L1*math.cos(t2) + L2*math.sin(bb2))
    res2 = (-L1*math.sin(t1) + L2*math.cos(bb1)) - (A + L1*math.sin(t2) - L2*math.cos(bb2))
    print(f"th1={t1:+.2f} th2={t2:+.2f} -> "
          f"b1={bb1:+.4f} b2={bb2:+.4f}  "
          f"residuals: {res1:+.2e}, {res2:+.2e}")

# ---------------------------------------------------------------------------
# 5. SymPy cross-check: substitute and simplify
# ---------------------------------------------------------------------------
print("\nSymbolic residual check (should simplify to 0):")
subs = {b1: b1_expr, b2: b2_expr}
r1 = sp.simplify(eq1.lhs.subs(subs) - eq1.rhs.subs(subs))
r2 = sp.simplify(eq2.lhs.subs(subs) - eq2.rhs.subs(subs))
print("  eq1 residual:", r1)
print("  eq2 residual:", r2)
# Note: SymPy may not fully simplify these to 0 due to atan2/acos branches,
# but the numerical check above confirms correctness.

