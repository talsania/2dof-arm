# 2DOF Serial Arm: Kinematics, Jacobian & Pick-and-Place

MATLAB project implementing forward kinematics, inverse kinematics (closed-form + Newton-Raphson), velocity IK, Jacobian manipulability, and a pick-and-place trajectory planner for a 2-DOF planar serial arm. No toolbox dependencies.

## Arm Parameters

- Link 1: L1 = 1.0 m
- Link 2: L2 = 0.8 m
- Workspace radius: 0.2 m to 1.8 m

---

## How It Was Built: The Three Layers

### Layer 1: FK + Closed-Form IK

`custom_FK_2DOF` implements three methods that must agree to < 1e-10 m:

**Geometric** (default, 4 trig ops):
```
x = L1·cos(θ1) + L2·cos(θ1+θ2)
y = L1·sin(θ1) + L2·sin(θ1+θ2)
```

**DH Transform** (4×4 matrix chain, extends to N-DOF):
```
T_total = T1 × T2
```

**Rotation chain** (explicit SE2 composition):
```
p_ee = R1·[L1;0] + R1·R2·[L2;0]
```

`custom_IK_2DOF` solves closed-form via the law of cosines:
```
cos(θ2) = (r² - L1² - L2²) / (2·L1·L2)
θ1      = atan2(sin(α-β), cos(α-β))     wrapped to [-π, π]
```
Supports elbow-up and elbow-down. Round-trip FK to IK to FK error < 1e-10 m across 1000 random samples.

---

### Layer 2: Jacobian + Velocity IK

`custom_jacobian` computes the 2×2 geometric Jacobian:
```
J = [ -L1·s1 - L2·s12,  -L2·s12 ]
    [  L1·c1 + L2·c12,   L2·c12 ]
```

Manipulability (Yoshikawa): `w = |det(J)| = |L1·L2·sin(θ2)|`  
Singular at θ2 = 0 or ±π. Maximum w = L1·L2 = 0.8 at θ2 = π/2.

`custom_IK_velocity` resolves θ̇ from ẋ = J·θ̇ via four modes:

| Mode | When used |
|------|-----------|
| `exact` | w > 0.1, direct J inverse |
| `dls` | 0.01 < w <= 0.1, damped least squares |
| `transpose` | w <= 0.01, Jᵀ, always safe |
| `auto` | selects above by w automatically |

`demo.m` shows the manipulability ellipse for 6 configurations and animates it sweeping θ2 from 10° to 170° and back:

<p align="center">
  <img src="https://github.com/user-attachments/assets/1005435f-1ee1-4eaa-838c-e10c4edbb6e9" width="100%" />
</p>

<br/>

<p align="center">
  <img src="https://github.com/user-attachments/assets/8a701580-84dd-456f-a74a-b1dfbeb48e7d" width="69%" />
</p>

<br/>

<p align="center">
  <img src="https://github.com/user-attachments/assets/2d359e31-155b-4c9f-a0a0-89b456186778" width="69%" />
</p>

---

### Layer 3: Newton-Raphson IK + Pick-and-Place

`custom_IK_newton` adds an iterative Newton-Raphson solver on top of the closed-form seed:

- Seeds from `custom_IK_2DOF`, typically converges in 3 iterations or fewer
- Switches to DLS (λ = 0.01) near singularities where |det(J)| < 1e-4
- Default tolerance: 1e-10 m; angles wrapped to [-π, π]
- Fallback chain in `custom_pick_place`: NR fails, drops to closed-form, then reuses previous angles

`custom_pick_place` plans a 7-phase trajectory:

| Phase | Segment |
|-------|---------|
| 1 | Home to Above Pick |
| 2 | Above Pick to Pick |
| 3 | Pick to Above Pick |
| 4 | Above Pick to Above Place |
| 5 | Above Place to Place |
| 6 | Place to Above Place |
| 7 | Above Place to Home |

Each phase is a straight Cartesian line with uniform step spacing. Total waypoints = 7 x n_steps (default 210).

---

## Pick-and-Place Demo

To change pick/place targets, edit the top of `demo_pick_place.m`:
```matlab
pick_pt  = [0.8,  0.2];   % object location
place_pt = [-0.7, 0.3];   % drop location
```

Run from project root:
```matlab
demo_pick_place
```

Produces three figures and a live animation.

**Figure 1: Full trajectory overview**
- Cartesian path coloured by phase (7 colours)
- Manipulability strip below shows w along all waypoints
- PICK (red x), PLACE (blue x), HOME (black square), hover lines shown

<p align="center">
  <img src="https://github.com/user-attachments/assets/43fadf38-32c6-429f-ad1b-21a3ed40b6fc" width="69%" />
</p>

**Figure 2: Phase snapshots**
- One subplot per phase, arm drawn at mid-point of that segment
- Manipulability ellipse drawn at end-effector for each pose
- Title shows phase name and w value

<p align="center">
  <img width="1682" height="525" alt="Pick   Place - Phase Snapshots" src="https://github.com/user-attachments/assets/d4857d7b-9d6d-4e8a-ae12-7d4fba642273" />
</p>

**Figure 3: Live animation**
- Arm moves through all 7 phases
- Ellipse drawn at EE, gripper state shown (open / closed)
- Manipulability bar at bottom

<p align="center">
  <img src="https://github.com/user-attachments/assets/653242f5-d0f7-4a85-b32a-ef7bd1d6b7fb" width="69%" />
</p>

Typical output:
```
Trajectory: 280 waypoints  |  valid=1  |  n_failed=0
```

---

## Tests

```
tests/main_test_2DOF.m      FK (3 methods), IK, 1000-sample round-trip, singularity, DH
tests/test_jacobian.m       Jacobian formula, FD cross-check, velocity IK all modes (28 tests)
tests/test_newton.m         convergence, seeds, accuracy vs closed-form, edge cases (21 tests)
tests/test_pick_place.m     output contract, phases, waypoints, FK consistency (44 tests)
tests/test_ellipse_math.m   SVD axes, w consistency, singularity, monotone ratio (9 tests)
```

Run all from project root:
```matlab
cd tests
main_test_2DOF
test_jacobian
test_newton
test_pick_place
test_ellipse_math
```

All tests use `assert()` with per-test tolerances:

| Quantity | Tolerance |
|----------|-----------|
| Position | 1e-9 m |
| FK error | 1e-7 m |
| Manipulability | 1e-12 |
| Angle | 1e-9 rad |

---

## Simulink Models

Setup (once per session):
```matlab
cd simulink
setup_simulink
```

Build both models:
```matlab
build_simulink_model
```

**arm_static.slx** fixed target (x=1.0, y=0.8), runtime 0.1 s  
Expected display values: x=1.00000, y=0.80000, θ1=0.00000 rad, θ2=1.57080 rad

<p align="center">
  <img src="https://github.com/user-attachments/assets/ff486571-1422-4425-b1ad-9f0ae5eeacef" width="80%" />
</p>

**arm_trajectory.slx** sine-wave x target (0.6 to 1.4), y=0.6 constant, runtime 20 s  
XY Graph traces a horizontal line; Scope shows θ1 and θ2 oscillating.

<p align="center">
  <img src="https://github.com/user-attachments/assets/afe611cc-29be-4585-961a-bc967453b0c9" width="45%" />
  <img src="https://github.com/user-attachments/assets/5141fe42-b68e-4209-b7b3-fad90af60dcc" width="45%" />
</p>

Verify results after running:
```matlab
cd ../tests
verify_simulink
```

---

## Quick Reference

```matlab
% FK
[x, y] = custom_FK_2DOF(theta1, theta2, L1, L2)

% IK closed-form
[t1, t2, ok] = custom_IK_2DOF(x, y, L1, L2)          % elbow-up
[t1, t2, ok] = custom_IK_2DOF(x, y, L1, L2, 0)       % elbow-down

% IK Newton-Raphson
[t1, t2, ok, iters, hist] = custom_IK_newton(x, y, L1, L2)

% Jacobian
[J, w, det_J, is_singular] = custom_jacobian(t1, t2, L1, L2)

% Velocity IK
theta_dot = custom_IK_velocity(t1, t2, x_dot, L1, L2)        % auto
theta_dot = custom_IK_velocity(t1, t2, x_dot, L1, L2, 'dls') % explicit

% Pick-and-place
traj = custom_pick_place([0.8 0.2], [-0.7 0.3], L1, L2)
traj = custom_pick_place(pick, place, L1, L2, opts)   % with options
% opts fields: hover_height, home, n_steps, elbow_up

% Inspect trajectory
traj.valid          % 1 if all IK solved
traj.n_failed       % count of fallback waypoints
traj.w              % manipulability at each waypoint
traj.phase_labels   % cell array of phase names
```
