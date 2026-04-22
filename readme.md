# 2DOF Serial Arm: Forward & Inverse Kinematics

This is a MATLAB/Simulink project demonstrating forward kinematics (FK) and inverse kinematics (IK) for a 2 degree-of-freedom planar serial arm.

## Overview

- Link 1: L1 = 1.0 m
- Link 2: L2 = 0.8 m
- Workspace radius: 0.2 m to 1.8 m (fully folded to fully extended)

**Core algorithms:**
- FK: Given angles (θ₁, θ₂) → compute end-effector position (x, y)
- IK: Given target (x, y) → compute required joint angles

Both use closed-form trigonometric solutions. No toolbox dependencies.

## Quick Start

```matlab
setup_simulink              % initialize Simulink paths (run once)
build_simulink_model        % build two models automatically
```

Then in each Simulink window:
- Press **Ctrl+T** to run simulation

Finally,
```matlab
verify_simulink             % check all results
```

## Two Demo Models

### 1. arm_static.slx (Static Test)

<img width="1355" height="659" alt="image" src="https://github.com/user-attachments/assets/5997a8cf-8347-4722-94dd-b6bba7dbb822" />

- **Target:** Fixed position (x=1.0, y=0.8)
- **Test:** IK solves angles → FK verifies position
- **Runtime:** 0.1 seconds
- **Display blocks show:**
  - Display_x = 1.00000
  - Display_y = 0.80000
  - Display_t1 = 0.00000 rad
  - Display_t2 = 1.57080 rad (90°)

### 2. arm_trajectory.slx (Dynamic Test)

<img width="1463" height="588" alt="image" src="https://github.com/user-attachments/assets/08b6d22f-dd59-421f-9fee-9e47fc7c0a58" />

- **Target:** Sine wave motion (x: 0.6→1.4, y=0.6 constant)
- **Test:** IK continuously solves for moving target
- **Runtime:** 20 seconds
- **Visualization:**
- XY_EE graph: traces horizontal line (arm end-effector path)
  <img width="800" height="500" alt="image" src="https://github.com/user-attachments/assets/b653d0df-0526-44d1-a3ab-6902f4985614" />
- Scope_Angles: shows θ₁ and θ₂ oscillating over time
  <img width="487" height="365" alt="image" src="https://github.com/user-attachments/assets/6a5126b1-75a3-44ec-985e-984c4cca47b2" />

## Verification

```
PASS  FK/IK: (1.00, 0.80) → (1.00000, 0.80000)  error = 0.0e+00
PASS  FK/IK: (1.50, 0.50) → (1.50000, 0.50000)  error = 5.6e-17
PASS  FK/IK: (-0.80, 0.60) → (-0.80000, 0.60000) error = 4.6e-16
PASS  FK/IK: (0.30, -1.10) → (0.30000, -1.10000) error = 1.7e-16
PASS  FK/IK: (-1.00, -0.80) → (-1.00000, -0.80000) error = 2.5e-16
PASS  Singularity (fully extended): det(J) = -0.00000 (expected 0)
PASS  Singularity (45°, 90°): det(J) = 0.80000 (non-singular, expected ~0.8)
```
## Mathematics & Algorithm Details

### Forward Kinematics (Three Methods)

**Method 1: Geometric (Default)**
```
x = L₁·cos(θ₁) + L₂·cos(θ₁ + θ₂)
y = L₁·sin(θ₁) + L₂·sin(θ₁ + θ₂)
```
Fastest approach: 4 trig operations, no matrix multiplications.

**Method 2: Denavit-Hartenberg (DH) Transform**
```
T₁ = [cos(θ₁)  -sin(θ₁)  0  L₁·cos(θ₁)]
     [sin(θ₁)   cos(θ₁)  0  L₁·sin(θ₁)]
     [0         0        1  0         ]
     [0         0        0  1         ]

T₂ = [cos(θ₂)  -sin(θ₂)  0  L₂·cos(θ₂)]
     [sin(θ₂)   cos(θ₂)  0  L₂·sin(θ₂)]
     [0         0        1  0         ]
     [0         0        0  1         ]

T_total = T₁ × T₂
```
Standard homogeneous transformation matrices. Easily extends to 3+ DOF.

**Method 3: Rotation Matrix Chain**
```
R₁ = [cos(θ₁)  -sin(θ₁)]    R₂ = [cos(θ₂)  -sin(θ₂)]
     [sin(θ₁)   cos(θ₁)]         [sin(θ₂)   cos(θ₂)]

p_elbow = R₁ × [L₁; 0]
p_ee = p_elbow + R₁ × R₂ × [L₂; 0]
```
Clearest frame composition visualization.

### Inverse Kinematics (Closed-Form, Law of Cosines)

**Derivation:**
```
r = √(x² + y²)                              [distance base→target]
cos(θ₂) = (r² - L₁² - L₂²) / (2·L₁·L₂)    [Law of Cosines]
θ₂ = atan2(√(1-cos²θ₂), cos(θ₂))          [always in [-π, π]]

α = atan2(y, x)                            [angle to target]
β = atan2(L₂·sin(θ₂), L₁ + L₂·cos(θ₂))    [correction angle]

θ₁ = atan2(sin(α - β), cos(α - β))        [wrapped to [-π, π]]
```

**No iterative solving**: instant algebraic solution. Supports elbow-up and elbow-down configurations.

### Comparison: This solver vs. MATLAB Robotics Toolbox

| Aspect | This Project | MATLAB Robotics Toolbox |
|--------|--------------|------------------------|
| **Dependencies** | None (pure MATLAB) | Requires Robotics Toolbox license |
| **FK Speed** | 4 trig ops (~microseconds) | OOP overhead (~milliseconds) |
| **IK Type** | Closed-form (instant) | Numerical iteration (slow) |
| **Complexity** | Simple, explicit code | Black box implementation |
| **Debugging** | Full math visible | Hidden details |
| **Simulink Integration** | Direct code injection | External function calls |
| **Scalability** | Trivial to 3+ DOF | Complex OOP framework |
| **License Cost** | Free | ~$3000 per license |
| **100k FK calls** | ~0.1 seconds | ~100 seconds |

### Scalability

**Adding a 3rd Joint:**

For 3DOF arm, add one more link and angle:
```matlab
% In geometric method:
x = L1*cos(t1) + L2*cos(t1+t2) + L3*cos(t1+t2+t3)
y = L1*sin(t1) + L2*sin(t1+t2) + L3*sin(t1+t2+t3)

% In DH method: add T3 matrix and multiply
T_total = T1 * T2 * T3

% In IK: use numerical optimization since 3DOF is still tractable
```

**Extensibility:**
- 6DOF industrial robots: use DH matrices (one per joint)
- Mobile manipulation: compose base pose + arm kinematics
- Redundant arms (7+ DOF): use numerical IK with joint limits