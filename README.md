# 🤖 RoboTool – URDF-Native Robotics Kinematics Library

A lightweight robotics kinematics library for URDF-based robot analysis, differential kinematics, and numerical inverse kinematics.

RoboTool provides a transparent robotics workflow that bridges URDF robot models with executable kinematic analysis while keeping the underlying mathematics explicit and accessible.

> **From URDF → Forward Kinematics → Jacobians → Adaptive Inverse Kinematics**

---

# 🚀 Features

## URDF Support

- URDF parser
- Native URDF kinematic trees
- Arbitrary joint-axis support
- Automatic kinematic chain generation

Supported joint types:

- Revolute
- Continuous
- Prismatic
- Fixed

---

## Forward Kinematics

- Homogeneous transformation propagation
- Full kinematic chain computation
- Arbitrary joint orientations
- Rodrigues rotation formulation
- End-effector pose computation

---

## Differential Kinematics

- Geometric Jacobian computation
- Linear Jacobian
- Angular Jacobian
- Numerical Jacobian validation
- Finite-difference verification
- Differential workspace analysis

---

## Inverse Kinematics

- Iterative numerical IK
- Adaptive Damped Least Squares (DLS)
- Position-only IK
- Orientation-aware IK (position + orientation, full 6-DOF pose error)
- Adaptive damping using Jacobian conditioning
- Yoshikawa manipulability analysis
- Automatic singularity escape
- Adaptive step-size limiting
- Convergence detection
- Stall detection
- Workspace validation
- Unreachable target detection

---

## CLI Utilities

- Forward Kinematics inspection
- Jacobian computation
- Numerical Jacobian validation
- Target-based IK solving (position, with optional orientation target)

> **Note:** the CLI is a thin diagnostic wrapper around the library. [RobotCAD](https://github.com/drfenixion/freecad.robotcad) currently serves as the primary integration platform for RoboTool.

---

# 🧠 Motivation

RoboTool originated from the need for a lightweight robotics toolkit capable of performing kinematic analysis directly from URDF robot descriptions.

During robotics development workflows, many existing solutions required heavyweight simulation environments, proprietary software, or tightly coupled frameworks that made experimentation and debugging unnecessarily difficult.

RoboTool was created to provide:

- transparent robotics mathematics
- lightweight execution
- reproducible numerical algorithms
- direct access to intermediate computations
- URDF-native workflows

Instead of hiding robotics concepts behind large frameworks, RoboTool exposes every computational step.

---

# 🎯 Design Goals

RoboTool emphasizes:

- Explicit robotics mathematics
- Lightweight implementation
- Numerical robustness
- Reproducibility
- Simulation interoperability
- URDF-native workflows
- Educational transparency

---

# ⚙️ Mathematical Foundations

---

## Homogeneous Transformation

Each joint transformation is represented by a homogeneous transformation matrix

```math
T=
\begin{bmatrix}
R & p\\
0_{1\times3} & 1
\end{bmatrix}
```

where

- $R\in\mathbb{R}^{3\times3}$ is the rotation matrix.
- $p\in\mathbb{R}^{3}$ is the translation vector.

---

## Rodrigues Rotation Formula

Joint rotations are computed using Rodrigues' rotation formula

```math
R(\theta)
=
I
+
\sin(\theta)K
+
(1-\cos\theta)K^2
```

where

```math
K=
\begin{bmatrix}
0&-u_z&u_y\\
u_z&0&-u_x\\
-u_y&u_x&0
\end{bmatrix}
```

allowing arbitrary joint axes extracted directly from URDF.

---

## Kinematic Propagation

Forward kinematics is computed as

```math
{}^{0}T_n
=
\prod_{i=1}^{n}
{}^{i-1}T_i(q_i)
```

---

# Geometric Jacobian

The geometric Jacobian is constructed explicitly.

For revolute joints

```math
J_i=
\begin{bmatrix}
z_i\times(p_n-p_i)\\
z_i
\end{bmatrix}
```

For prismatic joints

```math
J_i=
\begin{bmatrix}
z_i\\
0_{3\times1}
\end{bmatrix}
```

Joint axes are projected into the global frame

```math
z_i
=
{}^{0}R_i\hat{u}_i
```

allowing arbitrary URDF joint orientations.

---

# Numerical Jacobian Validation

Analytical Jacobians are validated through finite differences, applied independently to the linear and angular blocks of the Jacobian.

**Linear block** (end-effector position with respect to joint $i$):

```math
J_{v_i}
\approx
\frac{
p(q+\delta q_i)-p(q)
}{
\delta q_i
}
```

**Angular block** (end-effector orientation with respect to joint $i$), using the first-order rotation vector approximation obtained from the relative rotation matrix between the perturbed and nominal configurations:

```math
J_{\omega_i}
\approx
\frac{
\phi\!\left(R(q)^TR(q+\delta q_i)\right)
}{
\delta q_i
}
```

where $\phi(\cdot)$ denotes the axis-angle rotation vector extracted from a relative rotation matrix using its skew-symmetric part, providing a first-order approximation of the angular velocity in $\mathbb{R}^3$.

Typical validation accuracy

```text
Linear Jacobian error  : ~1e-7
Angular Jacobian error : ~1e-13
```

---

# Inverse Kinematics

RoboTool solves inverse kinematics using an iterative Adaptive Damped Least Squares algorithm, supporting both position-only and full 6-DOF pose targets.

**Position-only error**

```math
e_p
=
x_d-x
\in\mathbb{R}^3
```

**Pose error (position + orientation)**, when an orientation target is provided:

```math
e
=
\begin{bmatrix}
e_p\\
e_o
\end{bmatrix}
\in\mathbb{R}^6,
\qquad
e_o=\phi(R_dR^T)
```

where

- $e_p\in\mathbb{R}^3$ is the Cartesian position error.
- $e_o\in\mathbb{R}^3$ is the orientation error represented as an axis-angle rotation vector.
- $R_d$ is the desired end-effector orientation.
- $R$ is the current end-effector orientation.
- $x_d\in\mathbb{R}^3$ is the desired end-effector position.
- $x\in\mathbb{R}^3$ is the current end-effector position.

When no orientation target is provided, the solver falls back to the position-only formulation and uses only the linear block of the Jacobian.

---

## Adaptive Damped Least Squares

The inverse kinematics solver is based on a Damped Least Squares formulation computed through the Singular Value Decomposition (SVD).

```math
J
=
U\Sigma V^T
```

The joint update is computed as

```math
\Delta q
=
V
\,
\mathrm{diag}
\left(
\frac{\sigma_i}
{\sigma_i^2+\lambda^2}
\right)
U^Te
```

where

- $\sigma_i$ are the singular values of the Jacobian matrix.
- $\lambda$ is the adaptive damping coefficient, adjusted according to the Jacobian condition number.
- $e$ is either the position error $e_p$ or the full pose error $[e_p;\,e_o]$, depending on whether an orientation target is provided.

The damping coefficient increases automatically as the Jacobian becomes ill-conditioned, improving numerical stability near singular configurations.

---

## Manipulability Analysis

To detect singular configurations, RoboTool computes the Yoshikawa manipulability index using the **linear component** of the geometric Jacobian:

```math
w
=
\sqrt{
\det(J_vJ_v^T)
}
```

Configurations with very low manipulability trigger an automatic singularity escape procedure before optimization continues.

---

## Numerical Stability

The solver includes several stabilization mechanisms:

- Adaptive DLS damping
- Adaptive step-size limiting
- Manipulability analysis
- Automatic singularity escape
- Stall detection
- Convergence thresholds
- Optional joint wrapping
- Workspace validation

---

# 🏗️ Architecture

```text
URDF
   │
   ▼
Robot Model
   │
   ▼
Forward Kinematics
   │
   ▼
Geometric Jacobian
   │
   ├──────────────► Numerical Validation
   │
   ▼
Adaptive DLS IK
   │
   ▼
Joint Configuration
```

---

# 📂 Project Structure

```text
robotool/
├── cli.py
├── models.py
├── urdf_parser.py
├── kinematics.py
├── jacobians.py
├── inverse_kinematics.py
└── models_robot/
```

---

# 🚀 CLI

Forward Kinematics

```bash
python3 cli.py robot.urdf
```

Inverse Kinematics (position only)

```bash
python3 cli.py robot.urdf --target X Y Z
```

Inverse Kinematics (position + orientation)

```bash
python3 cli.py robot.urdf --target X Y Z --target-rpy ROLL PITCH YAW
```

> `--target-rpy` requires `--target` and expects roll, pitch, and yaw in radians.

---

# 🧪 Validation

The project contains multiple experimental URDF models intentionally designed to stress-test

- Singular configurations
- Unreachable targets
- Malformed kinematic chains
- Constrained workspaces
- Arbitrary joint axes

---

# 📦 Installation

```bash
git clone https://github.com/u4456163-wq/RoboTool.git
cd RoboTool
pip install -r requirements.txt
```

---

# 🛣️ Roadmap

## Implemented

- ✅ URDF Parsing
- ✅ Forward Kinematics
- ✅ Geometric Jacobians
- ✅ Numerical Jacobian Validation
- ✅ Adaptive Damped Least Squares IK
- ✅ Orientation-aware IK
- ✅ Manipulability Analysis
- ✅ Singularity Escape
- ✅ Adaptive Step Limiting

## Planned

- Joint limits
- Null-space optimization
- Collision-aware IK
- Trajectory generation
- Dynamics engine
- Tree-structured IK
- ROS2 bridge
- Visualization backend

---

# 🧩 Design Philosophy

RoboTool follows a simple principle:

> **Robotics software should expose the mathematics, not hide them.**

The library favors transparency over abstraction by making every stage of the kinematic computation accessible, inspectable, and reproducible.

Rather than hiding robotics behind heavyweight frameworks, RoboTool exposes

- Homogeneous transformations
- Geometric Jacobians
- Manipulability analysis
- Numerical inverse kinematics
- Differential robot motion

allowing users to understand and control every computation.

---

# 🤝 Credits

Inspired by

- URDF robotics workflows
- Geometric robotics
- Differential kinematics
- Numerical optimization
- Modern Robotics (Lynch & Park)
- RobotCAD
