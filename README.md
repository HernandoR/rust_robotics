RustRobotics
====

This package is a rust implementation of [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics).

Build
```
git clone https://github.com/HernandoR/rust_robotics.git
cd rust_robotics
cargo build
```

Run (Example)
```
cargo run --bin ekf
```

# Table of Contents
- [RustRobotics](#rustrobotics)
- [Table of Contents](#table-of-contents)
- [Localization](#localization)
  - [Extended Kalman Filter Localization](#extended-kalman-filter-localization)
  - [Particle Filter Localization](#particle-filter-localization)
- [SLAM](#slam)
  - [Iterative Closest Point](#iterative-closest-point)
  - [FastSLAM 1.0](#fastslam-10)
- [Path Planning](#path-planning)
  - [Bezier Path](#bezier-path)
  - [Cubic Spline](#cubic-spline)
  - [Dynamic Window Approach](#dynamic-window-approach)
  - [Model Predictive Trajectory Generator](#model-predictive-trajectory-generator)
  - [Dijkstra algorithm](#dijkstra-algorithm)
  - [Potential Field algorithm](#potential-field-algorithm)
  - [State Lattice Planner](#state-lattice-planner)
  - [Rapidly-Exploring Random Trees](#rapidly-exploring-random-trees)
- [Path Tracking](#path-tracking)
  - [Move to Pose](#move-to-pose)
  - [Pure Pursuit](#pure-pursuit)
  - [Stanly Control](#stanly-control)
  - [LQR steer control](#lqr-steer-control)
  - [Nonlinear Model predictive control with C-GMRES](#nonlinear-model-predictive-control-with-c-gmres)

# Localization
## Extended Kalman Filter Localization

<img src="./img/ekf.svg" width="640px">


Red:GPS, Brue:Ground Truth, Green:EKF, Yellow:Dead Reckoning

- [src](https://github.com/rsasaki0109/RustRobotics/blob/master/src/bin/ekf.rs)

```
cargo run --bin ekf
```

## Particle Filter Localization

# SLAM
## Iterative Closest Point
## FastSLAM 1.0


# Path Planning
## Bezier Path

<img src="./img/bezier_path.svg" width="640px">

Brack:Control points, Green: Path, Red: Start and Goal

- [src](https://github.com/rsasaki0109/RustRobotics/blob/master/src/bin/bezier_path.rs)

```
cargo run --bin bezier_path
```

## Cubic Spline


<img src="./img/csp.svg" width="640px">

Brack:Control points, Green: Path

- [src](https://github.com/rsasaki0109/RustRobotics/blob/master/src/cubic_spline_planner.rs)

```
cargo run --bin csp
```


## Dynamic Window Approach

<img src="./img/dwa.svg" width="640px">

Brack: Obstacles, Green: Trajectry, Yellow: Predected trajectry

- [src](https://github.com/rsasaki0109/RustRobotics/blob/master/src/bin/dwa.rs)

```
cargo run --bin dwa
```

## Model Predictive Trajectory Generator

<img src="./img/model_predictive_trajectory_generator.svg" width="640px">

Green: Path

- [src](https://github.com/rsasaki0109/RustRobotics/blob/master/src/bin/model_predictive_trajectory_generator.rs)


```
cargo run --bin model_predictive_trajectory_generator
```

## Dijkstra algorithm

<img src="./media/dijkstra-motion-planner.gif" width="640px">

- [src](./src/bin/dijkstra.rs)

## Potential Field algorithm
## State Lattice Planner
## Rapidly-Exploring Random Trees

# Path Tracking
## Move to Pose
<img src="./img/move_to_pose.svg" width="640px">

Green: Path, Red: Start and Goal

- [src](https://github.com/rsasaki0109/RustRobotics/blob/master/src/bin/move_to_pose.rs)

```
cargo run --bin move_to_pose
```

## Pure Pursuit

<img src="./img/pure_pursuit.svg" width="640px">

Brack: Planned path, Green: Tracked path

- [src](https://github.com/rsasaki0109/RustRobotics/blob/master/src/bin/pure_pursuit.rs)


```
cargo run --bin pure_pursuit
```

## Stanly Control

<img src="./img/stanley_control.svg" width="640px">

Brack: Planned path, Green: Tracked path

- [src](https://github.com/rsasaki0109/RustRobotics/blob/master/src/bin/stanley_controller.rs)


```
cargo run --bin stanley_controller
```


## LQR steer control

<img src="./img/lqr_steer_control.svg" width="640px">

Brack: Planned path, Green: Tracked path

- [src](https://github.com/rsasaki0109/RustRobotics/blob/master/src/bin/lqr_steer_control.rs)


```
cargo run --bin lqr_steer_control
```

## Nonlinear Model predictive control with C-GMRES


