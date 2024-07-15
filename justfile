# build
build:
    cargo build

# ekf
ekf:
    cargo run --bin ekf

# bezier path
bezier_path:
    cargo run --bin bezier_path

# Cubic Spline
csp:
    cargo run --bin csp

# Dynamic Window Approach
dwa:
    cargo run --bin dwa

# Model Predictive Trajectory Generator
model_predictive_trajectory_generator:
    cargo run --bin model_predictive_trajectory_generator

# Dijkstra-motion-planner
dijkstra:
    @echo 'do remember to install gnuplot before run this recipe.'
    cargo run --bin dijkstra