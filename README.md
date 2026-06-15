# pairs_precise_landing

Vision-guided precise landing for the PAIRS UAV stack. The UAV detects an
AprilTag landing pad, estimates its pose, and drives an autonomous descent that
aligns the aircraft over the pad and touches down on it. This repository holds
the landing controller, the landing-pad pose estimator, and the Gazebo
simulation resources used to test the behaviour.

## Contents

This repository provides three packages:
- `pairs_precise_landing` — the landing state machine/controller; ships the `pairs_precise_landing::PreciseLanding` composable node and a `land`/`abort` service interface.
- `pairs_landing_pad_estimation` — estimates the landing-pad pose from AprilTag detections; ships the `pairs_landing_pad_estimation::LandingPadEstimation` composable node.
- `pairs_precise_landing_gazebo` — Gazebo models (recursive AprilTag marker) and a world for simulating precise landings.

## Branches
- `ros1` — ROS 1 Noetic (catkin)
- `ros2` — ROS 2 Jazzy (ament_cmake)

## Install (ROS 2 Jazzy)
```bash
sudo apt install ros-jazzy-pairs-precise-landing
```

## Usage

Run the full Gazebo demo via the bundled tmux session:

```bash
cd ros_packages/pairs_precise_landing_gazebo/tmux && ./start.sh
```

## License
BSD 3-Clause. Derived from the CTU-MRS `pairs_precise_landing` package; the original
copyright is retained in [LICENSE](LICENSE).
