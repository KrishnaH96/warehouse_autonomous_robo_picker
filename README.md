# ENPM808X-final-project-boilerplate

[![codecov](https://codecov.io/gh/KrishnaH96/warehouse_autonomous_robo_picker/branch/main/graph/badge.svg)](https://codecov.io/gh/KrishnaH96/warehouse_autonomous_robo_picker)

![CICD Workflow status](https://github.com/KrishnaH96/warehouse_autonomous_robo_picker/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg)

# TurtleBot based fork-lift

# Team Members:

- Krishna Hundekari

- Tej Kiran
  
- Abrarudin Syed

# Project Overview:

## Introduction
The "RoboPicker" prototype is designed to operate in controlled warehouse environments; it is an advanced system that has been carefully engineered for optimal operation. Its primary functions include picking up boxed items that have been arranged at random, navigating inside the warehouse autonomously, and carefully placing these items into designated empty spaces. With its ability to handle and place items with precision and adaptability, the "RoboPicker" redefines efficiency in warehouse logistics. It operates under the premise of a structured warehouse layout with clearly defined paths and drop-off zones.

## Purpose
We propose developing an autonomous warehouse collection robot using the simulated TurtleBot as its foundation. The robot will navigate a warehouse, targeting specific locations to retrieve marked boxes and transport them to predetermined positions. Fiducial markers on the boxes will contain crucial information about their designated destinations. Our goal is complete autonomy in stacking these boxes in their assigned positions within the warehouse. This solution aims to advance warehouse automation, improving efficiency in inventory management and streamlining order fulfillment. To emulate a real warehouse scenario, we will use forklifts for actuation purposes.


## Modules
The repository consists of following ROS2 packages:
 
  - *Simulation*: We will use Gazebo and a custom warehouse model to simulate the environment
  - *Modeling*: We will take the existing Turtle bot design and attach a fork list with prismatic join to the side of the robot using a dummy joint. We will also attach an RGB camera to the model for detecting the packages to be transferred.
  - *SLAM*: We will use the ROS2 slam tool box for generating the 2D map and localization of the robot.
  - *Navigation*: ROS2 Nav2 component will be integrated to the framework to perform autonomous navigation from start location to the end location.
  - *Pick & Place*: A custom package will be developed to orient robot and perform pick & place using the fork-lift


## Technologies and Libraries
The module will be developed using the following technologies:
- Programming Language: C++11/14
- Libraries: ROS2 Gazebo ( Apache v2.0 ), ROS2 Nav2 ( SPDX-ID - Various Licenses ), 
- ROS2 slam toolbox ( GPL v2.1 ).
- Build System: CMakes
- Testing Framework: Google Test
- Static Code Analysis: cppcheck

## Project Assumptions
The following assumptions are made for developing the project:
- The environment is a standard warehouse
- 2D Lidar is used for mapping and obstacle avoidance
- The model includes a forklift that is attached to a turtlebot using a dummy joint that simulates a physical fixed joint



## How to build and run demo

```bash
# rm -rf build/ install/
# colcon build 
# source install/setup.bash
# ros2 launch my_controller run_demo.launch.py
```

## How to build for tests (unit test and integration test)

```bash
rm -rf build/ install/
colcon build --cmake-args -DCOVERAGE=1 
```

## How to run tests (unit and integration)

```bash
source install/setup.bash
colcon test
```

## How to generate coverage reports after running colcon test

First make sure we have run the unit test already.

```bash
colcon test
```

### Test coverage report for `my_controller`:

``` bash
ros2 run my_controller generate_coverage_report.bash
open build/my_controller/test_coverage/index.html
```

### Test coverage report for `my_model`:

``` bash
colcon build \
       --event-handlers console_cohesion+ \
       --packages-select my_model \
       --cmake-target "test_coverage" \
       --cmake-arg -DUNIT_TEST_ALREADY_RAN=1
open build/my_model/test_coverage/index.html
```

### Combined test coverage report

``` bash
./do-tests.bash
```

## How to generate project documentation
``` bash
./do-docs.bash
```

## How to use GitHub CI to upload coverage report to Codecov

### First, sign up Codecov with you GitHub account.

  https://about.codecov.io/sign-up/

### Then, follow the similar instruction provided in the cpp-boilerplate-v2 repo

  https://github.com/TommyChangUMD/cpp-boilerplate-v2
