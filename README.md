# game_theory_ws
A ROS workspace to implement complex, intelligent human-robot interaction behaviors in dynamic environments using game-theoretical approaches. This was initially created as a final project for ASE 389: Game Theoretical Modeling of Multiagent Systems, Fall 2022 at the University of Texas - Austin taught by [Dr. David Fridovich-Keil](https://clearoboticslab.github.io/).

This repository is intended to be a central hub for developing game-theoretical planning and control methods for my Ph.D. research. Specifically, I am interested in robots that operate in dynamic, human-centered environments. Game theoretical models are well-suited for these situations, since they inherently involve multiple agents (players) in collaborative and/or adversarial roles, and no two operating environments (game states) are the same.

As of December 2022, this project is organized as follows:
- This repository ([game_theory_ws](https://github.com/roboav8r/game_theory_ws)), which contains the project writeup, initial results and future plans, and installation instructions
- A Robot Operating System (ROS)-Gazebo simulator testbed ([hri_game_testbed](https://github.com/roboav8r/hri_game_testbed)), which serves as a controlled, repeatable development environment prior to deployment on robot hardware, and
- A ROS-enabled game-theoretical, hierarchical planner-controller module ([hierarchical_game_control_ros](https://github.com/roboav8r/hierarchical_game_control_ros)), which receives state variable observations from `hri_game_testbed`, computes an optimal course of action/control inputs, and sends them to the simulated robot.

These elements are discussed in further detail below. For collaboration or any questions/comments/concerns about this project, feel free to [contact me](mailto@john.a.duncan@utexas.edu) or [raise an issue](https://github.com/roboav8r/game_theory_ws/issues/new/choose)!

# Motivation and Hierarchical Control Approach
Robots and autonomous systems are expanding into a range of complex, dynamic, and populated operating environments. Novel service applications see robots operating in airports, hospitals, museums, city centers, and battlefields. However, robot action planning in such environments is inherently challenging: the correct course of action is situationally-dependent, or [*situated*](http://erichorvitz.com/naacl_directions_dialog.pdf), and depends on the robot's goals and capabilities in addition to the status of the environment and agents within it (i.e. humans and vehicles).

The motivating example for this project is *human-robot interaction* in such environments. New service applications will require robots to accomplish service tasks while acting alongside humans in various roles (teammates, pedestrians, adversaries). Potential service tasks include social navigation, cargo/parcel delivery, information sharing, and leading/following behaviors. Since these tasks involve a dynamic combination of multiple discrete and continuous elements, robots in dynamic environments require a planning and control architecture that can act optimally across a range of potential operating states and conditions.

To this end, this project employs a two-layer hierarchical planning & control architecture. This architecture could enable robots & autonomous systems to affect intelligent behaviors amidst uncertain or changing states. The architecture is directly motivated by the implementation in Thakkar *et al* "[Hierarchical Control for Cooperative Teams in Competitive Autonomous Racing](https://arxiv.org/abs/2204.13070)" in which the authors employ a high-level, low-frequency discrete planner with a low-level, high-frequency continuous controller (see Figure below).

![Thakkar et al's hierarchical control architecture](data/thakkar.png)

Specifically, this project implements a **Monte-Carlo Tree Search (MCTS)** to plan high-level robot behaviors, and a **Nonlinear Constrained Optimal Controller** to send motion control commands. These elements are discussed in detail in the following section.

# System Overview
The main system elements interact as shown in the diagram below.

![System Diagram](data/SystemDiagram.png)

This repository retains the testbed and hierarchical controller as submodules, while the hierarchical controller receives data from & sends commands to the testbed. Please see the following subsections for additional information on the testbed & hierarchical controller.

## hri_game_testbed
The testbed is implemented in ROS Gazebo and allows controlled, rapid development of the hierarchical planning & control architecture. It is intended to be a stand-in for real robot hardware until the hierarchical controller can safely be deployed on robot hardware. As such, it provides simulated sensor data and can accept robot motion commands via standard ROS message types.

At present, the only level is a simulated cargo pickup/dropoff task for a [PR2 robot](http://wiki.ros.org/Robots/PR2), taking place in an indoor hospital environment with known map. There are two variants, `worlds/hospital.world` and `worlds/hospital_empty.world`, which respectively do and do not contain humans.

![The Empty Hospital Gazebo world](data/gazebo.png)

The map can be launched by following the instructions in the [Usage](#usage) section and publishes relevant information about the PR2 on the following ROS topics:
```
/base_pose_ground_truth       # Robot odometry as a nav_msgs/Odometry message
/base_scan                    # Laser ranging data as a sensor_msgs/LaserScan message
```

The PR2 can be commanded to move by publishing a `geometry_msgs/Twist` message to the following topic:
```
/base_controller/command
```

## hierarchical_game_control_ros
The game theoretical, hierarchal planner-controller is implemented in the [hierarchal_game_control_ros](https://github.com/roboav8r/hierarchical_game_control_ros) package. The high-level planner and low-level controller work in tandem to interpret sensor measurements, select an optimal plan of action, and send commands to the (simulated) robot. At present the system is designed for a single task: cargo pickup and delivery. However, the software could be extended to work in additional applications (see [Future Work](#future-work)).

### lowlevel_controller_node

#### Inputs
The low level controller node has three inputs:
- The robot's current state $x$, comprised of its 2D position $[x_{robot}, y_{robot}]$, heading $\theta$, and velocity $v$. This information is received through a `nav_msgs/Odometry` ROS message on the `/base_pose_ground_truth` topic and necessary conversions are made by [robotOdomCallback in lowlevel_controller_node.h](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/include/lowlevel_controller_node.h#L54-L65).
- A 2D navigation goal $[x_{goal}, y_{goal}]$, in the map frame. $[x_{goal}, y_{goal}]$ is received as a `move_base_msgs/MoveBaseGoal` message on the `/nav_goal` topic.
- LiDAR range data from the laser scanner, received as a `sensor_msgs/LaserScan` ROS message on the `/base_scan` topic. The range measurements are converted from the robot's frame into the `map` frame and saved by the [scanCallback function in lowlevel_controller_node.h](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/include/lowlevel_controller_node.h#L67-L81).

The ROS subscribers are created in [lowlevel_controller_node.cpp](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/src/lowlevel_controller_node.cpp#L20-L29)

#### Outputs
Given the inputs above, the controller computes the optimal control output $\bar{u} = [\omega \\ a]^T$, where $\omega = \dot{\theta}$ is the yaw rate (rotation about the robot's $+z$ axis), and $a=\dot{v}$ is the linear acceleration (translation about the robot's $+x$ axis). The optimal control outputs are sent to the robot base as a `geometry_msgs/Twist` ROS message on the `/base_controller/command` topic by a [publisher in `lowlevel_controller_node.cpp`](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/src/lowlevel_controller_node.cpp#L18).

#### Algorithm
Uses IFOPT as a backend solver.
Constraints, variables, and objective functions
The controller node computes the control input 

given ...
the target state $\bar{x}$ comprised of robot position $[x_{robot}, y_{robot}]$ velocity $v$, and heading $\theta$ that minimizes the following cost functions:
$$J_{goal} = w_{goal}[(x_{robot} - x_{goal} + v_tcos(\theta_t)\Delta t)^2 + (y_{robot} - y_{goal} + v_tsin(\theta_t)\Delta t)^2]$$
$$J_{input} = w_{yaw}(\omega)^2 + w_{acc}(a)^2$$
$$J_{obst} = -\sum^{N_{obst}}_{i=1}w_{axial}log[cos\theta_t(x_{obst,i} - x_{robot}) + sin\theta_t(y_{obst,i} - y_{robot})]^2\\
-\sum^{N_{obst}}_{i=1}w_{lat}log[-sin\theta_t(x_{obst,i} - x_{robot}) + cos\theta_t(y_{obst,i} - y_{robot})]^2
$$

Subject to the following constraints:
$$
C_{dyn}: \begin{bmatrix} 
x_{robot}'\\
y_{robot}'\\
\theta'\\
v'\\
\end{bmatrix} = 
\begin{bmatrix} 
x_{robot}^0 + \Delta t v'cos\theta'\\
y_{robot}^0 + \Delta t v'sin\theta'\\
\theta^0 + \omega \Delta t\\
v^0 + a\Delta t\\
\end{bmatrix} \\
C_{state}: v_t \in [v_{min},v_{max}], \theta_t \in [-\pi,\pi]\\
C_{input}: \omega \in [\omega_{min},\omega_{max}], a \in [a_{min},a_{max}] \\
$$


The cost functions have the following Jacobians, taken with respect to each target state variable:
$$
{\partial J_{goal} \over \partial \bar{x}} = 
\begin{bmatrix} 
{\partial J_{goal}} / {\partial x_{robot}}  \\
{\partial J_{goal}} / {\partial y_{robot}} \\
{\partial J_{goal}} / {\partial \theta_t} \\
{\partial J_{goal}} / {\partial v_t} \\
\end{bmatrix} = 
\begin{bmatrix}
2w_{goal}(x_{robot} - x_{goal} + v_tcos\theta_t\Delta t) \\
2w_{goal}(y_{robot} - y_{goal} + v_tsin\theta_t\Delta t) \\
2w_{goal}v_t\Delta t[(x_{robot} - x_{goal})sin\theta_t + (y_{robot} - y_{goal})cos\theta_t] \\
2w_{goal}\Delta t[v_t\Delta t + (x_{robot} - x_{goal})cos\theta_t + (y_{robot} - y_{goal})sin\theta_t] \\
\end{bmatrix}\\
{\partial J_{acc} \over \partial \bar{x}} = 
\begin{bmatrix} 
{\partial J_{acc}} / {\partial x_{robot}}  \\
{\partial J_{acc}} / {\partial y_{robot}} \\
{\partial J_{acc}} / {\partial \theta_t} \\
{\partial J_{acc}} / {\partial v_t} \\
\end{bmatrix} = 
\begin{bmatrix}
TODO
\end{bmatrix}
$$

Note that obstacle positions $[x_{obst,i}, y_{obst,i}]$, robot state variables $[x_{robot}, y_{robot}]$, and heading $\theta$ are in the map/global frame, not the robot frame.

### highlevel_planner_node

# Initial Results

# Closing Thoughts

# Usage (optional, if running the code is desired)

## Setup and installation 
This assumes you have ROS Noetic installed and are using Ubuntu 20.04.

First, clone the repository and its submodules. At a terminal in your home directory (`~`):
```
git clone --recursive TODO
```

Install ROS package dependencies. In the `game_theory_ws` directory at a terminal:
```
rosdep install --from-paths . --ignore-src -r -y
```

Set up environmental variables for Gazebo. At a terminal:
```
cd ~/game_theory_ws/src/hri_game_testbed
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`pwd`/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:`pwd`/worlds
```

# Running the Hierarchical Controller Demonstration
```
roslaunch hierarchical_game_control_ros hospital_demo.launch                    # For an empty hospital environment
roslaunch hierarchical_game_control_ros hospital_demo.launch level:=hospital    # For a populated environment
```
# Future Work
If you have an idea for an autonomous robot application or are interested in collaborating, please raise an issue or send me an e-mail.


## Potential Improvements
As of December 2022, this repo is meant to be a proof of concept and has many areas for improvement. 

Code/implementation improvements:
- Launch localization and mapping; use actual ROS data instead of using the map as an OpenCV matrix
- Implement the optimal controller as a `ros_control` interface/controller
- Separate `graph_datatypes` from the `highlevel_planner_node` and make it more generic and modular; currently specific to the cargo pickup/dropoff problem
- Update access modifiers in the hl, ll, and mcts classes; everything is currently public

Algorithm improvements:
- Instead of planning only the next waypoint, plan an entire motion path
