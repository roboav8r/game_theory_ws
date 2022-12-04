# game_theory_ws
A ROS workspace to implement complex, intelligent human-robot interaction behaviors in dynamic environments using game-theoretical approaches. This was initially created as a final project for ASE 389: Game Theoretical Modeling of Multiagent Systems, Fall 2022 at the University of Texas - Austin taught by David Fridovich-Keil [TODO link].

# Motivation and Hierarchical Control Approach
Robots and autonomous systems are expanding into a range of complex, dynamic, and populated operating environments. Novel service applications see robots operating in airports, hospitals, museums, city centers, and battlefields. However, robot action planning in such environments is inherently challenging: the correct course of action is *situated*[TODO Bohus paper], and depends on the robot's goals and capabilities, and the status of the environment and agents within it (i.e. humans and vehicles).

The motivating example for this project is *human-robot interaction* in such environments. New service applications will require robots to accomplish service tasks while acting alongside humans in various roles (teammates, pedestrians, adversaries). Potential service tasks include social navigation, cargo/parcel delivery, information sharing, and leading/following behaviors. Since these tasks involve a dynamic combination of multiple discrete and continuous elements, robots in dynamic environments require a planning and control architecture that can act optimally across a range of potential operating states and conditions.

To this end, this project employs a two-layer hierarchical planning & control architecture. This architecture can enable robots & autonomous systems to affect intelligent behaviors amidst uncertain or changing states. The architecture is motivated by the implementation in Thakkar *et al* "[Hierarchical Control for Cooperative Teams in Competitive Autonomous Racing](https://arxiv.org/abs/2204.13070)" in which the authors employ a high-level, low-frequency discrete planner with a low-level, high-frequency continuous controller (see Figure below).

![Thakkar et al's hierarchical control architecture](data/thakkar.png)

Specifically, this project implements a **Monte-Carlo Tree Search (MCTS)** as a high-level planner, and a **Constrained Optimal Controller** for the low-level motion controller. These elements are discussed in detail in the following section.

# Repo overview
This is the main workspace, a meta-repo that contains all necessary dependencies as submodules. `game_theory_ws` is a Robot Operating System workspace, intended to have all the elements needed to demonstrate a hierarchical controller on a (simulated) robot. At present, the only level is a simulated cargo pickup/dropoff task, taking place in an indoor hospital environment with known map.

## hri_game_testbed
The testbed uses a 

## hierarchical_game_control_ros

### lowlevel_controller_node

### highlevel_planner_node

### game_manager_node

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

# Collaboration and Future Work
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

