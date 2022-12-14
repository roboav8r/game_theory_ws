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
Given the inputs above, the controller computes the optimal control output $u = [\omega \\ a]^T$, where $\omega = \dot{\theta}$ is the yaw rate (rotation about the robot's $+z$ axis), and $a=\dot{v}$ is the linear acceleration (translation about the robot's $+x$ axis). The optimal control outputs are sent to the robot base as a `geometry_msgs/Twist` ROS message on the `/base_controller/command` topic by a [publisher in `lowlevel_controller_node.cpp`](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/src/lowlevel_controller_node.cpp#L18).

#### Parameters
The controller is parametrized by cost weights $w_{i}$ as described in the Algorithm section.

#### Algorithm
The low level controller models the control problem as a nonlinear program comprised of a **cost function** to be minimized by adjusting **variables** subject to **constraints**. [Optimizer Interface (IFOPT)](https://github.com/ethz-adrl/ifopt), a C++/Eigen-based interface to solvers such as IPOPT and SNOPT, then solves the constrained optimization problem.

The low level controller minimizes the following cost functions:
```math
J_{goal} = w_{goal}[(x_{robot} - x_{goal} + v_tcos(\theta_t)\Delta t)^2 + (y_{robot} - y_{goal} + v_tsin(\theta_t)\Delta t)^2]
```
```math
J_{input} = w_{yaw}(\omega)^2 + w_{acc}(a)^2\\
```
```math
J_{obst} = -\sum^{N_{obst}}_{i=1}w_{axial}log[cos\theta_t(x_{obst,i} - x_{robot}) + sin\theta_t(y_{obst,i} - y_{robot})]^2\\

-\sum^{N_{obst}}_{i=1}w_{lat}log[-sin\theta_t(x_{obst,i} - x_{robot}) + cos\theta_t(y_{obst,i} - y_{robot})]^2\\
```
which penalize distance from the navigation goal, large control inputs, and proximity to obstacles, respectively. This cost structure is derived from Lasse Peters _et al_, ["Inferring Objectives in Continuous Dynamic Games from Noise-Corrupted Partial State Observations"](https://arxiv.org/pdf/2106.03611.pdf), 
who use a similar cost structure for an autonomous driving scenario. Notably, the obstacle cost used in this controller is modified; the $N_{obst}$ laser range measurements from the LiDAR scan are converted into obstacle position measurements in the _map_ frame $[x_{obst,i}, y_{obst,i}]$, and separated into lateral and axial components in the _robot_ frame. This is to enable different weights for lateral and axial obstacles. The cost functions are implemented in a separate header file, [`solver_params.h`](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/include/solver_params.h#L351-L526), which is included in the controller node.

To optimize the cost functions, the solver adjusts the controller inputs $\bar{u}$ and the desired state variables $\bar{x}$:

$$
\bar{u} = \left[ \begin{array}{c}
    \omega' \\
    a'
  \end{array} \right]; \\
\bar{x} = \left[ \begin{array}{c}
    x_{robot}' \\
    y_{robot}' \\
    \theta' \\
    v'
  \end{array} \right]
$$

where ${\bar{\cdot}}$ notation denotes a variable set to solve, and ${\cdot}'$ denotes an individual variable to be solved. Both variable sets are defined in [the variables section of `solver_params.h`](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/include/solver_params.h#L8-L100).

Subject to the following dynamics constraints and variable limits:

$$
C_{dyn}: \left[ \begin{array}{c}
    x_{robot}' \\
    y_{robot}' \\
    \theta' \\
    v'
  \end{array} \right] = 
  \left[ \begin{array}{c}
    x_{robot}^0 + \Delta t v'cos\theta'\\
    y_{robot}^0 + \Delta t v'sin\theta'\\
    \theta^0 + \omega' \Delta t\\
    v^0 + a'\Delta t\\
  \end{array} \right];\\
$$

$$
  C_{state}: v_t \in \left[v_{min},v_{max}\right], \theta_t \in \left[-\pi,\pi\right];\\
$$

$$
  C_{input}: \omega' \in \left[\omega_{min},\omega_{max}\right], a' \in \left[a_{min},a_{max}\right];\\
$$


where ${\cdot}^0$ notation denotes a _current_ value of the state variable. Dynamics constraints are defined in the [constraints section of `solver_params.h`](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/include/solver_params.h#L105-L215), while state and input variable limits are part of their respective [variable set entries](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/include/solver_params.h#L8-L100). Additionally, `solver_params.h` includes analytical Jacobians for the cost and dynamics functions. These were derived manually and included to improve real-time solver performance, but are not strictly necessary.

Lastly, these constraints are added to the [`lowlevel_controller_node.cpp`](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/src/lowlevel_controller_node.cpp#L46-L72) and run in a continuous, high-frequency loop. During each iteration, [current state variables are updated with the most recent ROS state data](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/src/lowlevel_controller_node.cpp#L89-L91) and the solver object computes a solution to the nonlinear program, publishing it to the robot testbed.

### highlevel_planner_node
The planner node is a ROS node which, given the current state of the environment (game) and robot agent (player), computes the player's optimal move. The planner node then sends the action to the robot controller, as appropriate. The planner logic is implemented in a separate header file (`graph_datatypes.h`), which contains the Monte Carlo Tree Search implementation and associated data structures (`GameState`/node and `Tree`). The main ROS node is implemented in `highlevel_planner_node.cpp` and `highlevel_planner_node.h`. Since the project is currently simulated, the ROS node also handles some game logic at an abstract level. For example, this node sets boolean values to denote if the robot is carrying cargo rather than do a dynamic simulation of the action.

#### Inputs
The planner node's inputs consist of the variables that define the state of the robot and cargo within the environment:
- The robot's current 2D position $[x_{robot}, y_{robot}]$. This information is received through a `nav_msgs/Odometry` ROS message on the `/base_pose_ground_truth` topic and necessary conversions are made by [robotOdomCallback in highlevel_planner_node.h](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/include/highlevel_planner_node.h#L102-L112).
- The 2D location where the cargo will be picked up, $[x_{cargo}, y_{cargo}]$. Currently hardcoded in [`highlevel_planner_node.h`](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/include/highlevel_planner_node.h#L55-L56) but should eventually become an input.
- The 2D location where the cargo will be delivered, $[x_{dest}, y_{dest}]$. Currently hardcoded in [`highlevel_planner_node.h`](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/include/highlevel_planner_node.h#L59-L60) but should eventually become an input.
- Boolean variables depicting the current location of the cargo: $robotHasCargo, cargoAtPickup, cargoAtDest$. Currently hardcoded in [`highlevel_planner_node.h`](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/include/highlevel_planner_node.h#L52-L54) but should eventually become an input.

#### Outputs
The planner node publishes the optimal robot action after running MCTS. Actions can be output in one of two ways:
- Navigation goals $[x_{goal}, y_{goal}]$ are published as a `move_base_msgs/MoveBaseGoal` message on the `/nav_goal` topic by a [publisher in `highlevel_planner_node.cpp`](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/src/highlevel_planner_node.cpp#L25), in the `map` frame. 
- Actions such as `PickUp` or `DropOff` are set as variables and processed internally by `highlevel_planner_node.cpp`. Since these capabilities are handled abstractly (i.e. no pickup or drop off actions are simulated in the testbed), these actions can be replaced by actual movement commands in hardware implementations of the planner.
- To help interpret the planner's intent during development, the planner also publishes an OpenCV image containing the map, current state belief, and the action selected.

#### Parameters
- A map of the operating environment. By default, the planner loads the map on startup and converts it to an OpenCV matrix for easier processing. The map is an occupancy grid, obtained by a mapping run of the environment and saved for later use. Sample maps of the hospital level are provided in `hri_game_testbed/maps/*.pgm`. `clean_hospital_map` is used by default.
- ROS map parameters: the map origin location, free/occupied threshold values, grid resolution are provided in `hri_game_testbed/maps/*.yaml` and enable the spatial and occupancy information to be properly interpreted. The parameters are discussed [on the ROS wiki](http://wiki.ros.org/map_server).
- [Solver time $dt$](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/include/highlevel_planner_node.h#L36): Amount of time between the ROS timer call/amount of time to allow the Monte Carlo Tree Search to run. Currently set to 3 seconds.
- Cargo distance: the threshold beneath which the robot is considered close enough to pick up or drop off cargo from the map.
- `movementStepSize`: how much the planner advances the robot's position during a "move" action. Currently 1 meter. 

#### Algorithm
`highlevel_planner_node` is the main ROS node which runs the planner in a low-frequency loop (currently $\Delta t= 3s$). During each iteration, `highlevel_planner_node`:
- Creates a new `GameState` node using the current state information.
- Creates a new `Tree` with the current state as root.
- Runs a search on the `Tree` object. 
- Renders a visual representation of the map, current state, and optimal move in an OpenCV display window.
These steps are implemented in the main loop in [`highlevel_planner_node.cpp`](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/src/highlevel_planner_node.cpp#L71-L77).

The MCTS algorithm and solver-specific datatypes are included in `include/graph_datatypes.h`:

##### GameState (Node) Variables
The game state is completely characterized by:
- Pixel indices of the robot, cargo pickup site, and dropoff site. This includes the `robotXPix`,`robotYPix`,`pickupXPix`,`pickupYPix`, `destXPix`, and `destYPix` variables. Note that these positions are converted between pixel indices in the OpenCV occupancy grid and $[x,y]$ positions in the `map` frame.
- Current location of the cargo. This is defined by the boolean variables `robotHasCargo`, `cargoAtPickup`, and `cargoAtDest`.

Each `GameState` node must also retain the following information for MCTS:
- `nTimesVisited`: The number of times the node has been simulated during the tree search.
- `score`: The total objective result from all simulations at the node.
- `parent`: The `GameState` which preceded the current state.

##### GameState Available Moves
When a `GameState` node is initialized, the [`getAvailMoves()` subroutine](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/include/graph_datatypes.h#L135-L217) computes the available actions (moves) as follows:
- If the robot is within `cargoDist` of the cargo pickup location and the pickup location has the cargo, the robot can pick up cargo.
- If the robot is within `cargoDist` of the cargo delivery destination and the robot has the cargo, the robot can drop off cargo.
- To check for movement availability, pixels along the `PlusX`, `PlusY`, `MinusX`, `MinusY` directions in the `map` frame are checked to ensure they are below the `freeThreshold`. If each pixel value between the robot's current position and the `movementStepSize` is free, this is a valid movement action.

##### GameState Transition Logic
The function [`propagate(Move)`](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/include/graph_datatypes.h#L306-L343) accepts a Move as input and advances the GameState as follows:
- Movement Moves (`PlusX`, `PlusY`,..) increment the robot's position (`robotXPix`,`robotYPix`) appropriately.
- `PickUp` and `DropOff` moves set the cargo boolean variables `robotHasCargo`, `cargoAtPickup`, and `cargoAtDest` appropriately.

`propagate(Move)` is also used during MCTS simulation.

##### Terminal Conditions
The following terminal conditions are defined in [the `gameResult` function](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/include/graph_datatypes.h#L345-L359):
- Cargo successfully arriving at the destination results in a **win and a score of +1**.
- If the simulated game executes `maxNumMoves=50` moves without delivering the cargo, this results in a **loss and a score of 0**.

If the game state is not terminal, a value of `-1` is returned to continue simulation.

##### MCTS Algorithm
The MCTS implementation is a straightforward implementation of the original algorithm by Coulom _et al_, "Efficient Selectivity and Backup Operators in Monte-Carlo Tree Search" (2007), and uses the Upper Confidence bounds applied to Trees (UCT) algorithm devised by Koscis & Szepesv??ri in "Bandit based Monte-Carlo Planning" (2006). For this project, MCTS is implemented in [`Tree.Search()`](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/include/graph_datatypes.h#L381-L461). The steps are:

- [**Initialization**](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/include/graph_datatypes.h#L383-L385): The search begins at the current state, which is initially set as the root of the tree.
- [**Selection**](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/include/graph_datatypes.h#L389-L393): The current node selects a child node with the **highest** value according to the UCT formula until a leaf is found. This is implemented in [`GameState.selectNextNode()`](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/include/graph_datatypes.h#L250-L279) and computes children's UCT values using [`GameState.computeChildUctValue`](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/include/graph_datatypes.h#L225-L236). In the event that multiple child nodes have equal value, a random child node is selected from the highest valued children. 

Recall that a leaf is a node with unexplored child nodes, computed by the helper function [`GameState.isLeaf()`](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/include/graph_datatypes.h#L220-L223), and the UCT formula computes the child node value as follows:

$$
    v_i = \frac{w_i}{n_i} + c\sqrt(\frac{log(N)}{n_i})
$$

where $w_i$ is the child node's `score`, $n_i$ is the child node's `nTimesVisited`, $N$ is the parent node's `nTimesVisited`, and $c$ is an exploration constant = $\sqrt(2)$.

- [**Expansion**](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/include/graph_datatypes.h#L395-L414): Once a leaf is found, a child node is added to the leaf. A valid, unexecuted `Move` is selected. A [specialized `GameState` constructor](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/include/graph_datatypes.h#L67-L90) creates the child node from the `GameState` parent and the selected `Move`.
- [**Simulate**](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/include/graph_datatypes.h#L427-L437): Random, uniform moves are selected from the child's set of available moves until a terminal state is reached. When a terminal state is reached, the simulate step add's the resulting score to the child node's `score` and increments the child node's `nTimesVisited`.
- [**Backpropagate**](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/include/graph_datatypes.h#L440-L455): The search moves to successive parent nodes, updating `score` and incrementing `nTimesVisited` for each node until the search reaches the root.

The search starts again at the selection step until the MCTS exceeds planning time $\Delta t = 3s$. Then, the best known result is saved as `Tree.bestMove`. To exploit the best known move, [`Tree.selectBestMove()`](https://github.com/roboav8r/hierarchical_game_control_ros/blob/99c2929aae19267068c40bffcc398295d80bbd75/include/graph_datatypes.h#L281-L304) selects the root's child with the highest value

$$
    v_i = \frac{w_i}{n_i}
$$


# Results
Qualitative results and observations from the initial system demonstration are provided here.

## Low Level Controller
During initial trials, the low level controller took between **10-40 ms** to solve the nonlinear control program, corresponding to a rate of 25-100 Hz. 
![Static output of the Low Level Controller](data/solver.png)
![Animated output of the Low Level Controller](data/ll_solver.gif)

The system is able to successfully navigate to a specified waypoint goal using the included objective functions:

![Animated output of the Low Level Controller Navigating to a Goal](data/nav_success.gif)

When a navigation goal _behind_ the robot is supplied, the robot moves backwards toward the goal, instead of rotating to face it and then moving forward:

![Animated output of the Low Level Controller Navigating to a Goal](data/nav_backwards.gif)

When the robot moves into a more congested region of the map, the solver converges; however, the optimal solution is not the target goal waypoint. The solution seems to place the robot into a static configuration where it is unable to exit and make progress toward the goal:

![Animated output of the Low Level Controller Navigating to a Goal](data/obst_avoid.gif)

## High Level Planner
During initial trials, the MCTS planner **simulated a total of 50,000 - 100,000 nodes during the 3 second planning period**. Typical scores at the root node ranged from **0-50**.

![Static output of the Planner](data/planner.png)

When integrated with the simulator and controller, the planner is able to compute and send commands to the system effectively. The simulated robot's controller responds to the planner's waypoints. The planner does not appear to converge on a single move, and instead appears to issue commands inconsistently and in opposition to one another. For example, in the animation below, the planner commands the robot back and forth, resulting in minimal progress toward picking up or dropping off cargo.

![Animated output of the Low Level Controller Navigating to a Goal](data/integrated_demo.gif)

# Interpretation of Results & Future Work

The initial results of this project provided valuable feedback toward the system's design and guide further development. Specific conclusions and future work include:
- **The planner-controller system is feasible for real-time usage**. Both the controller and planner could run continuously and perform a suitable number of computations for real-time usage. A mobile robot platform possesses the computational resources required to run the controller-planner. 
- **Modifications to the cost functions in the optimal controller**. The robot's motion displayed two undesirable characteristics: it backed toward goals instead of rotating to drive forwards toward them; and the robot base stopped when entering a congested area. This suggests that the controller has converged onto a solution that is locally optimal, but that does not reflect desired motion. To address these issues, a penalty could be implemented for rearward motion. The vehicle obstacle avoidance model from [Peters _et al_](https://arxiv.org/pdf/2106.03611.pdf) likely needs to be replaced with an obstacle avoidance model designed for a robot with LiDAR scanning rangefinder.
- **Modifications to the MCTS planner**. The planner failed to converge on a single solution during planning runs, and published seemingly contradictory actions. This could be a result of a very sparse reward structure in which the `score` is much less than the total number of simulations, `nVisited`. Specifically:

$$ 
    w_i << N \implies \frac{w_i}{n_i} << c\sqrt(\frac{log(N)}{n_i})
$$

The exploration term in the UCT value is much larger than the exploitation term, which could explain the seemingly random behavior in the planner. To address the issue, modifications to the rewards, actions, and search algorithms should be explored. Namely:
- The objective score from a win can be increased 
- The exploration constant $c$ can be adjusted
- The search can be guided towards known, valuable actions using a heuristic function (as in the $A*$ search), or using a model for the value
- Instead of planning the next action at each timestep, an optimal _sequence_ of actions could be planned upon request. This may be more suitable 

In conclusion, although the system did not affect the desired series of robot actions, a game-theoretical approach to robot action planning and execution is feasible. Modifications to the planning and controller algorithms implemented in this project could yield desirable behaviors for robots in dynamic environments.

## Potential Improvements
As of December 2022, this repo is meant to be a proof of concept and development environment. Based on the initial project results, potential improvements to the project include:

Controller Algorithm improvements:
- Explore different obstacle avoidance models using LiDAR
- Add penalty for rearward motion

Planner Algorithm improvements:
- Experiment with increasing score for wins
- Add incremental reward/score, e.g. an intermediate goal for picking up cargo
- Guide the search using a heuristic function or model
- Reduce the number of planning steps and intermediate goals by planning entire navigation segments, instead of the next action

Code/implementation improvements:
- Launch localization and mapping; use actual ROS data instead of using the map as an OpenCV matrix
- Implement the optimal controller as a `ros_control` interface/controller
- Separate `graph_datatypes` from the `highlevel_planner_node` and make it more generic and modular; currently specific to the cargo pickup/dropoff problem
- Update access modifiers in the hl, ll, and mcts classes; everything is currently public

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

Build the workspace. At a terminal:
```
cd ~/game_theory_ws
catkin_make # or catkin build
```

## Running the Hierarchical Controller Demonstration
Ensure the workspace is properly sourced, and then run the demo using the following commands:
```
cd ~/game_theory_ws
source devel/setup.bash
roslaunch hierarchical_game_control_ros hospital_demo.launch                    # For an empty hospital environment
roslaunch hierarchical_game_control_ros hospital_demo.launch level:=hospital    # For a populated environment
```
