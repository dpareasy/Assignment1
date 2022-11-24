# Assignment1

Parisi Davide Leo 4329668 

## Introduction ##

This repository contains ROS-based software, developed in python language, that simulates a behavioural architecture. The objective is to drive a robot around a particular environment created by manipulations service requests to aRMOR server. In order to not make the transitions between states of the state machine istantaneous, a dummy simulated behavior is implemented using mutexes.


## About the simulation ##

### Environment ###

The ontology used in this repository encodes the classes shown in the picture below, where each `LOCATION` can be a `ROOM`, if it has only one `DOOR`, and a `CORRIDOR`, if it has more doors. Each door is associated with a location through the object property `hasDoor`. In addition, each `LOCATION` has the data property `visitedAt`, which represents the more recent timestamp (in seconds) when the robot visited such a location (see figure below). The `ROBOT` class contains only one individual which has several properties like the `isIn` which represent the robot actual position and the `now` property which specify the last time the robot changed its location.

### Scenario ###

The scenario involves a surveilling robot deployed in a indoor environmnet. It's objective is to visit different locations and stay there for some times. It must wait for receiving all the information to build the topological map before starting surveilling. The robot should start in it's initial position which is also the recharging location. When it moves in a new location it should wait some times before starting reasoning for moving in another location.
When its battery is low it should move to the recharging position before starting again the above mentioned behavior.

### Assumptions ###

For simplicity we consider a scenario with the following assumptions:
* The environment created can be formed by any number of corridors;
* The number of rooms which a corridor can contain is the same for each corridor;
* Each i-th corridor is connected with (i-th - 1) and with the recharging location (which is a corridor);
* The duration of the battery is a random value between 40 and 60 seconds;
* The battery can become low at any time;
* The robot is automatically spowned in the recharging room every time the battery goes low;


### Requirements ###

The moving policy that the robot should follow is the one presented below:
* It should mainly stay on corridors;
* If a reachable room has not been visited for some specified amount of times it should visit it;

## Project structure ##

### Package list ###

This repository contains a ROS package named Assignment1 that includes the following resources:
1. `CMakeList.txt`: File to configure this package;
2. `package.xml`: File to configure this package;
3. `launch/`: Contains the configuration to launch this package;
    * launch_file.launch: it launches armor server and all the nodes used in this simulation.
4. `msg/`: It contains the message exchanged through ROS topics:
    * Point.msg: It is the message representing a 2D point.
5. `srv/`: It Contains the definition of each server used by this software:
    * GetPose.srv: It defines the request and response to get the current robot position;
    * SetPose.srv: It defines the request and response to set the current robot position.
6. `action/`: It contains the definition of each action server used by this software:
    * Plan.action: It defines the goal, feedback and results concerning motion planning;
    * Control.action: It defines the goal, feedback and results concerning motion controlling.
7. `scripts/`: It contains the implementation of each software components:
    * load_ontology.py: It creates the topological map of the environment;
    * my_state_machine.py: It defines the states of the state machine;
    * robot_state.py: It implements the robot state including: current position, and battery level;
    * planner.py: It is a dummy implementation of a motion planner;
    * controller.py: It is a dummy implementation of a motion controller.

## Launching the software ##

This software is based on ROS Noetic.

### Installation ###

Follow these steps to install the software:
* Clone this repository inside your workspace (make sure it is sourced in you .bashrc);
* Clone arch_skeleton repository which contains the controller and planner server; 
* Run `chmod +x <file_name>` for each file inside the scripts folder;
* Run `catkin_make` from the root of your workspace.

### Launcher ###

In order to launch the simulation a .lunch file can be used bay using this command into the terminal:
```
roslaunch Assignment1 launch_file.launch
```
It will launch the aRMOR server and all the nodes that implement the robot behavior. Install xterm to visualize feedback from the nodes launched.


### ROS Parameters

This software requires the following ROS parameters.
 
 - `config/environment_size`: It represents the environment boundaries as a list of two float numbers, i.e., `[x_max, y_max]`. The environment will have the `x`-th coordinate spanning in the interval `[0, x_max)`, while the `y`-th coordinate in `[0, y_max)`.

 - `config/user_pose`: It represents the user's position as a list of two float numbers, i.e., `[x, y]`. This pose should be within the `environmet_size`.

 - `state/initial_pose`: It represents the initial robot pose as a list of two float numbers, i.e., `[x, y]`. This pose should be within the environmet_size`.

 - `test/random_plan_points`: It represents the number of via points in a plan, and it should be a list of two integer numbers `[min_n, max_n]`. A random value within such an interval will be chosen to simulate plans of different lengths.

 - `test/random_plan_time`: It represents the time required to compute the next via point of the plan, and it should be a list of two float numbers, i.e., `[min_time, max_time]` in seconds. A random value within such an interval will be chosen to simulate the time required to compute the next via points.

 - `test/random_motion_time`: It represents the time required to reach the next via point, and it should be a list of two float numbers, i.e., `[min_time, max_time]` in seconds. A random value within such an interval will be chosen to simulate the time required to reach the next via points. 

 - `test/random_sense/active`: It is a boolean value that activates (i.e., `True`) or deactivates (`False`) the random-based generation of stimulus (i.e., speech, gesture and battery level). If this parameter is `True`, then the three parameters below are also required.  If it is `False`, then the three parameters below are not used.
 

In addition, the `random_sense.launch` also requires the following three parameters. This occurs because `test/random_sense/active` has been set to `True`.
 
 - `test/random_sense/battery_time`: It indicates the time passed within battery state changes (i.e., low/high). It should be a list of two float numbers, i.e., `[min_time, max_time]` in seconds, and the time passed between changes in battery levels will be a random value within such an interval.
