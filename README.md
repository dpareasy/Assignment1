# Assignment1

Parisi Davide Leo 4329668 

## Introduction ##


This repository contains ROS-based software, developed in python language, that simulates a behavioural architecture. The objective is to drive a robot around a particular indoor environment created by manipulations service requests to aRMOR server. 

## About the simulation ##

### Ontology ###

The ontology used in this repository encodes the classes shown in the picture below, where each `LOCATION` can be a `ROOM`, if it has only one `DOOR`, and a `CORRIDOR`, if it has more doors. Each door is associated with a location through the object property `hasDoor`. In addition, each `LOCATION` has the data property `visitedAt`, which represents the more recent timestamp (in seconds) when the robot visited such a location (see figure below). The `ROBOT` class contains only one individual which has several properties like the `isIn` which represent the robot actual position and the `now` property which specify the last time the robot changed its location. If a location has not been visited for a specified amount of time it becomes part of the `URGENT`class.

Visit https://github.com/buoncubi/topological_map for more details.

### Scenario ###

The scenario involves a surveying robot deployed in a indoor environmnet. It's objective is to visit different locations and stay there for some times. Before starting moving around the map it must wait for receiving all the information to build the topological map. The robot should start in it's initial position which is also the recharging location. Any time it  enters a room, the robot should check it for some times before starting reasoning again to choose the next location to visit.

The environment in which the robot moves is developed in a way in which different scenarios can be created even if only under certain assumptions which are presented in the "Assumptions" section.

### Requirements ###

The moving policy that the robot should follow is the one presented below:
* It should mainly stay on corridors;
* If a reachable room become `URGENT` the robot should visit it;
* When its battery is low it should move to the recharging position;

### Assumptions ###

For simplicity we consider a scenario with the following assumptions:
* The environment created can be formed by any number of corridors;
* The number of rooms which a corridor can contain is the same for each corridor;
* Corridor(i) is only connected to corridor(i+1) and to the recharging location (which is a corridor);
* The duration of the robot's battery is 60 seconds;
* The robot is automatically spawned in the recharging room every time the battery goes low;
* The corridors' timestamps `visitedAt` are not considered since their urgency requirement are different from rooms';
* If there are no urgent rooms the robot is forced to move around corridors;

## System limitations ##

As stated in the above section, some assumptions have been made to simplify the development of the simulation. First of all, when the battery goes down the robot is automatically spowned in the recarging room instead of searching for the best path which connects it to the recharging site. Moreover, it is not possible to create any type of environment, indeed the number of rooms is strictly related to the number of corridors, since the main hypothesis made is that the number of rooms is equal for each corridor.

## Possible improvements ##


## Project structure ##

### Package list ###

This repository contains a ROS package named Assignment1 that includes the following resources:
1. `CMakeList.txt`: File to configure this package;
2. `package.xml`: File to configure this package;
3. `launch/`: Contains the configuration to launch this package;
    * launch_file.launch: it launches armor server and all the nodes used in this simulation.
4. `msg/`: It contains the message exchanged through ROS topics
    * Point.msg: It is the message representing a 2D point.
5. `srv/`: It Contains the definition of each server used by this software:
    * GetPose.srv: It defines the request and response to get the current robot position;
    * SetPose.srv: It defines the request and response to set the current robot position.
6. `action/`: It contains the definition of each action server used by this software:
    * Plan.action: It defines the goal, feedback and results concerning motion planning;
    * Control.action: It defines the goal, feedback and results concerning motion controlling.
7. `scripts/`: It contains the implementation of each software components:
    * load_ontology.py: It creates the topological map of the environment;
    * robot_actions.py: It contains a class to implement the behavior of the robot;
    * my_state_machine.py: It defines the states of the state machine;
    * robot_state.py: It implements the robot state including: current position, and battery level;

## Launching the software ##

This software is based on ROS Noetic.

### Installation ###

Follow these steps to install the software:
* Clone this repository inside your workspace (make sure it is sourced in you .bashrc);
* Follow the stpes of this link for aRMOR installation;
* Use armor_api for server requests, you can clone it from this repository;
* Clone inside your workspace the repository in this link https://github.com/buoncubi/arch_skeleton which contains the controller and planner server; 
* Clone inside your workspace the repository in this link https://github.com/buoncubi/topological_map containing the ontology for this project;
* Run `chmod +x <file_name>` for each file inside the scripts folder;
* Run `catkin_make` from the root of your workspace.

### Launcher ###

In order to launch the simulation a .lunch file can be used bay using this command into the terminal:
```
roslaunch Assignment1 launch_file.launch
```
It will launch the aRMOR server and all the nodes that implement the robot behavior.
```
<?xml version="1.0"?>
<launch>
    <!-- Run the architecture's component and test it based on random-based stimulus. -->

    <rosparam param="state/initial_pose"> [ 0.0,  0.0] </rosparam>
    <rosparam param="config/environment_size"> [10.0, 10.0] </rosparam>
    <rosparam param="test/random_plan_points"> [2, 8] </rosparam>
    <rosparam param="test/random_plan_time"> [0.2, 0.8] </rosparam>
    <rosparam param="test/random_motion_time"> [0.1, 1.0] </rosparam>

    <node pkg="armor" type="execute" name="armor_service" args="it.emarolab.armor.ARMORMainService"/>
    <node pkg = "Assignment1" type = "my_state_machine.py" name = "my_state_machine" output = "screen" launch-prefix="xterm -e"> </node>
    <node pkg = "Assignment1" type = "robot_state.py" name = "robot_state" output = "screen" launch-prefix="xterm -e"> </node>
    <node pkg = "arch_skeleton" type = "planner.py" name = "planner" output = "screen" launch-prefix="xterm -e"> </node>
    <node pkg = "arch_skeleton" type = "controller.py" name = "controller" output = "screen" launch-prefix="xterm -e"> </node>
</launch>
```

Install xterm to visualize feedback from the nodes launched with the following command:

```
sudo apt-get -y install xterm
```


## Author and contacts ##
Author: Davide Leo Parisi

Contacts:
* personal email: davide.parisi1084@gmail.com
* institutional email: s4329668@studenti.unige.it

