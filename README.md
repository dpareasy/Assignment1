# Assignment1

Parisi Davide Leo 4329668 

## Introduction ##

This repository contains ROS-based software, developed in python language, that implements the simulation of a surveillance robot. To this purpose it is used [topological_map.owl](https://github.com/buoncubi/topological_map) ontology , which creates an indoor environment with a robot for the surveillance.

The software uses a Smach state machine and build the ontology with aRMOR server using [armor_api](https://github.com/EmaroLab/armor_py_api).

## Scenario ##

The scenario involves a surveying robot deployed in a indoor environmnet. It's objective is to visit different locations and stay there for some times. Before starting moving around the map it must wait for receiving all the information about the environment. The robot should start in it's initial position which is also the recharging location. Any time it  enters a room, the robot should check it for some times before starting reasoning again to choose the next location to visit. 

The environment in which the robot moves is developed in a way in which different scenarios can be created even if only under certain [Assumptions](#Assumptions). 

### Policy ###

The moving policy that the robot should follow is the one presented below:
* It should mainly stay on corridors;
* If a reachable room become `URGENT` the robot should visit it;
* When its battery is low it should move to the recharging position;

### Assumptions ###

For simplicity we consider a scenario with the following assumptions:
* The robot moves in a 2D environment with no obstacles;
* The environment created can be formed by any number of corridors;
* The number of rooms which a corridor can contain is the same for each corridor;
* Corridor(i) is connected to corridor(i+1) and to the recharging location (which is a corridor);
* The duration of the robot's battery is 60 seconds;
* The robot is automatically spawned in the recharging room every time the battery goes low;
* The corridors' timestamps `visitedAt` are not considered since their urgency requirement are different from rooms';
* If there are no urgent rooms the robot is forced to move around corridors;

## Software architecture ##

Given the scenario presented above the software is developed as follow.

### Components diagram ###

### The Finite State Machine ###

The figure below represent the structure of the Finite State Machine.

![smach](https://user-images.githubusercontent.com/92155300/204056867-88b33dd0-3f09-4bea-8fbf-265921ca48a1.png)


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

### Dependencies ###

## Software components ###

### The `state_machine` Node ###

This node defines the Finite State Machine of the architecture and manages the transitions between all the states.
The exectuion functions of each state relies on the class `helper` of the `helper_interface` module, developed by Luca Buoncompagni in [arch_skeleton](https://github.com/buoncubi/arch_skeleton) and modified to fit to this purpose, on the class `behavior` of the `robot_actions` module and on the class `ontology` of the `load_ontology` module of this repostory.

The following figure represents an example of the `state_machine` node terminal which shows the various transitions between states.

![state_machine_transitions](https://user-images.githubusercontent.com/92155300/204084546-4e3bb3e9-8910-454e-b1d9-296cc32cab04.png)

### The `robot_actions` Node ###

This node defines the `BehaviorHelper` class which defines the methods for helping the robot in reasoning, moving to location and moving to recharge position. The methods of this class are used inside the execute function of the states of the state machine.

#### The `decide_target` function ####
```
def decide_target():
   get actual robot position
   get reachable destinations
   get urgent locations
   intersect reachable destination with urgent locations
   if empty list:
      move trhough corridors
   else:
      choose the most urgent room

```

#### The `move_to_target` function ####
```
def move_to_target():
   move the robot in the new position
   if new position is a room:
      update location timestamps     
   update robot timestamps   

```

### The `load_ontology` Node ###

This node defines the `CreateMap` class used in the  `STATE_INIT` of the Finite State Machine, to pass to the robot all the information for the indoor movement. Such an environment is created through manipulations on the [topological_map.owl](https://github.com/buoncubi/topological_map) ontology with the help of the `ArmorClient` class defined in [armor_api](https://github.com/EmaroLab/armor_py_api).

### The `controller` and `planner` Nodes ###

You can finde a complete description in [arch_skeleton](https://github.com/buoncubi/arch_skeleton).

## Installation & Running ##

This software is based on ROS Noetic.

### Installation ###

Follow these steps to install the software:
1. Clone this repository inside your workspace (make sure it is sourced in you .bashrc);
2. Follow the steps for [aRMOR](https://github.com/EmaroLab/armor/issues/7) installation;
3. Use [armor_api](https://github.com/EmaroLab/armor_py_api) for server requests, you can clone it in your workspace from this repository ;
4. Clone inside your workspace [arch_skeleton](https://github.com/buoncubi/arch_skeleton) which contains the controller and planner server; 
5. Clone inside your workspace [topological_map](https://github.com/buoncubi/topological_map) containing the ontology for this project;
6. Run `chmod +x <file_name>` for each file inside the scripts folder;
7. Run `catkin_make` from the root of your workspace.

### Launcher ###

In order to launch the simulation a .lunch file can be used by copying this command into the terminal:
```
roslaunch Assignment1 launch_file.launch
```
It will launch the aRMOR server and all the nodes that implement the robot behavior.
```
<?xml version="1.0"?>
<launch>

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


## System limitations ##

As stated in [Assumptions](#Assumptions), some hypothesis have been made to simplify the development of the simulation. First of all, when the battery goes down the robot is automatically spowned in the recarging room instead of searching for the best path which connects it to the recharging site. Moreover, it is not possible to create any type of environment, indeed the number of rooms is strictly related to the number of corridors, since the main hypothesis made is that the number of rooms is equal for each corridor.

## Possible improvements ##

## Author and contacts ##
Author: Davide Leo Parisi

Contacts:
* personal email: davide.parisi1084@gmail.com
* institutional email: s4329668@studenti.unige.it

