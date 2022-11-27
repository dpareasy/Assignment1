# Assignment1

Author: Davide Leo Parisi

Contacts:
* personal email: davide.parisi1084@gmail.com
* institutional email: s4329668@studenti.unige.it


## Introduction ##

This repository contains a ROS-based software, developed in python language, that implements the behavior of a surveillance robot. To this purpose it is used [topological_map.owl](https://github.com/buoncubi/topological_map/blob/main/topological_map.owl) ontology , which creates the indoor environment for the simulation.

The software uses a Smach state machine and build the ontology with aRMOR server using [armor_api](https://github.com/EmaroLab/armor_py_api).

## Scenario ##

The scenario involves a surveying robot deployed in a indoor environmnet. Its purpose is to move through the different locations of the building and spend some times inside before starting reasoning to decide the next room to visit. Before starting moving around the environment the robot must wait until receiving all the information about the map. Any time the battery goes down the robot moves to the recharging location which is the initial position.


### Policy ###

The moving policy that the robot should follow is the one presented below:
* It should mainly stay on corridors;
* If a reachable room become `URGENT` the robot should visit it;
* When its battery is low it should move to the recharging position;

### Assumptions ###

For simplicity following assumptions have been considered:
* The robot moves in a 2D environment with no obstacles;
* The environment created can be formed by any number of corridors;
* All the corridors contain the same number of rooms;
* Corridor(i) is connected to corridor(i+1) and to the recharging location (which is a corridor);
* The robot is automatically spawned in the recharging location every time the battery goes low;
* The corridors' timestamps `visitedAt` are not considered since their urgency requirement are different from rooms';
* If there are no urgent rooms the robot is forced to move around corridors;

## Software architecture ##

Given the scenario presented above the software is developed as follow.

### Components diagram ###

### Sequence diagram ###

In the figure below is presente the sequence diagram of the architecture

![sequence_diagram](https://user-images.githubusercontent.com/92155300/204132186-6b968e18-f537-4d8b-ac87-34f86a7a6382.png)

As can be seen from the above image, the state machine waits until the whole map has been built

### The Finite State Machine ###

Hereafter the structure of the Finite State Machine.

![smach](https://user-images.githubusercontent.com/92155300/204056867-88b33dd0-3f09-4bea-8fbf-265921ca48a1.png)


The figure shows a hierachical Finite State Machine made of the following states:
1. *INITIALIZE_MAP*: The state in which the robot build the map of the environment.
2. *NORMAL*: composed of three other states:
    * *DECIDE_LOCATION*: The state in which the robot decides the location to reach.
    * *MOVING_TO_LOCATION*: The state in which the robot moves until reaching the target location.
    * *SURVEYING*: The state in which the robot takes some times to survey the location.
3. *RECHARGING*: The state in which the robot charges its battery.

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
    * state_machine.py: It defines the states of the state machine;
    * robot_state.py: It implements the robot state including: current position, and battery level;

### Dependencies ###

## Software components ###

### The `state_machine` Node ###

This node defines the Finite State Machine of the architecture and manages the transitions between all the states.


It relies on three different classes:
* The class `InterfaceHelper` of the `interface_helper` module, developed by Luca Buoncompagni in [arch_skeleton](https://github.com/buoncubi/arch_skeleton) and modified to fit to this purpose, which manages all the interactions with [planner](https://github.com/buoncubi/arch_skeleton/blob/main/scripts/planner.py) and [controller](https://github.com/buoncubi/arch_skeleton/blob/main/scripts/controller.py).
* The class `BehaviorHelper` of the `robot_actions` module, which manages the interactions with the aRMOR server (i.e. robot position, reachable locations, urgency ecc.)
* The class `CreateOntology` of the `load_ontology` module, which manages the initialisation of the map.

The following figure represents an example of the `state_machine` node terminal which shows the various transitions between states.

![state_machine_transitions](https://user-images.githubusercontent.com/92155300/204084546-4e3bb3e9-8910-454e-b1d9-296cc32cab04.png)

https://user-images.githubusercontent.com/92155300/204129148-e2371c3d-3a2e-49e7-aa8c-c68b78763a5b.mp4



### The `robot_actions` Node ###

This node defines the `BehaviorHelper` class which defines the methods for making the robot reason, move to location and move to recharging position. This node uses the class `ArmorClient` to make request to the aRMOR server for all the manipulations and queries nedded for making the robot perform its actions.

Here the reasoning and moving function pseudocode is shown.

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

The function asks the user some information to build the environment which is developed in such a way that different scenarios can be created, even if only under certain [Assumptions](#Assumptions). 

### The `robot_state` Node ###

This node implements two different servers:
* The `state/get_pose`. 
* The `state/set_pose`
The first requires nothing and returns a `Point` which is the actual position of the robot, while the second requires a `Point` to be set and returns nothing. This node also get the robot initial position as a parameter and calls the `init_robot_pose` function, defined inside the `interface_helper`, which makes the request for the controller server.

A publisher, the `state/battery_low`, is also implemented here. It is a `Boolean` publisher which has two possible states:
* Low battery: if `True`is published.
* Charged: if `False` is published.

### The `controller` and `planner` Nodes ###

You can finde a complete description of these two nodes in [arch_skeleton](https://github.com/buoncubi/arch_skeleton).

### ROS parameters ###

This software requires the following ROS parameters.
 
 - `config/environment_size`: It represents the environment boundaries as a list of two float
   numbers, i.e., `[x_max, y_max]`. The environment will have the `x`-th coordinate spanning
   in the interval `[0, x_max)`, while the `y`-th coordinate in `[0, y_max)`.

 - `state/initial_pose`: It represents the initial robot pose as a list of two float numbers, 
   i.e., `[x, y]`. This pose should be within the `environmet_size`.

 - `test/random_plan_points`: It represents the number of via points in a plan, and it should be
   a list of two integer numbers `[min_n, max_n]`. A random value within such an interval will be
   chosen to simulate plans of different lengths.

 - `test/random_plan_time`: It represents the time required to compute the next via point of the 
   plan, and it should be a list of two float numbers, i.e., `[min_time, max_time]` in seconds. 
   A random value within such an interval will be chosen to simulate the time required to 
   compute the next via points.

 - `test/random_motion_time`: It represents the time required to reach the next via point, and 
   it should be a list of two float numbers, i.e., `[min_time, max_time]` in seconds. A random
   value within such an interval will be chosen to simulate the time required to reach the next 
   via points. 

## Installation & Running ##

This software is based on ROS Noetic.

### Installation ###

Follow these steps to install the software:
1. Clone this repository inside your workspace (make sure it is sourced in you .bashrc);
2. Follow the steps for [aRMOR](https://github.com/EmaroLab/armor/issues/7) installation;
3. Use [armor_api](https://github.com/EmaroLab/armor_py_api) for server requests, you can clone it in your workspace;
4. Clone inside your workspace [arch_skeleton](https://github.com/buoncubi/arch_skeleton) which contains the [controller.py](https://github.com/buoncubi/arch_skeleton/blob/main/scripts/controller.py) and [planner.py](https://github.com/buoncubi/arch_skeleton/blob/main/scripts/planner.py) server; 
5. Clone inside your workspace [topological_map](https://github.com/buoncubi/topological_map) containing the ontology for this project;
6. Run `chmod +x <file_name>` for each file inside the scripts folder;
7. Run `catkin_make` from the root of your workspace.

As regardin the third point, the developer have encountered some issues with the function `disj_inds_of_class(self, class_name)` of [armor_manipilation_client.py](https://github.com/EmaroLab/armor_py_api/blob/main/scripts/armor_api/armor_manipulation_client.py). To fix this problem a new function have been created:


```python
def disjoint_all_ind(self, ind_list):
        try:
            res = self._client.call('DISJOINT', 'IND','', ind_list)

        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

        if res.success:
            return res.is_consistent
        else:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)
```

Copy this function inside `armor_manipulation_client.py` after having cloned the package inside your workspace.

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

As stated in [Assumptions](#Assumptions), some hypothesis have been made to simplify the development of the simulation. First of all, when the battery goes down the robot is automatically spowned in the recarging room instead of searching for the best path which connects it to the recharging site. Another limitation is that it is not possible to create any type of environment, indeed the number of rooms is strictly related to the number of corridors, since the main hypothesis made by the author is that the number of rooms is equal for each corridor.

## Possible improvements ##

