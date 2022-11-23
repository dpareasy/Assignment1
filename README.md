# Assignment1

Parisi Davide Leo 4329668 

## Introduction ##

This repository contains ROS-based software, developed in python language, that simulates a behavioural architecture. The objective is to drive a robot around a particular environment created by manipulations service requests to aRMOR server. In order to not make the transitions between states of the state machine istantaneous, a dummy simulated behavior is implemented using mutexes.

## Scenario ##

The scenario involves a surveilling robot deployed in a indoor environmnet. It's objective is to visit different locations and stay there for some times.It must wait for receiving all the information to build the topological map before starting surveilling. The robot should start in it's initial position which is also the recharging location. When it moves in a new location it should wait some times before starting reasoning for moving in another location.
When its battery is low it should move to the recharging position before starting again the above mentioned behavior.

The moving policy that the robot should hae is the following:
* It should mainly stay on corridors;
* If a reachable room has not been visited for some specified times it should visit it.

Reasoning and urgency are managed by the ontology.

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
* Clone this repossitory inside your workspace (make sure it is sourced in you .bashrc);
* Run chmod +x <file_name> for each file inside the scripts folder;
* Run catkin_make from the root of your workspace.
* Install xterm by entering the command: `sudo apt install -y xterm`

### Launcher ###

In order to launch the simulation a .lunch file can be used bay using this command into the terminal:
```
roslaunch Assignment1 launch_file.launch
```
It will launch the aRMOR server and all the nodes that implement the robot behavior.
