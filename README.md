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

This repository contains a ROS package named Assignment1 that includes the following resources:
* `CMakeList.txt`: File to configure this package;
* `package.xml`: File to configure this package;
* `launch/`: Contains the configuration to launch this package
         * launch_file.launch: it launches armor server and all the nodes used in this simulation.

    
    
    


