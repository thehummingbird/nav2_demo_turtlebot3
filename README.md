# Navigation 2 demo in ROS2 using TurtleBot3

![nav2_screenshot](./images/nav2_screenshot.png)

This repository demonstrates the usage of Navigation 2 framework in ROS 2 using TurtleBot3 Gazebo simulation.

It is inspired by the works of Davide Faconti's [BehaviorTree.CPP](https://www.behaviortree.dev/) and Sebastian Castro's [navigation 2 demo](https://github.com/sea-bass/turtlebot3_behavior_demos). I reverse engineered and stripped down navigation 2 demo further for first time developers. I hope this serves as a great base for you to start working with navigation 2 in ROS 2.

The repository is in service of Navigation 2 Learning experience and fulfills the demonstration of the following -
* Navigation 2 framework
* Behavior Trees (C++)

Additionally, for first time ROS 2 developers, this repository also demonstrates the following -
* ROS 2 python package usage
* ROS 2 C++ package usage
* ROS 2 launch infrastructure

Note - Add visual demo and pre-req tools/packages
To reproduce the above result, follow the steps below -

1. Clone this repository in a new ros2 workspace in src directory (`turtlebot3_ws/src`) 
```
git clone git@github.com:thehummingbird/nav2_demo_turtlebot3.git .
```
2. Import TurtleBot3 packages with vcs 
```
vcs import . < turtlebot3.repos
```
3. Build all packages 
```
colcon build --symlink-install
```
4. Source the workspace
```
source ./install/setup.bash
```
5. Export TurtleBot3 model

```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models

export TURTLEBOT3_MODEL=waffle_pi
```

6. Launch turtlebot3 simulation infrastructure

```
ros2 launch tb3_sim turtlebot3_world.launch.py
```

7. Launch nav2 infrastructure (nav2 + amcl initial pose)

```
ros2 launch tb3_sim nav2.launch.py
```

8. Launch autonomy behavior for demo

```
ros2 launch tb3_autonomy autonomy.launch.py
```

This starts our demonstration where turtlebot moves between 4 different locations in the world simulation (set in `tb3_sim/config/sim_house_locations.yaml`)


