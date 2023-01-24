# Navigation 2 demo in ROS2 using TurtleBot3

This repository demonstrates the usage of Navigation 2 framework in ROS 2 using TurtleBot3 Gazebo simulation

This is in service of Navigation 2 Learning experience and fulfills the demonstration of the following -
* Navigation 2 framework
* Behavior Trees (C++)

Note - Add visual demo and pre-req tools/packages
To reproduce the above result, follow the steps below -

1. Clone this repository in a new ros2 workspace in src directory (`turtlebot3_ws/src`) 
2. Import TurtleBot3 packages with vcs - `vcs import . < turtlebot3.repos`
3. Build all packages `colcon build --symlink-install`
4. Source the workspace `source ./install/setup.bash`
5. Export TurtleBot3 model

`export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models`

`export TURTLEBOT3_MODEL=waffle_pi`

6. Launch turtlebot3 simulation infrastructure

`ros2 launch tb3_sim turtlebot3_world.launch.py`

7. Launch nav2 infrastructure (nav2 + amcl initial pose)

`ros2 launch tb3_sim nav2.launch.py`

8. Launch autonomy behavior for demo

`ros2 launch tb3_autonomy autonomy.launch.py`

This starts our demonstration where turtlebot moves between 4 different locations in the world simulation


