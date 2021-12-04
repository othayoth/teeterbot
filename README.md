# TeeterBot 
TeeterBot is a self-balancing robot simulation model for ROS / Gazebo. The dimensions and mass of each component are easily configured using launch file arguments, so it is easy to adjust physical parameters to test robustness of control algorithms.

# Teeterbot self-balancing
This repository is modified from the original teeterbot repository to implement a sel-balancing controller. 

## Example cases and visualization
Please note that the robot is purposely made to lose balance by driving it at high velocities or suddenly changing velocities -- this causes large deviations from the linear approximation and leads to loss of balance

https://user-images.githubusercontent.com/4443765/144693354-0a5a1ac5-e981-4324-9062-4aed298d4653.mp4

## Instructions for running the code

1. Clone this repository to your workspace
```
git clone git@github.com:othayoth/teeterbot.git
```
2. Install the ROS Package `teleop_twsit_keyboard`. Follow instructions [here](http://wiki.ros.org/teleop_twist_keyboard)
3. Build your catkin workspace using command `$ catkin_make`
4. Source your `setup.bash` file from your catkin workspace
```
source devel/setup.bash
```
6. Run the following launch file
```
roslaunch teeterbot_gazebo teeterbot_control.launch
```
7. In a separate terminal, start the teleoperation node
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
## Evaluating the solution
- Use the instructions in terminal in which `teleop_twist_keyboard` is running to change the forward and turning speed of the robot. 
- At the default turning and forward speeds specified by the node, the robot moves stably, even when directions are changed.
- Sudden stops and starts become unstable at pure forward translation beyond ~4.5 m/s (this may reduce when the robot simultaneously turns and translates)


## Description of solution

 
 

