# Week 4 - Studio 4.1 - Gazebo Demo

The objective of this lab is to install the UR3e packages and have a working simulation of the robot in Gazebo.

## Package Installation

There are two methods to do this:

### 1. Source Installation

Clone the following repositories in your workspace:

```bash
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git
```

Build and source the workspace

### Troubleshooting - no joint trajcetory controller or controller interface

If you run into an issue with building packages due to missing a joint controller run:

``` bash && sudo apt install ros-humble-joint-trajectory-controller```

``` bash && sudo apt install ros-humble-controller-interface```

### 2. Pre-configured Docker Container

Find the Dockerfile in ```/Resources/Docker Container/humble_dockerfile.Dockerfile``` and build and run the container. This is the preferred method but it can lead to issues with Gazebo (looking into a foolproof solution - will be updated this week)

```bash
sudo docker build -t humble_image -f humble_dockerfile.Dockerfile .
sudo docker run -it humble_image
```

## Running UR3 Demo on Gazebo

Launch the UR3e in Gazebo

```bash
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py
```

It should open up two windows with UR3e arm in Gazebo & RViz. 

### Troubleshooting - Gazebo does not open up; waiting for controller

Run the following command in a seperate terminal before launching the previous command

```bash
gazebo -s libgazebo_ros_init.so -s libgazebo_ros_factory.so myworld.world
```

To test if the simulation works, run the following command

```bash
ros2 launch ur_robot_driver test_joint_trajectory_controller.launch.py
```
This will keep moving the robot continously in multiple positions.



## Assignment

Prepare a report answering the following questions and posting relevant screenshots
1. Screenshots of Gazebo & RViz with the UR3 in 3 different positions
2. Show the topics 
