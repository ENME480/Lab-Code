# Week 4 - Studio 4.1 - Gazebo Demo

The objective of this lab is to install the UR3e packages and have a working simulation of the robot in Gazebo.

## Package Installation

There are two methods to do this:

## 1. Source Installation

Clone the following repositories in your workspace:

```bash
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git
```

**Build and source the workspace**


## 2. Pre-configured Docker Container

Find the Dockerfile in ```/Resources/Docker Container/humble_dockerfile.Dockerfile``` and build and run the container. This is the preferred method but it can lead to issues with Gazebo (looking into a foolproof solution - will be updated this week)

```bash
sudo docker build -t humble_image -f humble_dockerfile.Dockerfile .
sudo docker run -it humble_image
```


### Troubleshooting - no joint trajcetory controller or controller interface

If you run into an issue with building packages due to missing a joint controller run:

``` bash
sudo apt install ros-humble-joint-trajectory-controller

sudo apt install ros-humble-controller-interface

sudo apt install ros-humble-ur-*

sudo apt install ros-humble-control-*
```

## Troubleshooting on Macs

Gazebo doesn’t directly support ARM64 architecture. As a result, we need to manually compile and install it.

Install necessary dependencies:

```bash
sudo apt-add-repository ppa:dartsim

sudo apt update

sudo apt install libdart-dev libdart-utils-dev libdart-external-ikfast-dev libsdformat9-dev libfreeimage-dev libprotoc-dev libprotobuf-dev protobuf-compiler freeglut3-dev libcurl4-openssl-dev libtinyxml-dev libtinyxml2-dev libtar-dev libtbb-dev libogre-1.9-dev libxml2-dev pkg-config qtbase5-dev libqwt-qt5-dev libltdl-dev libgts-dev libboost-thread-dev libboost-system-dev libboost-filesystem-dev libboost-program-options-dev libboost-regex-dev libboost-iostreams-dev libsimbody-dev libignition-common3-dev libignition-fuel-tools4-dev libignition-transport8-dev libignition-math6-dev libignition-msgs5-dev
```

2. Clone the Gazebo source code from GitHub:
```bash
cd ~/Downloads/

git clone https://github.com/osrf/gazebo
```


3. Modify the line 647 of ```SearchForStuff.cmake``` in ```Downloads/gazebo/cmake```. Change from 9.8 to 9.7 as the default libsdformat version of ubuntu22 is 9.7.

4. Compile and install Gazebo:

```bash
cd ~/Downloads/gazebo
mkdir build && cd build
cmake ../
make -j3
sudo make install
```

5. Add Gazebo to your environment path by **modifying .bashrc***:

```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export PATH=/usr/local/bin:$PATH
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
```


6. In your workspace, add the ```gazebo_ros``` package:

```bash
cd ~/enme480_ws/src
git clone https://github.com/ros-simulation/gazebo_ros_pkgs
cd gazebo_ros_pkgs
git checkout ros2
```

7. **Build and source your workspace**


# Running UR3 Demo on Gazebo

Launch the UR3e in Gazebo

```bash
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py
```

It should open up two windows with UR3e arm in Gazebo & RViz. 

### Troubleshooting - Gazebo does not open up; waiting for controller

Run the following command in a seperate terminal before launching the previous command (in order to use this method, start and source 3 consoles, run this command, then the one above it then the last command on this page).

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
