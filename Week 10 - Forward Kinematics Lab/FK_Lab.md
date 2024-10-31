# Week 10 - UR3e Forward Kinematics

## Objectives

- Use forward kinematics (FK) to compute the pose of a UR3e robot arm's end effector.
- Determine where a laser pointer on the end effector intersects with a workbench at an arbitrary height.
- Publish commands to control the UR3e robot in ROS2 and visualize results.

## 1. Getting Started with the UR3e

- Power on the robot
- Release the brakes


2. Interfacing the Robot with PC

The robot connections and configuration has been setup. You have to interface the robot with ROS to receive and send comands.

- Find the `commands2run.txt` file on the Desktop
- Follow instructions in the file to get the robot interfaced with ROS

In one more terminal windows launch these commands:

```bash
ros2 launch ur3e_mrc ur3e_enme480.launch
```


## 2. Modify the FK script to move the robot

Due to inaccuracies in some of the DH tables, resulting in safety risks, we are giving you a code structure that you need to enter your code in.

It is located in `~/rosPackages/ENME480_ws/enme480_fk_labs/enme480_fk_labs/ur3e_fk.py`

If you are using your own code, remember to change the node name to `ur3e_fk_publisher`

To run it:

```bash
ros2 run enme480_lab_fk ur3e_fk joint1 joint2 joint3 joint4 joint5 joint6
```

## 3. Predicting where the laser point will land

Since you know the position and orientation of the end effector (attached with a laser pointer), you have to predict where the laser point will land on the workbench. (Hint: Think in terms of vector and plane intersection)

Assume the `z_table = 0`. 

We are providing you with the code in lab, but you need to show the math behind it in your lab report.

Turning on the laser pointer:

```bash
ros2 topic pub --once /ur3/laser_point std_msgs/msg/Bool data:\ true\
```

## 4. Test Points

Run the robot for the following test points:

| Test Point Inputs (𝜽𝟏, … 𝜽𝟔)    | End Effector Position (Your Code)        | Laser Position (from Code) | Laser Position (Measured) |
| --------------- |:---------------:| --------:| --------:|
| [0, -45, 0, 45, -90, 60] | | |
| [-30, -60, 80, -10, -90, -30] | | |
| [30 -70 80 -10 -90 10] | | |
| [-30, -60, 60, -10, -90, -30] | | |

# Submission

We will send out details for creating a lab report later on so record the data you enter in table above for future