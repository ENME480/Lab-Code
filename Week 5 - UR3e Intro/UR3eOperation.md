# Week 5 - UR3e Intro & Operation

## Objectives

- Learn how to use the Pendant
- Interface UR3e with ROS packages on RAL machines
- Visualize ROS Processes
- Use MoveIt to operate the robot

## Procedure

1.  You will be shown how to use the pendant

- Power on the robot
- Release the brakes
- Try Freedrive
- Try Emergency Stop
- Try setting the joint anngles to moe the robot (Forward Kinematics)
- Try setting the final end effector position to move the robot (Inverse Kinematics)
- 

2. Interfacing the Robot with PC

The robot connections and configuration has been setup. You have to interface the robot with ROS to receive and send comands.

- Find the `commands2run.txt` file on the Desktop
- Follow instructions in the file

3. Visualize ROS Processes

- Get the list of ROS topics
- Open up RQT
- Visualize Node Graphs
- You will be shown how to generate plots in RQT to analyze data

4. Move the robot using MoveIt

- Open up the MoveIt node

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e
```

- You will be shown how to move the robot using MoveIt





