# Week 6 - UR3e Forward Kinematics on Gazebo

## Objectives

- Add elements to your Gazebo environment
- Calculate DH parameters of UR3e
- Create a publisher to move the robot to desired joint states
- Create a subscriber to find the end effector pose
- Validate and compare the pose readings from DH-parameter calculation & end effector

## 1. Adding Physical Elements in Gazebo

Your aim is to add a base plate to your Gazebo environment and mount the robot on top of it, and link them.

To do so, we modify the Unified Robot Description Format (URDF) file for the environment. This is an XML (Extensible Markup Language) specification. Another commonly known markup language is HTML. The major contrast between XML and HTML is that in addition to diplaying data, XML allows different applications to exchange and store data and its structure in a way that is universally understood. For more info, refer to this link: http://wiki.ros.org/urdf/XML/model

### Step 1: Build the UR Description Package

This package contains the mesh files and all the description files to simulate the UR robots. Clone the repositiory in your **src** folder of your workspace

```bash
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
```

Build and source your workspace

### Step 2: Modify the Description Files to Add a Plate

Check the ```urdf``` folder in the directory. We will be modifying ```ur.urdf.xacro``` and creating a duplicate of ```ur_macro.xacro``` and rename the copy as ```enme480_fk.xacro```. As to why we are modifying these files, will be explained in the studio session.

#### - Changes in ```ur.urdf.xacro``` :

Here just change two things:

- Replace the main macro file being imported from ```ur_macro.xacro``` to ```enme480_fk.xacro```
- The default ```ur_type``` value should be ```ur3e``` (just to make life easy by making your command shorter)

#### - Changes in ```enme480_fk.xacro```

Add the following snippet to add the plate mesh to the environment. The code will be explained in class and you have to figure out where to add the snippets.

Snippet 1: Defining the description of Base Plate

```xml
   <!-- Define the base plate -->
   <link name="${tf_prefix}base_plate">
     <visual>
       <origin xyz="0 0 0" rpy="0 0 0" />  <!-- Modify: Adjust origin as needed -->
       <geometry>
         <box size="0 0 0"/>  <!-- Modify: Size of the base plate (length x width x height) -->
       </geometry>
       <material name="orange"/>
     </visual>
     <collision>
       <origin xyz="0 0 0" rpy="0 0 0" /> <!-- Modify this -->
       <geometry>
         <box size="0 0 0"/> <!-- Modify this -->
       </geometry>
     </collision>
     <inertial>
       <mass value="1.0"/>
       <origin xyz="0 0 0" rpy="0 0 0" /> <!-- Modify this -->
       <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
     </inertial>
   </link>
```

Snippet 2: Defining the relationship between base plate and the robot

You will find a code block that defines how the base link connects to the environment. Replace that with the snippet below

```xml
   <!-- base_joint fixes ..... to the environment  (Find this similar part in the code and replace it) -->
   <joint name="${tf_prefix}base_joint" type="fixed">
     <xacro:insert_block name="origin" />
     <parent link="${parent}" />
     <child link="${tf_prefix}................." /> <!-- Modify this -->
   </joint>

   <!-- Attach base plate to the robot's base link -->
   <joint name="${tf_prefix}base_to_base_plate" type="fixed">
     <parent link="${tf_prefix}............" /> <!-- Modify this based on the child and parent -->
     <child link="${tf_prefix}............." /> <!-- Modify this based on child and parent  (Hint: Check the subsequent code to know the child/parent) -->
     <origin xyz="0 0 0" rpy="0 0 0"/>  <!-- Adjust origin to place the base plate correctly -->
   </joint>
```

Try launching the robot simulation to check if the pate is visible and the robot is standing on the plate:

```bash
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py ur_type:=ur3e
```

You can modify the launch file to have ```ur3e``` as the default argument so that you don't need to specify it everytime

## 2. DH Parameters of UR3e

Link for UR3e specifications: https://www.universal-robots.com/media/1807464/ur3e-rgb-fact-sheet-landscape-a4.pdf

The PDF for UR3 dimensions is included in the folder for ```Week 6```

You need to create a DH-table for the robot and annotate the given PDF to show the frames and axes used. The unknowns here will be the joint angles. Include the base plate in your calculations as well.

## 3. Creating a Publisher script to move the robot

Using the topic ```/joint_trajectory_controller/joint_trajectory``` and the message type ```JointTrajectory``` and ```JointTrajectoryPoint``` from ```trajectory_msgs```, create a publisher to move the robot to desired joint angles. Keep in mind that the angles given to th robot sould be in radians but we want to give the input in degrees so ensure that you have converted that.

The second step is to create a function (or multiple functions) in the same Python class to calculate the end effector pose using forward kinematics via DH-parameters, and print that out as the final transformation matrix.

Hint: Use the structure from your ```pubsub``` codes which you have done previously. You can get the message info for ```JointTrajectory``` and ```JointTrajectoryPoint``` here: http://docs.ros.org/en/noetic/api/trajectory_msgs/html/msg/JointTrajectory.html & http://docs.ros.org/en/noetic/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html

Your command should look something like this:

```bash
ros2 run <package_name> ur3e_fk 0 0 0 0 0 0
```
where the numbers represent the six joint angles in degrees. Hint: Look into how you can send arguments to a Python script

Don't forget to add the node to your ```setup.py``` in your package.

## 4. Create a subscriber to get the end effector pose

Here you will be using the ```/tf``` topic which denotes the transformations in your workspace. The topic publishes the relative transform between all the joints. Your goal is to find the relative transform between the ```base_plate``` and the last link on the robot (figure out which is the last link). You will be shown what ```tf``` is in class.

Get the relative transform and print the position and orientation.

## 5. Compare the readings 

You need to compare the readings from the DH-parameters method with the actual robot position through ```/tf```. Put that in a table for the following 5 test cases:

Set 1: ```[0 0 0 0 0 0]``` (robot should be horizontal)

Set 2: ```[0 0 -90 90 0 0]```

Set 3: ```[0 -45 45 -90 30 90]```

Set 4: ```[90 -60 30 20 10 50]```

Set 5: ```[0 -90 0 0 0 0]``` (robot should be upright)

## Submission

1. Show a screenshot of the base plate with the robot 

2. Show the DH Table for the robot

3. Show a figure with frames and axes marked

4. For each test case, show:

- The set of joint angle values (θ1, θ2, θ3, θ4, θ5, θ6)
- The final transformation matrix (from Python script). You can
add it as a readable image of the output window as well.
- The calculated pose from DH table in simulation vs the pose from ```tf```
- The scalar error

5. Discuss the sources of error

6. An appendix to show your scripts

- ```enme480_fk.xacro```
- FK publisher
- ```tf``` subscriber

Add everything in one single PDF file and upload it.
