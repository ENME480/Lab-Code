# Week 11 - UR3e Inverse Kinematics

## Objectives

- Use inverse kinematics (FK) to compute the required joint angles of a UR3e robot arm for its end effector to reach a specific point.
- Determine if the laser pointer is close to the predicted cartesian coordinates.
- Publish commands to control the UR3e robot in ROS2 and visualize results.

## 1. Getting Started with the UR3e

- Power on the robot
- Release the brakes


### Interfacing the Robot with PC

The robot connections and configuration has been setup. You have to interface the robot with ROS to receive and send comands.

- Find the `commands2run.txt` file on the Desktop
- Follow instructions in the file to get the robot interfaced with ROS.

In one more terminal window launch this command:

```bash
ros2 launch ur3e_mrc ur3e_enme480.launch
```

## 2. Modify the IK script to move the robot

Due to inaccuracies in some of your IK calculations, resulting in safety risks due to singulartities, we are giving you a code structure that you need to enter your code in.

It is located in `~/rosPackages/ENME480_ws/src/enme480_fk_labs/enme480_fk_labs/ur3e_ik.py`

To refresh the folder to original state run the following commands:

```bash
cd ~/ENME480_ws/src/enme480_lab_fk
git checkout .
```

You need to modify the following functions within the given snippet (do not change anything else in the code):

- `send_command()` - will be the same as last time (just remove conversion to radians since IK takes care of it)
- `calculate_dh_transform()` - will be exactly same as last time (just make changes to DH parameters if wrong)
- `inverse_kinematics()` - will be exactly similar as your simulation code

Helpful Tip: Use tools like Pastebin or Google Docs to move your code from your laptop to the lab machine

If you are using your own code, remember to change the node name to `ur3e_ik_publisher`

To run it:

```bash
ros2 run enme480_lab_fk ur3e_ik x y z yaw
```

If your IK code has high or slight error, you will receive a prompt on your terminal. Please follow the instructions. The robot will move regardless but those error mean that you need to check your calculations.

## 3. Predicting where the laser point will land

Since we are constraining IK to always face down, the laser point will exactly point at the same `(x,y)` as the end effector. You just need to measure z. Your prediction will depend on your DH transformation.

Turning on the laser pointer:

```bash
ros2 topic pub --once /ur3/laser_point std_msgs/msg/Bool "data: true"
```

If your DH transform is right, you should recieve a similar transformation matrix as the `Correct Transformation Matrix` on your terminal. Otherwise, work on it to get a matrix as similar as possible


## 4. Test Points

Run the robot for the following test points and record the following data:

| Test Point Inputs (x, y, z, Yaw)    | Joint Angles (Your Code)  `(𝜽𝟏, … 𝜽𝟔) `  | Correct Joint Angles `(𝜽𝟏, … 𝜽𝟔) ` | Laser Position on Workbench (Your Prediction) `(x, y)` |  Laser Position on Workbench (Correct Prediction) `(x, y)` | Laser Position on Workbench (Measured Prediction) `(x, y)` | End Effector Position (Your Prediction) `(x, y, z)`| End Effector Position (Correct Prediction) `(x, y, z)`| End Effector Position (Measured) `(x, y, z)`|
| --------------- |:---------------:| --------:| --------:| --------:| --------:| --------:| --------:| --------:|
| [0.2, 0.2, 0.2, 0] | | | | | | | |
| [0.2, 0.4, 0.2, 0] | | | | | | | |
| [0.3, 0.4, 0.1, 45] | | | | | | | |
| [0.3, -0.2, 0.25, 60]  | | | | | | | |
| [0.25, 0.3, 0.3, -30]  | | | | | | | |

## 5. Before you leave the lab

Send yourself the backup/copy of your script and restore the package to its blank version.

IMPORTANT: The below command will erase your script from the computer so take a backup of it before you run it

```bash
cd ~/rosPackages/ENME480_ws/src/enme480_lab_fk
git checkout .
```

# Submission

We will update the submission instructions soon.
<!-- Please create a neatly typed/written report for the lab including the following:

- Correct frame and axes assignments for the UR3e
- A correct DH table for the UR3e (with the updated dimensions)
- A detailed derivation of how the position of laser point is predicted on the workbench.
- Error Analysis (for at least 2 points)
- Your code snippets for the functions supposed to be changed (function for moving the robot and calulating DH transformation matrix) -->