# ENME480 Final Project

# Pick and Place Task using Camera and UR3

## Objectives

The objective of this lab is to control the UR3 to move three colored blocks (AR-tagged in lab) to desired positions using camera image as inputs in Gazebo simulation. We will use OpenCV for processing the image data. The program will also integrate the functions of previous lab assignments. In this lab we will:
- Use OpenCV functions to find the centroid of each block
- Convert the pixel coordinates in an image to coordinates in the world frame
- Move the blocks from the detected positions to predefined desired positions

## Task Description

### Overview

A simulation environment for the project is shown in Figure 1.