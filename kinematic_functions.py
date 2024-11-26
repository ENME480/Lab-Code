import numpy as np
import sys
import math

class KinematicFunctions():


    def calculate_dh_transform(self, joint_positions):
        # DH parameters for UR3e
        # Modify these parameters according to the robot's configuration # a , alpha, d, theta 
        dh_params = [ 
            [0.15*np.sqrt(2), 		0.0, 		0.0, 		3*math.pi/4],
            [0.0, 		        -math.pi/2, 	0.162, 		joint_positions[0]-3*math.pi/4],  # theta1
            [0.24365, 		    0.0, 		    0.0, 		joint_positions[1]],  # theta2
            [0.21325, 		    0.0, 		    0.0, 		joint_positions[2]],  # theta3
            [0.0, 		    -math.pi / 2, 	    0.104, 	    joint_positions[3]-math.pi/2],  # theta4
            [0.0, 		    math.pi / 2, 	    0.083, 	    joint_positions[4]],  # theta5
            [0.0535, 		    0.0, 		    0.151, 		joint_positions[5]]   # theta6
        ]

        transform = np.eye(4)  
        
        for a, alpha, d, theta in dh_params:
            transform_i = np.array([
                [math.cos(theta), -math.sin(theta) * math.cos(alpha), math.sin(theta) * math.sin(alpha), a * math.cos(theta)],
                [math.sin(theta), math.cos(theta) * math.cos(alpha), -math.cos(theta) * math.sin(alpha), a * math.sin(theta)],
                [0, math.sin(alpha), math.cos(alpha), d],
                [0, 0, 0, 1]
            ])
            transform = np.dot(transform, transform_i)  

        print(f'DH Transformation Matrix:\n{transform}')


        return transform


    def inverse_kinematics(self, xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
        
        return_value = np.array([0, 0, 0, 0, 0, 0])

        ####################################### Your Code Starts Here #######################################
        
        # TODO: Function that calculates an elbow up inverse kinematics solution for the UR3

        # Step 1: find gripper position relative to the base of UR3
        x_grip = xWgrip + 0.15
        y_grip = yWgrip - 0.15
        z_grip = zWgrip - 0.01
        theta_yaw = np.radians(yaw_WgripDegree)

        # Step 2: find x_cen, y_cen, z_cen
        L9 = 0.0535
        x_cen = x_grip - L9 * np.cos(theta_yaw)
        y_cen = y_grip - L9 * np.sin(theta_yaw)
        z_cen = z_grip
        #print(str(x_cen) + ", " + str(y_cen) + ", " + str(z_cen))

        # Step 3: find theta_1
        L4 = 0.093
        L2 = 0.120
        L6 = 0.104
        theta_cen = np.arctan(y_cen/x_cen)
        Ly = L2-L4+L6
        r_cen = np.sqrt(x_cen**2 + y_cen**2)
        theta_arm = np.arcsin(Ly/r_cen)
        theta_1 = theta_cen - theta_arm

        # Step 4: find theta_6
        theta_6 = theta_1 - theta_yaw + np.radians(90)
        theta_5 = np.radians(-90)

        # Step 5: find x3_end, y3_end, z3_end
        L7 = 0.083
        gap = 0.027  
        L8 = 0.092
        x1 = x_cen - L7 * np.cos(theta_1)
        y1 = y_cen - L7 * np.sin(theta_1)
        x3_end = x1 + (L6 + gap) * np.sin(theta_1)
        y3_end = y1 - (L6 + gap) * np.cos(theta_1)
        z3_end = z_cen + 0.059 + L8
        # print(np.arctan2(y3_end, x3_end)*180/np.pi)

        # Step 6: find theta_2, theta_3, theta_4
        L1 = 0.152
        L3 = 0.244
        L5 = 0.213
        r_end = np.sqrt(x3_end**2 + y3_end**2)
        h1 = z3_end - L1

        a1 = L3**2 + L5**2 - (r_end**2 + h1**2)
        a2 = 2*L3*L5
        beta1 = np.arccos(a1/a2)
        theta_3 = np.pi - beta1

        a3 = L3**2 + (r_end**2 + h1**2) - L5**2
        a4 = 2*L3*np.sqrt(r_end**2 + h1**2)
        beta2 = np.arccos(a3/a4)
        beta3 = np.arctan2(h1, r_end)

        theta_2 = - beta2 - beta3
        theta_4 = - theta_2 - theta_3

        ############################# Your Code Ends Here #######################################

        # print theta values (in degree) calculated from inverse kinematics
        
        # print("Correct Joint Angles: ")
        # print(str(theta_1*180/np.pi) + " " + str(theta_2*180/np.pi) + " " + \
        #         str(theta_3*180/np.pi) + " " + str(theta_4*180/np.pi) + " " + \
        #         str(theta_5*180/np.pi) + " " + str(theta_6*180/np.pi))

        # obtain return_value from forward kinematics function
        return_value = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]

        return return_value 