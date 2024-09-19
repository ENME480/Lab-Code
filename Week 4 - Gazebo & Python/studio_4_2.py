#!/usr/bin/env python
import numpy as np

def GetRotationMatrix(phi):
    R = np.zeros(3)

    # ===== Your code starts here =====
    # Define R as shown in assignment document.


    # ===== Your code ends here =====

    return R

def Test():
    phi1 = 0.5
    phi2 = 0.8

    # ===== Your code starts here =====
    # Step 1: Define a 3x1 vector v1, and generate two 
    # matrices R1, R2 by calling GetRotationMatrix() 
    # function.
    

    # Step 2: using np.matmul() to calculate the matrix
    # multiplication product v2 = R2 * R1 * v1, and show v2 value


    # ===== Your code ends here =====
    
    print("Done.")

if __name__ == '__main__':
    Test()
