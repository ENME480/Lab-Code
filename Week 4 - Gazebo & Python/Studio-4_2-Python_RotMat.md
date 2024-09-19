# Week 4 - Studio 4.2 - Rotation Matrices in Python

## Overview

This assignment involves modifying a Python script named `studio_4_2.py` (located in the this folder) that performs matrix operations using rotation matrices. The task includes defining a function to generate a 3x3 rotation matrix, initializing a specific vector, and calculating the result of matrix multiplications involving two angles.

## Requirements

- Python 3.x
- NumPy library

## Instructions

1. **Define a function `R(phi)`**:
   - This function should take an angle `φ` (phi) as input and generate a 3x3 rotation matrix using the formula:

     $` R(\phi) = 
     \begin{bmatrix}
     \cos\phi & -\sin\phi & 0 \\
     \sin\phi & \cos\phi & 0 \\
     0 & 0 & 1
     \end{bmatrix} `$
    

2. **Define a vector `v1`**:
   - Inside the `Test()` function, define a 3x1 NumPy array:

     $` v_1 = 
     \begin{bmatrix}
     1 \\
     0.6 \\
     0.8
     \end{bmatrix} `$

3. **Calculate the result `v2`**:
   - Use the angles `φ1 = 0.5` and `φ2 = 0.8` to compute the matrix multiplication:

     $` v_2 = R(\phi_2) R(\phi_1) v_1 `$

4. **Run the script**:
   - Execute the script from the terminal using:

     ```
     python studio_4_2.py
     ```

   - The output should display the calculated value of `v2`.

## Submission

- Submit a PDF containing your `studio_4_2.py` script and a screenshot of the terminal output to ELMS.

## Notes

- Ensure that the script runs without errors and outputs the correct `v2` value.
- Refer to the rubric on ELMS for grading
- You will not be assessed if you do not follow the given script format
