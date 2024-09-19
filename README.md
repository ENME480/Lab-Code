
# Assignment: Matrix Multiplication with Rotation Matrices

## Overview

This assignment involves creating a Python script named `test.py` that performs matrix operations using rotation matrices. The task includes defining a function to generate a 3x3 rotation matrix, initializing a specific vector, and calculating the result of matrix multiplications involving two angles.

## Requirements

- Python 3.x
- NumPy library

## Instructions

1. **Define a function `R(phi)`**:
   - This function should take an angle `φ` (phi) as input and generate a 3x3 rotation matrix using the formula:

     \[
     R(\phi) = 
     \begin{bmatrix}
     \cos\phi & -\sin\phi & 0 \\
     \sin\phi & \cos\phi & 0 \\
     0 & 0 & 1
     \end{bmatrix}
     \]

2. **Define a vector `v1`**:
   - Inside the `Test()` function, define a 3x1 NumPy array:

     \[
     v_1 = 
     \begin{bmatrix}
     1 \\
     0.6 \\
     0.8
     \end{bmatrix}
     \]

3. **Calculate the result `v2`**:
   - Use the angles `φ1 = 0.5` and `φ2 = 0.8` to compute the matrix multiplication:

     \[
     v_2 = R(\phi_2) R(\phi_1) v_1
     \]

4. **Run the script**:
   - Execute the script from the terminal using:

     ```
     python test.py
     ```

   - The output should display the calculated value of `v2`.

## Submission

- Show your script to the TA.
- Submit the `test.py` file to ELMS for session credit.

## Notes

- Ensure that the script runs without errors and outputs the correct `v2` value.
- Completion of this assignment is graded based on completeness.
