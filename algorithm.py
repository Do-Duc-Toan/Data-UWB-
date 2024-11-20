import numpy as np
from scipy.optimize import least_squares

# Define the residuals function for least squares
def residuals(params, d_A, d_B, d_C, d_D):
    x, y, z = params
    res_A = np.sqrt(x**2 + y**2 + z**2) - d_A
    res_B = np.sqrt((x - 3)**2 + y**2 + z**2) - d_B
    res_C = np.sqrt((x - 3)**2 + (y - 3)**2 + z**2) - d_C
    res_D = np.sqrt(x**2 + (y - 3)**2 + z**2) - d_D
    return [res_A, res_B, res_C, res_D]

# Measured distances (assumed for example)
d_A, d_B, d_C, d_D = 4.8, 3.6, 4.1, 2.8  # Example distances

# Initial guess for the coordinates of P (x, y, z)
initial_guess = [1.5, 1.5, 1.5]

# Solve the system using least squares
result = least_squares(residuals, initial_guess, args=(d_A, d_B, d_C, d_D))

# Extract the solution
x_sol, y_sol, z_sol = result.x

print(f"Estimated coordinates of P: x = {x_sol}, y = {y_sol}, z = {z_sol}")
