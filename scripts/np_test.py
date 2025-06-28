import numpy as np

# Define the matrix
matrix = np.array([
    [319.744, -137.38, 158.686],
    [-137.38, 158.897, -88.9288],
    [158.686, -88.9288, 83.0644]
])

# Calculate the inverse of the matrix
matrix_inverse = np.linalg.inv(matrix)
print(matrix_inverse)
