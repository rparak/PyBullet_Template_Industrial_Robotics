import numpy as np

def matrix_to_quaternion(matrix):
    if matrix.shape != (4, 4):
        raise ValueError("Input matrix must be a 4x4 homogeneous transformation matrix.")

    # Extract the rotation part of the matrix
    rotation_matrix = matrix[:3, :3]

    # Calculate the trace of the rotation matrix
    trace = np.trace(rotation_matrix)

    if trace > 0:
        S = 2 * np.sqrt(trace + 1)
        w = 0.25 * S
        x = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / S
        y = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / S
        z = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / S
    elif (rotation_matrix[0, 0] > rotation_matrix[1, 1]) and (rotation_matrix[0, 0] > rotation_matrix[2, 2]):
        S = 2 * np.sqrt(1 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2])
        w = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / S
        x = 0.25 * S
        y = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / S
        z = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / S
    elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
        S = 2 * np.sqrt(1 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2])
        w = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / S
        x = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / S
        y = 0.25 * S
        z = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / S
    else:
        S = 2 * np.sqrt(1 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1])
        w = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / S
        x = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / S
        y = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / S
        z = 0.25 * S

    return np.array([w, x, y, z])

# Example usage
homogeneous_matrix = np.array([[1.0, 0.0, 0.0, 1],
                               [0.0, 0.0, -1.0, 2],
                               [0.0, 1.0, 0.0, 3],
                               [0, 0, 0, 1]])

quaternion = matrix_to_quaternion(homogeneous_matrix)
print("Quaternion:", quaternion)