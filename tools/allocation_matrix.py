import numpy as np

weight = np.array([1, 1, 1, 1])
H = np.diag(np.concatenate((weight, weight)))
uav1_pos = np.array([0.275, 0])
uav2_pos = np.array([-0.275, 0])

A = np.array([[1, 0, 0, 0, 1, 0, 0, 0],
              [-uav1_pos[1], 1, 0, 0, -uav2_pos[1], 1, 0, 0],
              [uav1_pos[0], 0, 1, 0, uav2_pos[0], 0, 1, 0],
              [0, 0, 0, 1, 0, 0, 0, 1]])

U = np.linalg.inv(H) @ A.T @ np.linalg.inv(A @ np.linalg.inv(H) @ A.T)


F = np.array([10,1,0,0])
print(U)
print(F.T)