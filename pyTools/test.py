import numpy as np

if __name__ == "__main__":
    R = [[1.732 / 2, -0.5, 0],
         [0.5, 1.732 / 2, 0],
         [0, 0, 1]]
    R_inv = np.linalg.inv(R)
    print(R_inv)