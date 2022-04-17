
import numpy as np

if __name__ == "__main__":
    TVehicle2Stereo = [[-0.00680499, -0.0153215, 0.99985, 1.64239],
                       [-0.999977, 0.000334627, -0.00680066, 0.247401],
                       [-0.000230383, -0.999883, -0.0153234, 1.58411],
                       [0, 0, 0, 1]]
    TVehicle2Imu = [[1, 0, 0, -0.07],
                    [0, 1, 0, 0],
                    [0, 0, 1, 1.7],
                    [0, 0, 0, 1]]
    T = np.matmul(np.linalg.inv(TVehicle2Imu), TVehicle2Stereo)
    # T = np.matmul((TVehicle2Imu), np.linalg.inv(TVehicle2Stereo))
    # T = np.matmul(np.linalg.inv(TVehicle2Stereo), (TVehicle2Imu))
    print(T)
    # print(np.linalg.inv(T))
