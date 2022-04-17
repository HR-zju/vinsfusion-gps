
import numpy as np
import csv

if __name__ == "__main__":

    # input
    vioCsvPath = "/home/wzb/project/huawei/vo_gps/catkin_ws_vio_gps_error/src/VINS-Fusion/output/vio.csv"
    # output
    tumPosePath = "/home/wzb/project/huawei/vo_gps/catkin_ws_vio_gps_error/src/VINS-Fusion/output/tum_vio_gps_sw20_weight500_rb_offline.txt"

    
    tumPoseWriter = open(tumPosePath, "w")
    with open(vioCsvPath, 'r') as f:
        reader = csv.reader(f)
        print(type(reader))

        for row in reader:
            for i in range(0, 8):
                if i == 0:
                    tumPoseWriter.write(str(float(row[i]) * 1e-9))
                else:
                    tumPoseWriter.write(row[i])
                if i != 7:
                    tumPoseWriter.write(" ")
            tumPoseWriter.write("\n")

    print("the process has been finished!")