
import numpy as np
import csv

if __name__ == "__main__":

    # input
    globalPosePath = "/home/wzb/datasets/KAIST/urban39/global_pose.csv"
    # output
    timeStampPath = "/home/wzb/datasets/KAIST/urban39/myData/poseTum/timeStamp.txt"
    poseKittiPath = "/home/wzb/datasets/KAIST/urban39/myData/poseTum/poseKitti.txt"

    timeStampWriter = open(timeStampPath, "w")
    poseKittiWriter = open(poseKittiPath, "w")
    with open(globalPosePath, 'r') as f:
        reader = csv.reader(f)
        print(type(reader))

        for row in reader:
            timeStampWriter.write(str(float(row[0]) * 1e-9) + "\n")
            for i in range(1, 13):
                poseKittiWriter.write(row[i])
                if i != 12:
                    poseKittiWriter.write(" ")
            poseKittiWriter.write("\n")