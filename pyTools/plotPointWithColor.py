import folium
import csv
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

def loadKAISTGPS(GPSCSVPath):
    x = []
    y = []
    var = []
    with open(GPSCSVPath, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            x.append(float(row[1]))
            y.append(float(row[2]))
            var.append(float(row[4]))
    return x, y, var


if __name__ == "__main__":

    # set the path
    GPSCSVPath = "/home/wzb/datasets/KAIST/urban39/sensor_data/gps.csv"
    # VRSGPSCSVPath = "/home/wzb/datasets/KAIST/urban39/sensor_data/vrs_gps.csv"

    x, y, var = loadKAISTGPS(GPSCSVPath)
    # points_vrsgps = loadKAISTGPS(VRSGPSCSVPath)

    for index in range(1000):
        c = ''
        if var[index] < 10:
            c = 'pink'
        if var[index] >= 10 and var[index] < 20:
            c = 'red'
        if var[index] >= 20 and var[index] < 30:
            c = 'purple'
        if var[index] >= 30:
            c = 'black'
        plt.scatter([x[index]], [y[index]], c=c)
    plt.show()

    # m = folium.Map(location=[37.397253103333334, 127.10720864499999], zoom_start=15)
    #
    # # for index in range(len(points_gps)):
    # for index in range(1000):
    #     """
    #     'red', 'darkred',  'lightred', 'orange', 'beige',
    #                  'green', 'darkgreen', 'lightgreen',
    #                  'blue', 'darkblue', 'cadetblue', 'lightblue',
    #                  'purple',  'darkpurple', 'pink',
    #                  'white', 'gray', 'lightgray', 'black'
    #     """
    #     c = ''
    #     if var[index] < 10:
    #         c = 'pink'
    #     if var[index] >= 10 and var[index] < 20:
    #         c = 'red'
    #     if var[index] >= 20 and var[index] < 30:
    #         c = 'purple'
    #     if var[index] >= 30:
    #         c = 'black'
    #
    #     folium.Marker(points_gps[index],
    #                   popup=('patient{} \n 74contacts'.format(index)),
    #                   icon=folium.Icon(color=c, icon='info-sign')).add_to(m)
    #
    # # folium.PolyLine(points_gps, color='red').add_to(m)
    # # folium.PolyLine(points_vrsgps, color='green').add_to(m)
    # m.save("./map_point.html")
    # print("the process has been finished!")





