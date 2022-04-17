import folium
import csv

def loadKAISTGPS(GPSCSVPath):
    points = []
    with open(GPSCSVPath, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            points.append([float(row[1]), float(row[2])])
    return points


if __name__ == "__main__":

    # set the path
    GPSCSVPath = "/home/wzb/datasets/KAIST/urban39/sensor_data/gps.csv"
    VRSGPSCSVPath = "/home/wzb/datasets/KAIST/urban39/sensor_data/vrs_gps.csv"

    points_gps = loadKAISTGPS(GPSCSVPath)
    points_vrsgps = loadKAISTGPS(VRSGPSCSVPath)

    m = folium.Map(location=[37.397253103333334, 127.10720864499999], zoom_start=15)

    # for index, point in enumerate(points_gps):
    #     folium.Marker(point,
    #                   popup=('patient{} \n 74contacts'.format(index)),
    #                   icon=folium.Icon(color='green', icon='plus')).add_to(m)

    folium.PolyLine(points_gps, color='red').add_to(m)
    folium.PolyLine(points_vrsgps, color='green').add_to(m)
    m.save("./map.html")
    print("the process has been finished!")




    # m = folium.Map(location=[37.4601908, 126.4406957],
    #                zoom_start=15)
    #
    # place_lat = [37.4601928, 37.4593108, 37.4641108, 37.4611508]
    # place_lng = [126.4406957, 126.4432957, 126.4476917, 126.4423957]
    #
    # points = []
    # for i in range(len(place_lat)):
    #     points.append([place_lat[i], place_lng[i]])
    #
    # for index, lat in enumerate(place_lat):
    #     folium.Marker([lat,
    #                    place_lng[index]],
    #                   popup=('patient{} \n 74contacts'.format(index)),
    #                   icon=folium.Icon(color='green', icon='plus')).add_to(m)
    # print(points)
    # folium.PolyLine(points, color='red').add_to(m)
    #
    # m.save("./map.html")
