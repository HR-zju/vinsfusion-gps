# eruoc 

## IMU+mono

# KAIST Example

### 5.1 mono + imu

    roslaunch vins vins_rviz.launch
    rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/kaist/kaist_mono_imu_config.yaml
    rosbag play YOUR_DATASET_FOLDER/yourbag.bag

### 5.2 mono + imu + gps(紧耦合)

same as EuRoC Example(using kaist_mono_imu_localFusionGps_config.yaml as the input parameter). 

    roslaunch vins vins_rviz.launch
    rosrun vins vins_node src/vinsfusion-gps/config/KAIST/kaist_mono_imu_localFusionGps_config.yaml
    rosbag play YOUR_DATASET_FOLDER/yourbag.bag

### 5.2 mono + imu + gps(松耦合)

same as EuRoC Example(using kaist_mono_imu_localFusionGps_config.yaml as the input parameter). 

    roslaunch vins vins_rviz.launch
    rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/kaist/kaist_mono_imu_config.yaml
    rosrun global_fusion global_fusion_node
    rosbag play YOUR_DATASET_FOLDER/yourbag.bag

## 



# ZJUT

## Vins-mono

~~~
roslaunch vins vins_rviz.launch
~~~

~~~
rosrun vins vins_node src/vinsfusion-gps/config/zjut/zjut_mono_imu_config.yaml 
~~~

## Vins-mono + GPS

~~~
roslaunch vins vins_rviz.launch
~~~

~~~
rosrun vins vins_node src/vinsfusion-gps/config/zjut/zjut_mono_imu_config.yaml 
~~~

~~~
rosrun global_fusion global_fusion_node
~~~

