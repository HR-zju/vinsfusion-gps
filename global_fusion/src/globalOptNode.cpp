/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "ros/ros.h"
#include "globalOpt.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <queue>
#include <mutex>

GlobalOptimization globalEstimator;
ros::Publisher pub_global_odometry, pub_global_path, pub_car;
nav_msgs::Path *global_path;
double last_vio_t = -1;
std::queue<sensor_msgs::NavSatFix> gpsQueue;
//std::queue<sensor_msgs::NavSatFixConstPtr> gpsQueue;
std::mutex m_buf;

inline void m_assignment(sensor_msgs::NavSatFixConstPtr const_ptr, sensor_msgs::NavSatFixPtr &ptr){
    ptr->header = const_ptr->header;
    ptr->altitude = const_ptr->altitude;
    ptr->longitude = const_ptr->longitude;
    ptr->latitude = const_ptr->latitude;
    ptr->position_covariance = const_ptr->position_covariance;
    ptr->position_covariance_type = const_ptr->position_covariance_type;
}

void publish_car_model(double t, Eigen::Vector3d t_w_car, Eigen::Quaterniond q_w_car)
{
    visualization_msgs::MarkerArray markerArray_msg;
    visualization_msgs::Marker car_mesh;
    car_mesh.header.stamp = ros::Time(t);
    car_mesh.header.frame_id = "map";
    car_mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    car_mesh.action = visualization_msgs::Marker::ADD;
    car_mesh.id = 0;

    car_mesh.mesh_resource = "package://global_fusion/models/car.dae";

    Eigen::Matrix3d rot;
    rot << 0, 0, -1, 0, -1, 0, -1, 0, 0;
    
    Eigen::Quaterniond Q;
    Q = q_w_car * rot; 
    car_mesh.pose.position.x    = t_w_car.x();
    car_mesh.pose.position.y    = t_w_car.y();
    car_mesh.pose.position.z    = t_w_car.z();
    car_mesh.pose.orientation.w = Q.w();
    car_mesh.pose.orientation.x = Q.x();
    car_mesh.pose.orientation.y = Q.y();
    car_mesh.pose.orientation.z = Q.z();

    car_mesh.color.a = 1.0;
    car_mesh.color.r = 1.0;
    car_mesh.color.g = 0.0;
    car_mesh.color.b = 0.0;

    float major_scale = 2.0;

    car_mesh.scale.x = major_scale;
    car_mesh.scale.y = major_scale;
    car_mesh.scale.z = major_scale;
    markerArray_msg.markers.push_back(car_mesh);
    pub_car.publish(markerArray_msg);
}

bool first_gps = false;
sensor_msgs::NavSatFix temp_GPS;

void GPS_callback(const sensor_msgs::NavSatFixConstPtr &GPS_msg)
{
    //printf("gps_callback! \n");
    m_buf.lock();
    double xyz[3];
    double latitude = GPS_msg->latitude;
    double longitude = GPS_msg->longitude;
    double altitude = GPS_msg->altitude;
    if(!first_gps){//定原点
        globalEstimator.GPS2XYZ(latitude, longitude, altitude, xyz);
        first_gps = true;
    }
    //<<<<<<<<<<<<<<<<<<<<<<<<<<
    //插值
    else{
        for(int i=1; i<=39; i++){
            sensor_msgs::NavSatFix d_GPS_msg;
            d_GPS_msg = *GPS_msg;
//            m_assignment(GPS_msg, d_GPS_msg);
//            *d_GPS_msg = *GPS_msg;
//            temp_GPS = *GPS_msg;
//            temp_GPS[i].header.stamp = GPS_msg->header.stamp;
            double b_t = gpsQueue.back().header.stamp.toSec();
            double temp_t = GPS_msg->header.stamp.toSec() - b_t;
            d_GPS_msg.header.stamp = ros::Time().fromSec( b_t + temp_t * i / 40);
            d_GPS_msg.latitude = gpsQueue.back().latitude + (GPS_msg->latitude - gpsQueue.back().latitude) * i / 40;
            d_GPS_msg.longitude = gpsQueue.back().longitude + (GPS_msg->longitude - gpsQueue.back().longitude) * i / 40;
            d_GPS_msg.altitude = gpsQueue.back().altitude + (GPS_msg->altitude - gpsQueue.back().altitude) * i / 40;

//            *d_GPS_msg = temp_GPS;
//            d_GPS_msg->latitude = &temp_GPS.latitude;
            gpsQueue.push(d_GPS_msg);
        }
    }
    //<<<<<<<<<<<<<<<<<<<<<<<<<<
    sensor_msgs::NavSatFix temp_GPS_ptr;
//    *temp_GPS_ptr = *GPS_msg;
//    m_assignment(GPS_msg, temp_GPS_ptr);
    temp_GPS_ptr = * GPS_msg;
    gpsQueue.push(temp_GPS_ptr);

//    GlobalOptimization G();


    m_buf.unlock();
}

void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //printf("vio_callback! \n");
    double t = pose_msg->header.stamp.toSec();
    last_vio_t = t;
    Eigen::Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Eigen::Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;
    globalEstimator.inputOdom(t, vio_t, vio_q);


    m_buf.lock();
    while(!gpsQueue.empty())
    {
        sensor_msgs::NavSatFix GPS_msg = gpsQueue.front();
        double gps_t = GPS_msg.header.stamp.toSec();
//        printf("vio t: %f, gps t: %f \n", t, gps_t);
        // 100ms sync tolerance(cam publish: 10hz 100ms, gps publish: 5hz, 200ms)(raw sync tolerance: 10ms)
        if(gps_t >= t - 0.01 && gps_t <= t + 0.01)  // TODO parameter: sync tolerance time
        {
            //printf("receive GPS with timestamp %f\n", GPS_msg->header.stamp.toSec());
            double latitude = GPS_msg.latitude;
            double longitude = GPS_msg.longitude;
            double altitude = GPS_msg.altitude;
            //int numSats = GPS_msg->status.service;
            double pos_accuracy = GPS_msg.position_covariance[0];
            if(pos_accuracy <= 0)
                pos_accuracy = 1;
            //printf("receive covariance %lf \n", pos_accuracy);
            //if(GPS_msg->status.status > 8)
                globalEstimator.inputGPS(t, latitude, longitude, altitude, pos_accuracy);
            gpsQueue.pop();
            break;
        }
        else if(gps_t < t - 0.01)//
            gpsQueue.pop();
        else if(gps_t > t + 0.01)
            break;
    }
    m_buf.unlock();

    Eigen::Vector3d global_t;
    Eigen:: Quaterniond global_q;
    globalEstimator.getGlobalOdom(global_t, global_q);

    nav_msgs::Odometry odometry;
    odometry.header = pose_msg->header;
    odometry.header.frame_id = "map";
    odometry.child_frame_id = "map";
    odometry.pose.pose.position.x = global_t.x();
    odometry.pose.pose.position.y = global_t.y();
    odometry.pose.pose.position.z = global_t.z();
    odometry.pose.pose.orientation.x = global_q.x();
    odometry.pose.pose.orientation.y = global_q.y();
    odometry.pose.pose.orientation.z = global_q.z();
    odometry.pose.pose.orientation.w = global_q.w();
    pub_global_odometry.publish(odometry);
    pub_global_path.publish(*global_path);
    publish_car_model(t, global_t, global_q);


    // write result to file tum格式
    std::ofstream foutC("/home/myk/slam_lrn/output/vio_global.txt", ios::app);
    foutC.setf(ios::fixed, ios::floatfield);
    foutC.precision(0);
    foutC << pose_msg->header.stamp.toSec() * 1e9 << " ";
    foutC.precision(6);
    foutC << global_t.x() << " "
            << global_t.y() << " "
            << global_t.z() << " "
            << global_q.x() << " "
            << global_q.y() << " "
            << global_q.z() << " "
            << global_q.w() << endl;
    foutC.close();
}
//int xyz_count = 0;
void xyz_callback(const geometry_msgs::PointStamped &_pose_msg)
{
    geometry_msgs::PointStamped xyz;
    double lla[3];
    m_buf.lock();
    globalEstimator.XYZ2GPS(lla, _pose_msg.point.x, _pose_msg.point.y, _pose_msg.point.z);
    m_buf.unlock();

    printf("第%d个点的xyz坐标为(%f, %f, %f) GPS为(%f, %f, %f)", _pose_msg.header.seq, _pose_msg.point.x, _pose_msg.point.y, _pose_msg.point.z, lla[0], lla[1], lla[2]);
}


void command(){
    while (1)
    {
        char c = getchar();
        if(c == 'g')
        {
            m_buf.lock();
            globalEstimator.output_global_pose();
            globalEstimator.output_global_truth();
            m_buf.unlock();
            printf("save vio_gps_graph to /home/myk/slam_lrn/output/final_global.txt \n");
            printf("save gps_truth to /home/myk/slam_lrn/output/global_truth.txt \n");
        }
    }
    std::chrono::milliseconds dura(5);
    std::this_thread::sleep_for(dura);
}

int main(int argc, char **argv)
{
    if(argc != 2)
    {
        printf("please intput: rosrun global_fusion global_fusion_node /gps_topic \n"
               "for example: rosrun global_fusion global_fusion_node /gps/fix \n"
               );
        return 1;
    }

    string gps_topic = argv[1];
    printf("gps_topic: %s\n", argv[1]);

    ros::init(argc, argv, "globalEstimator");
    ros::NodeHandle n("~");

    global_path = &globalEstimator.global_path;



    ros::Subscriber sub_GPS = n.subscribe(gps_topic, 100, GPS_callback);
    ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 100, vio_callback);
    ros::Subscriber sub_xyz = n.subscribe("/clicked_point", 100, xyz_callback);

    pub_global_path = n.advertise<nav_msgs::Path>("global_path", 100);
    pub_global_odometry = n.advertise<nav_msgs::Odometry>("global_odometry", 100);
    pub_car = n.advertise<visualization_msgs::MarkerArray>("car_model", 1000);

    std::thread keyboard_command_process;
    keyboard_command_process = std::thread(command);

    ros::spin();
    return 0;
}
//this_type(r).swap(*this)
//global_truth