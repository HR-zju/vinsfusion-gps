//
// Created by wzb on 2020/12/29.
//

#ifndef SRC_GPSUTILS_H
#define SRC_GPSUTILS_H

#include <iostream>
#include <vector>
#include <cmath>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace GPSUtils{

    /*
     * basic class
     */
    class GPSData{
    public:
        double mT;
        std::vector<double> mvData;      // vector size: 6, elem: (x, y, z, var_latitude, var_longitude, var_altitude)    TODO var
    public:
        GPSData(const double& t, const std::vector<double>& data){
            mvData.resize(6);
            mT = t;
            mvData.assign(data.begin(), data.end());//重新賦值
        }
        GPSData(){
            mvData.resize(6);
            mT = -1;
        }
        void assign(const GPSData& gpsData){
            mT = gpsData.mT;
            mvData.assign(gpsData.mvData.begin(), gpsData.mvData.end());
        }
    };

    /*
     * interpolation
     */
    bool interpolationGPSNode(const GPSData& gpsData1, const GPSData& gpsData2, const double& timeStamp_target,
                              GPSData& pOutputNode, const double& timeThre_interpolation = -1);

    /*
     * align gps and other poses(2020ICRA Intermittent GPS-aided VIO: Online Initialization and Calibration)(GPS to VIO)
     */
//    void alignGPSAndOtherPoses_twoFrame();
//    void getP_VO2GPS(const Eigen::Vector3d& P_E1, const Eigen::Vector3d& P_E2,
//                      const Eigen::Vector3d& P_V1, const Eigen::Vector3d& P_V2);
    void getR_VO2GPS(const Eigen::Vector3d& P_E1, const Eigen::Vector3d& P_E2,
                      const Eigen::Vector3d& P_V1, const Eigen::Vector3d& P_V2,
                      double& cosTheta, double& sinTheta);  // only raw
    void getP_VO2GPS(const Eigen::Vector3d& P_E, const Eigen::Vector3d& P_V,
                     const Eigen::Matrix3d& R_VO2GPS, Eigen::Vector3d& P_VO2GPS);
    void getR_GPS2VO(const Eigen::Vector3d& P_E1, const Eigen::Vector3d& P_E2,
                     const Eigen::Vector3d& P_V1, const Eigen::Vector3d& P_V2,
                     double& cosTheta, double& sinTheta);
    void getP_GPS2VO(const Eigen::Vector3d& P_E, const Eigen::Vector3d& P_V,
                     const Eigen::Matrix3d& R_GPS2VO, Eigen::Vector3d& P_GPS2VO);
    void getThetaFromCosAndSin(const double& cos, const double& sin, double& theta);
    void twoStageEKFToUpdateThetaGPS2VO_first(const Eigen::Vector3d& P_E_sub, const Eigen::Vector3d& P_V_sub, const Eigen::Vector3d& var_gps_sub_xyz,
                                              const double& inputThetaGPS2VO, const double& var_inputTheta,
                                              double& outputThetaGPS2VO, double& var_outputTheta);
    void twoStageEKFToUpdateThetaGPS2VO_second(const Eigen::Vector3d& P_E_sub, const Eigen::Vector3d& P_V_sub, const Eigen::Vector3d& var_gps_sub_xyz,
                                              const double& inputThetaGPS2VO, const double& var_inputTheta,
                                              double& outputThetaGPS2VO, double& var_outputTheta);
    void EKFToUpdateP_GPS2VO(const Eigen::Vector3d& P_E, const Eigen::Vector3d& P_V, const Eigen::Matrix3d& var_gps_xyz,
                             const Eigen::Vector3d& input_P_E2V, const Eigen::Matrix3d& var_input_P_E2V,
                             const Eigen::Matrix3d& R_E2V,
                             Eigen::Vector3d& output_P_E2V, Eigen::Matrix3d& var_output_P_E2V);

    /*
     * list node
     */
    /*
     * basic class
     */
//    class GPSListNode{
//    public:
//        double mT;
//        std::vector<double> mvData;   // vector size: 6, elem: (latitude, longitude, altitude, var_latitude, var_longitude, var_altitude) or (x, y, z, var_latitude, var_longitude, var_altitude)
//        GPSListNode* mNext;
//    public:
//        GPSListNode(const double& t, const std::vector<double>& data, GPSListNode* next){
//            mvData.resize(6);
//            mT = t;
//            mvData.assign(data.begin(), data.end());
//            mNext = next;
//        }
//        GPSListNode(){
//            mvData.resize(6);
//            mT = -1;
//            mNext = nullptr;
//        }
//        void assign(const GPSListNode* node){
//            mT = node->mT;
//            mvData.assign(node->mvData.begin(), node->mvData.end());
//            mNext = node->mNext;
//        }
//    };
//
//    /*
//     * interpolation
//     */
//    /**
//     * interpolate the near GPSNode using the far GPSNode
//     * @param pNode1 first GPSNode
//     * @param pNode2 second GPSNode
//     * @param timeStamp_target
//     * @param timeThre_interpolation
//     * @return
//     */
//    bool interpolationGPSNode(const GPSListNode* pNode1, const GPSListNode* pNode2, const double& timeStamp_target,
//                              GPSListNode* pOutputNode, const double& timeThre_interpolation = -1);
//
//    /*
//     * update GPS node list
//     */
//    /**
//     * update GPS node list using input timeStamp, make (t1 >= GPSBufDummyNode->mNext->mT && t1 <= GPSBufDummyNode->mNext->mNext->mT)
//     * @param t1: input time stamp
//     * @param pGPSBufDummyNode
//     * @param pGPSBufLastNode
//     * @return whether select the two GPS node
//     */
//    bool selectGPSNodeFromList(const double& t1, GPSListNode* pGPSBufDummyNode, GPSListNode* pGPSBufLastNode);

}

#endif //SRC_GPSUTILS_H
