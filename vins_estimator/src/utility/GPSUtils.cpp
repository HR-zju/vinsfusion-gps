//
// Created by wzb on 2020/12/29.
//

#include "GPSUtils.h"
using namespace GPSUtils;

/*
 * interpolation
 */
bool GPSUtils::interpolationGPSNode(const GPSData& gpsData1, const GPSData& gpsData2, const double& timeStamp_target,
                          GPSData& pOutputNode, const double& timeThre_interpolation){
    if(gpsData1.mT > timeStamp_target || gpsData2.mT < timeStamp_target){
        std::cout<<"error at input GPS timeStamp"<<std::endl;
        return false;
    }

    double time_interpolationDiff;
    double time_diff1 = timeStamp_target - gpsData1.mT;
    double time_diff2 = gpsData2.mT - timeStamp_target;

    if(time_diff1 <= time_diff2){   // interpolate GPSNode1 using GPSNode2
//        time_interpolationDiff = time_diff1;
//        if(fabs(timeThre_interpolation + 1) > 1e-6){
//            if(time_interpolationDiff >= timeThre_interpolation){
//                return false;
//            }
//        }
        double weight = time_diff1 / (gpsData2.mT - gpsData1.mT);
        for(size_t i = 0; i < 3; i++){
            pOutputNode.mvData[i] = gpsData1.mvData[i] + weight * (gpsData2.mvData[i] - gpsData1.mvData[i]);
        }
        for(size_t i = 3; i < 6; i++){
            pOutputNode.mvData[i] = gpsData1.mvData[i] + weight * weight * (gpsData1.mvData[i] + gpsData2.mvData[i]);
        }
    }else{  // interpolate GPSNode2 using GPSNode1
//        time_interpolationDiff = time_diff2;
//        if(fabs(timeThre_interpolation + 1) > 1e-6){
//            if(time_interpolationDiff >= timeThre_interpolation){
//                return false;
//            }
//        }
        double weight = time_diff2 / (gpsData2.mT - gpsData1.mT);
        for(size_t i = 0; i < 3; i++){
            pOutputNode.mvData[i] = gpsData2.mvData[i] - weight * (gpsData2.mvData[i] - gpsData1.mvData[i]);
        }
        for(size_t i = 3; i < 6; i++){
            pOutputNode.mvData[i] = gpsData2.mvData[i] + weight * weight * (gpsData1.mvData[i] + gpsData2.mvData[i]);
        }
    }
    pOutputNode.mT = timeStamp_target;

    return true;
}


/*
 * align gps and other poses
 */
void GPSUtils::getR_VO2GPS(const Eigen::Vector3d& P_E1, const Eigen::Vector3d& P_E2,
                 const Eigen::Vector3d& P_V1, const Eigen::Vector3d& P_V2,
                 double& cosTheta, double& sinTheta){
    Eigen::Vector3d P_E_sub = P_E2 - P_E1;
    Eigen::Vector3d P_V_sub = P_V2 - P_V1;
    double target_cosTheta = (P_E_sub[0] * P_V_sub[0] + P_E_sub[1] * P_V_sub[1]) / (pow(P_E_sub[0], 2) + pow(P_E_sub[1], 2));
    double target_sinTheta = (P_E_sub[0] * P_V_sub[1] - P_E_sub[1] * P_V_sub[0]) / (pow(P_E_sub[0], 2) + pow(P_E_sub[1], 2));
    // lagrangian miltipliers
//    double lamda1 = sqrt(pow(target_cosTheta, 2) + pow(target_sinTheta, 2)) - 1;
//    double lamda2 = -sqrt(pow(target_cosTheta, 2) + pow(target_sinTheta, 2)) - 1;
    double temp = sqrt(pow(target_cosTheta, 2) + pow(target_sinTheta, 2));
    cosTheta = target_cosTheta / temp;
    sinTheta = target_sinTheta / temp;
}

void GPSUtils::getP_VO2GPS(const Eigen::Vector3d& P_E, const Eigen::Vector3d& P_V,
                 const Eigen::Matrix3d& R_VO2GPS, Eigen::Vector3d& P_VO2GPS){
    P_VO2GPS = P_V - R_VO2GPS * P_E;
}

void GPSUtils::getR_GPS2VO(const Eigen::Vector3d& P_E1, const Eigen::Vector3d& P_E2,
                 const Eigen::Vector3d& P_V1, const Eigen::Vector3d& P_V2,
                 double& cosTheta, double& sinTheta){
    Eigen::Vector3d P_E_sub = P_E2 - P_E1;
    Eigen::Vector3d P_V_sub = P_V2 - P_V1;
    double target_cosTheta = (P_E_sub[0] * P_V_sub[0] + P_E_sub[1] * P_V_sub[1]) / (pow(P_V_sub[0], 2) + pow(P_V_sub[1], 2));
    double target_sinTheta = (P_V_sub[0] * P_E_sub[1] - P_V_sub[1] * P_E_sub[0]) / (pow(P_V_sub[0], 2) + pow(P_V_sub[1], 2));
    // lagrangian miltipliers
//    double lamda1 = sqrt(pow(target_cosTheta, 2) + pow(target_sinTheta, 2)) - 1;
//    double lamda2 = -sqrt(pow(target_cosTheta, 2) + pow(target_sinTheta, 2)) - 1;
    double temp = sqrt(pow(target_cosTheta, 2) + pow(target_sinTheta, 2));
    cosTheta = target_cosTheta / temp;
    sinTheta = target_sinTheta / temp;
}

void GPSUtils::getP_GPS2VO(const Eigen::Vector3d& P_E, const Eigen::Vector3d& P_V,
                 const Eigen::Matrix3d& R_GPS2VO, Eigen::Vector3d& P_GPS2VO){
    P_GPS2VO = P_E - R_GPS2VO * P_V;
}

void GPSUtils::getThetaFromCosAndSin(const double& cos, const double& sin, double& theta){
    if(sin >= 0)
        theta = acos(cos);
    else
        theta = -acos(cos);
}

void GPSUtils::twoStageEKFToUpdateThetaGPS2VO_first(const Eigen::Vector3d& P_E_sub, const Eigen::Vector3d& P_V_sub, const Eigen::Vector3d& var_gps_sub_xyz,
                                                    const double& inputThetaGPS2VO, const double& var_inputTheta,
                                                    double& outputThetaGPS2VO, double& var_outputTheta){
    double Gk = -sin(inputThetaGPS2VO) * P_V_sub[0] - cos(inputThetaGPS2VO) * P_V_sub[1];
    double Pk_predict = var_inputTheta;
    double theta_predict = inputThetaGPS2VO;
    double Kk = Pk_predict * Gk / (Gk * Pk_predict * Gk + var_gps_sub_xyz[0]);
    var_outputTheta = (1 - Kk * Gk) * Pk_predict;
    outputThetaGPS2VO = inputThetaGPS2VO + Kk * (P_E_sub[0] - (cos(theta_predict) * P_V_sub[0] - sin(theta_predict) * P_V_sub[1]));
}

void GPSUtils::twoStageEKFToUpdateThetaGPS2VO_second(const Eigen::Vector3d& P_E_sub, const Eigen::Vector3d& P_V_sub, const Eigen::Vector3d& var_gps_sub_xyz,
                                           const double& inputThetaGPS2VO, const double& var_inputTheta,
                                           double& outputThetaGPS2VO, double& var_outputTheta){
    double Gk = cos(inputThetaGPS2VO) * P_V_sub[0] - sin(inputThetaGPS2VO) * P_V_sub[1];
    double Pk_predict = var_inputTheta;
    double theta_predict = inputThetaGPS2VO;
    double Kk = Pk_predict * Gk / (Gk * Pk_predict * Gk + var_gps_sub_xyz[1]);
    var_outputTheta = (1 - Kk * Gk) * Pk_predict;
    outputThetaGPS2VO = inputThetaGPS2VO + Kk * (P_E_sub[1] - (sin(theta_predict) * P_V_sub[0] + cos(theta_predict) * P_V_sub[1]));
}

void GPSUtils::EKFToUpdateP_GPS2VO(const Eigen::Vector3d& P_E, const Eigen::Vector3d& P_V, const Eigen::Matrix3d& var_gps_xyz,
                         const Eigen::Vector3d& input_P_E2V, const Eigen::Matrix3d& var_input_P_E2V,
                         const Eigen::Matrix3d& R_E2V,
                         Eigen::Vector3d& output_P_E2V, Eigen::Matrix3d& var_output_P_E2V){
    Eigen::Matrix3d Gk;
    Gk << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    Eigen::Matrix3d Pk_predict = var_input_P_E2V;
    Eigen::Vector3d P_E2V_predict = input_P_E2V;
    Eigen::Matrix3d Kk = Pk_predict * Gk.transpose() * (Gk * Pk_predict * Gk.transpose() + var_gps_xyz).inverse();
    var_output_P_E2V = (Eigen::Matrix3d::Identity(3, 3) - Kk * Gk) * Pk_predict;
    output_P_E2V = input_P_E2V + Kk * (P_E - (P_E2V_predict + R_E2V * P_V));
}



/*
 * GPS list node
 */
///*
//* interpolation
//*/
//bool GPSUtils::interpolationGPSNode(const GPSListNode* pNode1, const GPSListNode* pNode2, const double& timeStamp_target,
//                                    GPSListNode* pOutputNode, const double& timeThre_interpolation){
//    if(pNode1->mT > timeStamp_target || pNode2->mT < timeStamp_target){
//        std::cout<<"error at input GPS node"<<std::endl;
//    }
//
//    double time_interpolationDiff;
//    double time_diff1 = timeStamp_target - pNode1->mT;
//    double time_diff2 = pNode2->mT - timeStamp_target;
//
//    if(time_diff1 <= time_diff2){   // interpolate GPSNode1 using GPSNode2
//        time_interpolationDiff = time_diff1;
////        if(fabs(timeThre_interpolation + 1) > 1e-6){
////            if(time_interpolationDiff >= timeThre_interpolation){
////                return false;
////            }
////        }
//        double weight = time_diff1 / (pNode2->mT - pNode1->mT);
//        for(size_t i = 0; i < 3; i++){
//            pOutputNode->mvData[i] = pNode1->mvData[i] + weight * (pNode2->mvData[i] - pNode1->mvData[i]);
//        }
//        for(size_t i = 3; i < 6; i++){
//            pOutputNode->mvData[i] = pNode1->mvData[i] + weight * weight * (pNode1->mvData[i] + pNode2->mvData[i]);
//        }
//    }else{  // interpolate GPSNode2 using GPSNode1
//        time_interpolationDiff = time_diff2;
////        if(fabs(timeThre_interpolation + 1) > 1e-6){
////            if(time_interpolationDiff >= timeThre_interpolation){
////                return false;
////            }
////        }
//        double weight = time_diff2 / (pNode2->mT - pNode1->mT);
//        for(size_t i = 0; i < 3; i++){
//            pOutputNode->mvData[i] = pNode2->mvData[i] - weight * (pNode2->mvData[i] - pNode1->mvData[i]);
//        }
//        for(size_t i = 3; i < 6; i++){
//            pOutputNode->mvData[i] = pNode2->mvData[i] + weight * weight * (pNode1->mvData[i] + pNode2->mvData[i]);
//        }
//    }
//    pOutputNode->mT = timeStamp_target;
//
//    return true;
//}
//
//
///*
// * update GPS node list
// */
//bool GPSUtils::selectGPSNodeFromList(const double& t1, GPSListNode* pGPSBufDummyNode, GPSListNode* pGPSBufLastNode){
//    std::cout<<__FUNCTION__ <<" "<<__LINE__<<std::endl;     // TODO debug
//    if(pGPSBufLastNode == pGPSBufDummyNode){
//        std::cout<<"not receive local fusion gps"<<std::endl;
//        return false;
//    }
//    if(pGPSBufDummyNode->mNext == nullptr || pGPSBufDummyNode->mNext->mNext == nullptr){
//        std::cout<<"not receive enough local fusion GPS data"<<std::endl;
//        return false;
//    }
//
//    // get two local fusion GPS data around feature timestamp
//    if(t1 <= pGPSBufLastNode->mT && t1 >= pGPSBufDummyNode->mNext->mT){
//        while(true){
//            if(t1 >= pGPSBufDummyNode->mNext->mT && t1 <= pGPSBufDummyNode->mNext->mNext->mT){
//                break;
//            }
//            GPSListNode* deleteNode = pGPSBufDummyNode->mNext;
//            pGPSBufDummyNode->mNext = deleteNode->mNext;
//            delete deleteNode;
//        }
//    }else if(t1 > pGPSBufLastNode->mT){
//        std::cout<<"wait for feature"<<std::endl;
//        return false;
//    }else{
//        std::cout<<"wait for local fusion gps"<<std::endl;
//        return false;
//    }
//
//    std::cout<<__FUNCTION__ <<" "<<__LINE__<<std::endl;     // TODO debug
//
//    return true;
//}