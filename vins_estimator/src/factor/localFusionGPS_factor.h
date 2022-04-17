//
// Created by wzb on 2020/12/28.
//

#ifndef SRC_LOCALFUSIONGPS_FACTOR_H
#define SRC_LOCALFUSIONGPS_FACTOR_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>

// TODO error? input var or accuracy
struct TErrorVarXYZ
{
//    TErrorVarXYZ(const double& t_x, const double& t_y, const double& t_z,
//                 const double& var_x, const double& var_y, const double& var_z)
//            :t_x(t_x), t_y(t_y), t_z(t_z), var_x(var_x), var_y(var_y), var_z(var_z){}
//
//    template <typename T>
//    bool operator()(const T* tj, T* residuals) const
//    {
//        residuals[0] = (tj[0] - T(t_x)) / T(var_x);
//        residuals[1] = (tj[0] - T(t_y)) / T(var_y);
//        residuals[2] = (tj[0] - T(t_z)) / T(var_z);
//
//        return true;
//    }
//
//    static ceres::CostFunction* Create(const double& t_x, const double& t_y, const double& t_z,
//                                       const double& var_x, const double& var_y, const double& var_z)
//    {
//        return (new ceres::AutoDiffCostFunction<TErrorVarXYZ, 3, 3>(
//                new TErrorVarXYZ(t_x, t_y, t_z, var_x, var_y, var_z)));
//    }
//
//    double t_x, t_y, t_z, var_x, var_y, var_z;


    TErrorVarXYZ(const double& t_x, const double& t_y, const double& t_z,
                 const double& var_x, const double& var_y, const double& var_z)
            :t_x(t_x), t_y(t_y), t_z(t_z), var_x(var_x), var_y(var_y), var_z(var_z){}

    template <typename T>
    bool operator()(const T* tj, T* residuals) const
    {
        residuals[0] = (tj[0] - T(t_x)) / T(var_x);
        residuals[1] = (tj[1] - T(t_y)) / T(var_y);
        residuals[2] = (tj[2] - T(t_z)) / T(var_z);

        return true;
    }

    static ceres::CostFunction* Create(const double& t_x, const double& t_y, const double& t_z,
                                       const double& var_x, const double& var_y, const double& var_z)
    {
        return (new ceres::AutoDiffCostFunction<TErrorVarXYZ, 3, 7>(
                new TErrorVarXYZ(t_x, t_y, t_z, var_x, var_y, var_z)));
    }

    double t_x, t_y, t_z, var_x, var_y, var_z;




};

#endif //SRC_LOCALFUSIONGPS_FACTOR_H
