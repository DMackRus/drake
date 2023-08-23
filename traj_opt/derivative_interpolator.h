#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include "drake/traj_opt/inverse_dynamics_partials.h"
#include "drake/common/eigen_types.h"
//#include <Eigen/Dense>

struct derivative_interpolator {
    std::string keyPointMethod;
    int minN;
    int maxN;
    double jerkThreshold;
    double acellThreshold;
    double velChangeSensitivity;
    double iterativeErrorThreshold;
};

//namespace drake {
//namespace traj_opt {

template <typename T>
class DerivativeInterpolator {

public:
    DerivativeInterpolator(){

    }

    std::vector<int> ComputeKeypoints(derivative_interpolator interpolator, int horizon) const;

//    void SavePartials(std::string filename, InverseDynamicsPartials<T> *id_partials) const;
//    void GetApproximateDerivsOverTrajectory(derivative_interpolator interpolator);

private:

//    void ComputeDerivsAtSpecifiedKeypoints(std::vector<int> key_points);
//    void InterpolateDerivs(std::vector<int> key_points);

    std::vector<int> ComputeKeypoints_SetInterval(derivative_interpolator interpolator, int horizon) const;

    std::vector<int> ComputeKeypoints_AdaptiveJerk(derivative_interpolator interpolator, int horizon) const;

    std::vector<int> ComputeKeypoints_MagVelChange(derivative_interpolator interpolator, int horizon) const;

    std::vector<int> ComputeKeypoints_IterativeError(derivative_interpolator interpolator, int horizon) const;

};

//template <>
//std::vector<int> DerivativeInterpolator<Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>>::ComputeKeypoints(int) const;


//} // namespace traj_opt
//} // namespace drake

