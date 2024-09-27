#include "derivative_interpolator.h"

//void DerivativeInterpolator::GetApproximateDerivsOverTrajectory(derivative_interpolator interpolator){
//
//    std::vector<int> keypoints = ComputeKeypoints(interpolator);
//
//
//}

//void DerivativeInterpolator::ComputeDerivsAtSpecifiedKeypoints(std::vector<int> key_points){
//
//}

//void DerivativeInterpolator::InterpolateDerivs(std::vector<int> keypoints){
//
//}

namespace drake {
    namespace traj_opt {

        std::vector<int> DerivativeInterpolator::ComputeKeypoints(
                derivative_interpolator interpolator,
                int horizon) const {
            std::vector<int> keypoints;
            if (interpolator.keyPointMethod == "set_interval") {
                keypoints = ComputeKeypoints_SetInterval(interpolator, horizon);
            } else if (interpolator.keyPointMethod == "adaptive_jerk") {
                keypoints = ComputeKeypoints_AdaptiveJerk(interpolator, horizon);
            } else if (interpolator.keyPointMethod == "mag_vel_change") {
                keypoints = ComputeKeypoints_MagVelChange(interpolator, horizon);
            } else if (interpolator.keyPointMethod == "iterative_error") {
                keypoints = ComputeKeypoints_IterativeError(interpolator, horizon);
            } else {
                std::cout << "Invalid keypoint method" << std::endl;
            }

            return keypoints;
        }

        std::vector<int> DerivativeInterpolator::ComputeKeypoints_SetInterval(
                derivative_interpolator interpolator,
                int horizon) const {
            std::vector<int> keypoints;
            int counter = 0;
            // Push index 0 and 1 by default
            keypoints.push_back(0);
            keypoints.push_back(1);

            for (int i = 1; i < horizon - 1; i++) {
                if (counter >= interpolator.minN) {
                    keypoints.push_back(i);
                    counter = 0;
                }
                counter++;
            }

            if (keypoints[keypoints.size() - 3] != horizon - 2) {
                keypoints.push_back(horizon - 2);
            }

            // If second to last index is not horizon - 1
            if (keypoints[keypoints.size() - 2] != horizon - 1) {
                keypoints.push_back(horizon - 1);
            }

            // If last index is not horizon
            if (keypoints.back() != horizon) {
                keypoints.push_back(horizon);
            }

            return keypoints;
        }

        std::vector<int> DerivativeInterpolator::ComputeKeypoints_AdaptiveJerk(
                derivative_interpolator interpolator,
                int horizon) const {
            std::vector<int> keypoints;
            std::cout << interpolator.keyPointMethod << horizon << std::endl;

            return keypoints;
        }

        std::vector<int> DerivativeInterpolator::ComputeKeypoints_MagVelChange(
                derivative_interpolator interpolator,
                int horizon) const {
            std::vector<int> keypoints;
            std::cout << interpolator.keyPointMethod << horizon << std::endl;

            return keypoints;
        }

        std::vector<int> DerivativeInterpolator::ComputeKeypoints_IterativeError(
                derivative_interpolator interpolator,
                int horizon) const {
            std::vector<int> keypoints;
            std::cout << interpolator.keyPointMethod << horizon << std::endl;

            return keypoints;
        }

        void DerivativeInterpolator::SavePartials(
                std::string file_prefix,
                InverseDynamicsPartials<double> *id_partials) const {
            std::ofstream fileOutput;

            std::vector <drake::MatrixX<double>> &dtau_dqm = id_partials->dtau_dqm;
            std::vector <drake::MatrixX<double>> &dtau_dqt = id_partials->dtau_dqt;
            std::vector <drake::MatrixX<double>> &dtau_dqp = id_partials->dtau_dqp;

            // Save dtau_dqm
//            std::cout << "root " << __FILE__ << std::endl;
            std::string root = "/home/davidrussell/catkin_ws/src/drake/traj_opt/data/";
            std::string file_name = root + file_prefix + "_dtau_dqm.csv";
//            std::cout << file_name << std::endl;
            fileOutput.open(file_name);

//            //print the fileout location
//            std::cout << "fileout location: " << file_name << std::endl;

            int size = dtau_dqm.size();

            for (int i = 0; i < size; i++) {
                // Row
                for (int j = 0; j < dtau_dqm[i].rows(); j++) {
                    // Column
                    for (int k = 0; k < dtau_dqm[i].cols(); k++) {
                        fileOutput << dtau_dqm[i](j, k) << ",";
                    }
                }
                fileOutput << std::endl;
            }

            fileOutput.close();

            // Save dtau_dqt
            file_name = root + file_prefix + "_dtau_dqt.csv";
            fileOutput.open(file_name);

            size = dtau_dqt.size();

            for (int i = 0; i < size; i++) {
                // Row
                for (int j = 0; j < dtau_dqt[i].rows(); j++) {
                    // Column
                    for (int k = 0; k < dtau_dqt[i].cols(); k++) {
                        fileOutput << dtau_dqt[i](j, k) << ",";
                    }
                }
                fileOutput << std::endl;
            }

            fileOutput.close();

            // Save dtau_dqp
            file_name = root + file_prefix + "_dtau_dqp.csv";
            fileOutput.open(file_name);

            size = dtau_dqp.size();

            for (int i = 0; i < size; i++) {
                // Row
                for (int j = 0; j < dtau_dqp[i].rows(); j++) {
                    // Column
                    for (int k = 0; k < dtau_dqp[i].cols(); k++) {
                        fileOutput << dtau_dqp[i](j, k) << ",";
                    }
                }
                fileOutput << std::endl;
            }

            fileOutput.close();
        }

        double DerivativeInterpolator::ComputeError(
                const InverseDynamicsPartials<double> *id_partials1,
                const InverseDynamicsPartials<double> *id_partials2) const{

            double error = 0.0f;
            const std::vector <drake::MatrixX<double>> &dtau_dqm1 = id_partials1->dtau_dqm;
            const std::vector <drake::MatrixX<double>> &dtau_dqt1 = id_partials1->dtau_dqt;
            const std::vector <drake::MatrixX<double>> &dtau_dqp1 = id_partials1->dtau_dqp;

            const std::vector <drake::MatrixX<double>> &dtau_dqm2 = id_partials2->dtau_dqm;
            const std::vector <drake::MatrixX<double>> &dtau_dqt2 = id_partials2->dtau_dqt;
            const std::vector <drake::MatrixX<double>> &dtau_dqp2 = id_partials2->dtau_dqp;

            // -------------------------- dtau_dqm -------------------------------------
            int size = dtau_dqm1.size();
            double error_dqm = 0.0f;
            for (int i = 1; i < size; i++) {
                // Row
                for (int j = 0; j < dtau_dqm1[i].rows(); j++) {
                    // Column
                    for (int k = 0; k < dtau_dqm1[i].cols(); k++) {
                        error_dqm += abs(dtau_dqm1[i](j, k) - dtau_dqm2[i](j, k));
                    }
                }
            }

            error_dqm = error_dqm / size;

            // -------------------------- dtau_dqt -------------------------------------
            size = dtau_dqt1.size();
            double error_dqt = 0.0f;
            for (int i = 0; i < size; i++) {
                // Row
                for (int j = 0; j < dtau_dqt1[i].rows(); j++) {
                    // Column
                    for (int k = 0; k < dtau_dqt1[i].cols(); k++) {
                        error_dqt += abs(dtau_dqt1[i](j, k) - dtau_dqt2[i](j, k));
                    }
                }
            }

            error_dqt = error_dqt / size;

            // -------------------------- dtau_dqp -------------------------------------
            size = dtau_dqp1.size();
            double error_dqp = 0.0f;
            for (int i = 0; i < size; i++) {
                // Row
                for (int j = 0; j < dtau_dqp1[i].rows(); j++) {
                    // Column
                    for (int k = 0; k < dtau_dqp1[i].cols(); k++) {
                        error_dqp += abs(dtau_dqp1[i](j, k) - dtau_dqp2[i](j, k));
                    }
                }
            }

            error_dqp = error_dqp / size;

            std::cout << "error_dqm: " << error_dqm << std::endl;
            std::cout << "error_dqt: " << error_dqt << std::endl;
            std::cout << "error_dqp: " << error_dqp << std::endl;

            error = error_dqm + error_dqt + error_dqp;

            return error;
        }
    }
}


