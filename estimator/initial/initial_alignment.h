#pragma once

#include <map>
#include <iostream>
#include <Eigen/Dense>

#include "../factor/imu_factor.h"
#include "../utility/utility.h"
#include "../feature_manager.h"

class ImageFrame
{
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ImageFrame(){};
        ImageFrame(const std::map<int, std::vector<std::pair<int, Eigen::Vector3d>>>& _points, double _t):points{_points},t{_t},is_key_frame{false}
        {
        };
		std::map<int, std::vector<std::pair<int, Eigen::Vector3d>>> points;
        double t;
        Eigen::Matrix3d R;
		Eigen::Vector3d T;
        IntegrationBase *pre_integration;
        bool is_key_frame;
};

bool VisualIMUAlignment(std::map<double, ImageFrame> &all_image_frame, Eigen::Vector3d* Bgs, Eigen::Vector3d &g, Eigen::VectorXd &x);
