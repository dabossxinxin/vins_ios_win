#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

class MotionEstimator
{
  public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool solveRelativeRT(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &corres, Eigen::Matrix3d &R, Eigen::Vector3d &T);

  private:
    double testTriangulation(const std::vector<cv::Point2f> &l,
                             const std::vector<cv::Point2f> &r,
                             cv::Mat_<double> R, cv::Mat_<double> t);
    void decomposeE(cv::Mat E,
                    cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                    cv::Mat_<double> &t1, cv::Mat_<double> &t2);
};