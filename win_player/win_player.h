#pragma once

//#include <Eigen/Core>
//#include <Eigen/Dense>
//#include "estimator.h"
//#include <pangolin/pangolin.h>
//
//class PlayerWindows
//{
//public:
//	PlayerWindows(std::shared_ptr<Estimator>& estimator);
//
//	~PlayerWindows();
//
//	void play();
//private:
//
//	void ViewCameraPath(const std::vector<Eigen::Vector3d>& keypose);
//
//	void ViewCameraGlobalLandmark(const std::unordered_map<int, Eigen::Vector3d>& landmarks);
//
//	void ViewCameraLocalLandmarks(const std::unordered_map<int, Eigen::Vector3d>& landmarks);
//
//	void ViewCameraPose(Eigen::Vector3d& loop_correct_t,
//		Eigen::Matrix3d& loop_correct_r, pangolin::OpenGlMatrix& M);
//
//	void DrawCurrentCamera(pangolin::OpenGlMatrix& Twc);
//
//private:
//
//	std::shared_ptr<Estimator> m_estimator;
//};