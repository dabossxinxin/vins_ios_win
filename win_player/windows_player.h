#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include <unordered_map>
#include <pangolin/pangolin.h>

// 使显示器尽量简单：显示的数据从类外以值拷贝的形式传递
class PlayerWindows
{
public:
	PlayerWindows();

	~PlayerWindows();

	void close();

	void play();

	cv::Mat m_featureImg;

	Eigen::Vector3d m_tic;
	Eigen::Matrix3d m_ric;
	Eigen::Vector3d m_fre;

	Eigen::Vector3d m_loopCorrectT;
	Eigen::Matrix3d m_loopCorrectR;

	Eigen::Vector3d m_curPosT;
	Eigen::Matrix3d m_curPosR;

	std::vector<Eigen::Vector3d> m_keypose;
	
	std::unordered_map<int, Eigen::Vector3d> m_localLandmarks;
	std::unordered_map<int, Eigen::Vector3d> m_globalLandmarks;

private:
	void ViewCameraPath();

	void ViewCameraGlobalLandmark();

	void ViewCameraLocalLandmarks();

	void ViewCameraPose(pangolin::OpenGlMatrix& M);

	void DrawCurrentCamera(pangolin::OpenGlMatrix& Twc);

	void R2ypr(const Eigen::Matrix3d &R, Eigen::Vector3d& ypr)
	{
		Eigen::Vector3d n = R.col(0);
		Eigen::Vector3d o = R.col(1);
		Eigen::Vector3d a = R.col(2);

		double y = atan2(n(1), n(0));
		double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
		double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
		ypr(0) = y;
		ypr(1) = p;
		ypr(2) = r;

		ypr / CV_PI * 180.0;
	}

private:
	bool m_status;

	int m_imgWidth;
	int m_imgHeight;

	pangolin::DataLog m_logTic;
	pangolin::DataLog m_logRic;
	pangolin::DataLog m_logFre;

	//pangolin::Var<bool> m_menuFollowCamera;
	//pangolin::Var<bool> m_menuShowPoints;
	//pangolin::Var<bool> m_menuShowPath;

	pangolin::View m_d_image;
	pangolin::View m_d_camera;
	unsigned char *m_imgArray;
	pangolin::GlTexture m_imgTexture;
	pangolin::OpenGlRenderState m_s_camera;
};