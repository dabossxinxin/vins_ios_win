#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "parameters.h"
#include "utility/tic_toc.h"
#include "utility/camera_factory.h"
#include "utility/pinhole_camera.h"

bool inBorder(const cv::Point2f &pt);
void reduceVector(std::vector<cv::Point2f> &v, std::vector<uchar> status);
void reduceVector(std::vector<int> &v, std::vector<uchar> status);

class FeatureTracker
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	FeatureTracker();

	void readImage(const cv::Mat &_img);

	void setMask();

	void addPoints();

	bool updateID(unsigned int i);

	void readIntrinsicParameter(const std::string &calib_file);

	void showUndistortion(const std::string &name);

	void rejectWithF();

	std::vector<cv::Point2f> undistortedPoints();

	cv::Mat		mask;			// 特征点数量不足时，用于再提取一些新特征所设置的mask
	cv::Mat		fisheye_mask;	// 当所用相机为鱼眼相机时，鱼眼相机有效区域的mask
	cv::Mat		prev_img;		// 上一需发布时刻图像
	cv::Mat		cur_img;		// 上一跟踪时刻的图像
	cv::Mat		forw_img;		// 当前帧时刻的图像
	std::vector<cv::Point2f> n_pts;		// 特征数量不足时，新提取出的特征坐标
	std::vector<cv::Point2f> prev_pts;	// 上一个发布帧中提取出来的图像特征
	std::vector<cv::Point2f> cur_pts;	// 上一帧图像中提取出来的图像特征
	std::vector<cv::Point2f> forw_pts;	// 当前帧光流跟踪上一帧得到的图像特征
	std::vector<int> ids;				// 当前帧所有特征的ID
	std::vector<int> track_cnt;			// 当前帧所有特征的被跟踪次数
	PinholeCameraPtr m_camera;			// 当前特征跟踪系统所使用的相机模型
	static int			n_id;			// 静态全局变量，用于生成特征点ID
};
