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

	cv::Mat		mask;			// ��������������ʱ����������ȡһЩ�����������õ�mask
	cv::Mat		fisheye_mask;	// ���������Ϊ�������ʱ�����������Ч�����mask
	cv::Mat		prev_img;		// ��һ�跢��ʱ��ͼ��
	cv::Mat		cur_img;		// ��һ����ʱ�̵�ͼ��
	cv::Mat		forw_img;		// ��ǰ֡ʱ�̵�ͼ��
	std::vector<cv::Point2f> n_pts;		// ������������ʱ������ȡ������������
	std::vector<cv::Point2f> prev_pts;	// ��һ������֡����ȡ������ͼ������
	std::vector<cv::Point2f> cur_pts;	// ��һ֡ͼ������ȡ������ͼ������
	std::vector<cv::Point2f> forw_pts;	// ��ǰ֡����������һ֡�õ���ͼ������
	std::vector<int> ids;				// ��ǰ֡����������ID
	std::vector<int> track_cnt;			// ��ǰ֡���������ı����ٴ���
	PinholeCameraPtr m_camera;			// ��ǰ��������ϵͳ��ʹ�õ����ģ��
	static int			n_id;			// ��̬ȫ�ֱ�������������������ID
};
