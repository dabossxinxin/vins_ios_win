#ifndef __KEY_FRAME_
#define __KEY_FRAME_

#include <algorithm>
#include <math.h>
#include <mutex>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "loop_closure.h"
#include "utility/utility.h"
#include "utility/camera_factory.h"
#include "utility/pinhole_camera.h"

class BriefExtractor : public FeatureExtractor<FBrief::TDescriptor>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		virtual void operator()(const cv::Mat &im, const std::vector<cv::Point2f> window_pts,
			std::vector<cv::KeyPoint> &keys, std::vector<BRIEF::bitset> &descriptors) const;
	BriefExtractor(const std::string &pattern_file);

private:
	DVision::BRIEF m_brief;
};

class KeyFrame
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		KeyFrame(double _header, Eigen::Vector3d _vio_T_w_c, Eigen::Matrix3d _vio_R_w_c,
			Eigen::Vector3d _cur_T_w_c, Eigen::Matrix3d _cur_R_w_c, cv::Mat &_image, const char *_brief_pattern_file,
			Eigen::Vector3d _relocalize_t, Eigen::Matrix3d _relocalize_r);
	void setExtrinsic(Eigen::Vector3d T, Eigen::Matrix3d R);
	void FundmantalMatrixRANSAC(std::vector<cv::Point2f> &measurements_old, std::vector<cv::Point2f> &measurements_old_norm,
		const PinholeCameraPtr &m_camera);

	// 提取关键帧中的特征点以及对应描述子
	void extractBrief(cv::Mat &image);

	// 将前端estimator中管理的特征点加入到关键帧中
	void buildKeyFrameFeatures(Estimator &estimator, const PinholeCameraPtr &m_camera);

	bool inAera(cv::Point2f pt, cv::Point2f center, float area_size);

	bool searchInAera(cv::Point2f center_cur, float area_size,
		const BRIEF::bitset window_descriptor,
		const std::vector<BRIEF::bitset> &descriptors_old,
		const std::vector<cv::KeyPoint> &keypoints_old,
		cv::Point2f &best_match);

	void searchByDes(std::vector<cv::Point2f> &measurements_old,
		std::vector<cv::Point2f> &measurements_old_norm,
		const std::vector<BRIEF::bitset> &descriptors_old,
		const std::vector<cv::KeyPoint> &keypoints_old,
		const PinholeCameraPtr &m_camera);

	bool findConnectionWithOldFrame(const KeyFrame* old_kf,
		std::vector<cv::Point2f> &measurements_old, std::vector<cv::Point2f> &measurements_old_norm,
		Eigen::Vector3d &PnP_T_old, Eigen::Matrix3d &PnP_R_old,
		const PinholeCameraPtr &m_camera);

	void PnPRANSAC(std::vector<cv::Point2f> &measurements_old,
		std::vector<cv::Point2f> &measurements_old_norm,
		Eigen::Vector3d &PnP_T_old, Eigen::Matrix3d &PnP_R_old);

	// 更新闭环校正后的pose
	void updatePose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);

	// 更新前端里程计pose
	void updateOriginPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);

	// 获取闭环矫正后的pose
	void getPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);

	// 获取前端里程计pose
	void getOriginPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);

	void addConnection(int index, KeyFrame* connected_kf);

	void addConnection(int index, KeyFrame* connected_kf, Eigen::Vector3d relative_t, Eigen::Quaterniond relative_q, double relative_yaw);

	void updateLoopConnection(Eigen::Vector3d relative_t, Eigen::Quaterniond relative_q, double relative_yaw);

	void detectLoop(int index);

	void removeLoop();
	void getPath(Eigen::Vector3d& path);

	int HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b);

	Eigen::Vector3d getLoopRelativeT();

	double getLoopRelativeYaw();

	double			header;					// 当前关键帧时间戳
	int				global_index;			// 当前帧全局索引
	cv::Mat			image;					// 当前帧对应图像
	Eigen::Matrix3d qic;					// 当前帧旋转外参
	Eigen::Vector3d tic;					// 当前帧平移外参
	int				COL;					// 当前帧图像列数
	int				ROW;					// 当前帧图像行数
	bool			retrive;

	bool			has_loop;				// 当前关键帧是否有闭环关键帧
	int				loop_index;				// 当前关键帧闭环帧的索引
	bool			update_loop_info;

	bool			is_looped;				// 当前关键帧是否是其他关键帧的闭环
	int				resample_index;			// 闭环位姿图优化时所用索引
	const char		*BRIEF_PATTERN_FILE;	// 描述子模板文件路径

	std::vector<Eigen::Vector3d>	point_clouds;			// 特征点对应的世界系坐标
	std::vector<Eigen::Vector3d>	point_clouds_matched;

	std::vector<cv::Point2f>		measurements;			// 特征点相机坐标系坐标
	std::vector<cv::Point2f>		measurements_matched;
	std::vector<cv::Point2f>		pts_normalize;			// 归一化相机平面坐标

	std::vector<int>				features_id;			// 特征点的ID
	std::vector<int>				features_id_matched;

	std::vector<BRIEF::bitset>		descriptors;			// 当前帧所有特征点的描述子
	std::vector<cv::KeyPoint>		keypoints;				// 当前帧中提取出的特征点

	Eigen::Vector3d					relocalize_t;			// 当前帧回环矫正位置
	Eigen::Matrix3d					relocalize_r;			// 当前帧回环矫正姿态

private:
	Eigen::Vector3d					T_w_i;				// 当前关键帧闭环矫正后的位置
	Eigen::Matrix3d					R_w_i;				// 当前关键帧闭环矫正后的姿态
	Eigen::Vector3d					vio_T_w_i;			// 当前关键帧前端计算到的位置
	Eigen::Matrix3d					vio_R_w_i;			// 当前关键帧前端计算到的姿态
	std::mutex						mMutexPose;			// 互斥锁，位姿量
	std::mutex						mLoopInfo;			// 互斥锁，闭环信息
	std::vector<cv::KeyPoint>		window_keypoints;	// 前端滑窗中提取到的特征
	std::vector<BRIEF::bitset>		window_descriptors;	// 滑窗中特征对应的描述子
	Eigen::Matrix<double, 8, 1>		loop_info;			// tx,ty,tz,qw,qx,qy,qz,yaw
};
#endif