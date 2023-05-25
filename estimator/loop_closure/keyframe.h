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

	// ��ȡ�ؼ�֡�е��������Լ���Ӧ������
	void extractBrief(cv::Mat &image);

	// ��ǰ��estimator�й������������뵽�ؼ�֡��
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

	// ���±ջ�У�����pose
	void updatePose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);

	// ����ǰ����̼�pose
	void updateOriginPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);

	// ��ȡ�ջ��������pose
	void getPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);

	// ��ȡǰ����̼�pose
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

	double			header;					// ��ǰ�ؼ�֡ʱ���
	int				global_index;			// ��ǰ֡ȫ������
	cv::Mat			image;					// ��ǰ֡��Ӧͼ��
	Eigen::Matrix3d qic;					// ��ǰ֡��ת���
	Eigen::Vector3d tic;					// ��ǰ֡ƽ�����
	int				COL;					// ��ǰ֡ͼ������
	int				ROW;					// ��ǰ֡ͼ������
	bool			retrive;

	bool			has_loop;				// ��ǰ�ؼ�֡�Ƿ��бջ��ؼ�֡
	int				loop_index;				// ��ǰ�ؼ�֡�ջ�֡������
	bool			update_loop_info;

	bool			is_looped;				// ��ǰ�ؼ�֡�Ƿ��������ؼ�֡�ıջ�
	int				resample_index;			// �ջ�λ��ͼ�Ż�ʱ��������
	const char		*BRIEF_PATTERN_FILE;	// ������ģ���ļ�·��

	std::vector<Eigen::Vector3d>	point_clouds;			// �������Ӧ������ϵ����
	std::vector<Eigen::Vector3d>	point_clouds_matched;

	std::vector<cv::Point2f>		measurements;			// �������������ϵ����
	std::vector<cv::Point2f>		measurements_matched;
	std::vector<cv::Point2f>		pts_normalize;			// ��һ�����ƽ������

	std::vector<int>				features_id;			// �������ID
	std::vector<int>				features_id_matched;

	std::vector<BRIEF::bitset>		descriptors;			// ��ǰ֡�����������������
	std::vector<cv::KeyPoint>		keypoints;				// ��ǰ֡����ȡ����������

	Eigen::Vector3d					relocalize_t;			// ��ǰ֡�ػ�����λ��
	Eigen::Matrix3d					relocalize_r;			// ��ǰ֡�ػ�������̬

private:
	Eigen::Vector3d					T_w_i;				// ��ǰ�ؼ�֡�ջ��������λ��
	Eigen::Matrix3d					R_w_i;				// ��ǰ�ؼ�֡�ջ����������̬
	Eigen::Vector3d					vio_T_w_i;			// ��ǰ�ؼ�֡ǰ�˼��㵽��λ��
	Eigen::Matrix3d					vio_R_w_i;			// ��ǰ�ؼ�֡ǰ�˼��㵽����̬
	std::mutex						mMutexPose;			// ��������λ����
	std::mutex						mLoopInfo;			// ���������ջ���Ϣ
	std::vector<cv::KeyPoint>		window_keypoints;	// ǰ�˻�������ȡ��������
	std::vector<BRIEF::bitset>		window_descriptors;	// ������������Ӧ��������
	Eigen::Matrix<double, 8, 1>		loop_info;			// tx,ty,tz,qw,qx,qy,qz,yaw
};
#endif