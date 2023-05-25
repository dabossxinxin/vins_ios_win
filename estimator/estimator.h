#pragma once

#include "parameters.h"
#include "feature_manager.h"
#include "utility/utility.h"
#include "utility/tic_toc.h"
#include "utility/print.h"
#include "initial/solve_5pts.h"
#include "initial/initial_sfm.h"
#include "initial/initial_alignment.h"
#include "initial/initial_ex_rotation.h"
#include "include/Header.h"
#include "include/Float32.h"
#include <assert.h>
#include <ceres/ceres.h>
#include "factor/imu_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/projection_factor.h"
#include "factor/marginalization_factor.h"

#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>

struct RetriveData
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	int		old_index;		// �ػ�֡ID
	int		cur_index;		// ��ǰ֡ID
    double	header;			// ��ǰ֡ʱ���
	bool	relocalized;	// ��ǰ��Ϣ�Ƿ��Ѿ������ض�λ
	bool	relative_pose;	// ��ǰ��Ϣ���Ƿ��лػ�ǰ����ͬ֡��λ�˱任
	double	relative_yaw;	// �ػ�ǰ��ػ�֡��YAW�任��
	double	loop_pose[7];	// �ػ�֡ͨ��PnP����õ���λ��
	Eigen::Vector3d				P_old;			// �ػ�֡λ��
	Eigen::Matrix3d				R_old;			// �ػ�֡��̬
	Eigen::Vector3d				relative_t;		// �ػ�ǰ��ػ�֡��λ�ñ仯��
	Eigen::Quaterniond			relative_q;		// �ػ�ǰ��ػ�֡����̬�仯��
	std::vector<cv::Point2f>	measurements;	// ��ǰ֡�ڻػ���ƥ�䵽��������
	std::vector<int>			features_ids;	// ��ǰ֡��ƥ�䵽��������ID
};

class Estimator
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Estimator();

	void setParameter();

	// interface
	void processIMU(double t, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity);
	void processImage(const std::map<int, std::vector<std::pair<int, Eigen::Vector3d>>> &image, const std_msgs::Header &header);

	// internal
	void clearState();
	bool initialStructure();
	bool visualInitialAlign();
	bool relativePose(Eigen::Matrix3d &relative_R, Eigen::Vector3d &relative_T, int &l);
	void slideWindow();
	void solveOdometry();
	void slideWindowNew();
	void slideWindowOld();
	void optimization();
	void vector2double();
	void double2vector();
	bool failureDetection();
	void updatePointCloud();		// 新增项

	enum SolverFlag {
		INITIAL,
		NON_LINEAR
	};

	enum MarginalizationFlag {
		MARGIN_OLD = 0,
		MARGIN_SECOND_NEW = 1
	};

	SolverFlag solver_flag;
	MarginalizationFlag  marginalization_flag;
	Eigen::Vector3d g;
	Eigen::MatrixXd Ap[2], backup_A;
	Eigen::VectorXd bp[2], backup_b;

	Eigen::Matrix3d ric[NUM_OF_CAM];
	Eigen::Vector3d tic[NUM_OF_CAM];

	Eigen::Vector3d Ps[(WINDOW_SIZE + 1)];
	Eigen::Vector3d Vs[(WINDOW_SIZE + 1)];
	Eigen::Matrix3d Rs[(WINDOW_SIZE + 1)];
	Eigen::Vector3d Bas[(WINDOW_SIZE + 1)];
	Eigen::Vector3d Bgs[(WINDOW_SIZE + 1)];

	Eigen::Matrix3d back_R0, last_R, last_R0;
	Eigen::Vector3d back_P0, last_P, last_P0;
	std_msgs::Header Headers[(WINDOW_SIZE + 1)];

	IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
	Eigen::Vector3d acc_0, gyr_0;

	std::vector<double> dt_buf[(WINDOW_SIZE + 1)];
	std::vector<Eigen::Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
	std::vector<Eigen::Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

	int frame_count;
	int sum_of_outlier;
	int sum_of_back;
	int sum_of_front;
	int sum_of_invalid;

	FeatureManager f_manager;
	MotionEstimator m_estimator;
	InitialEXRotation initial_ex_rotation;

	bool first_imu;
	bool is_valid;
	bool is_key;
	bool failure_occur;

	std::unordered_map<int, Eigen::Vector3d> global_cloud;	// 新增项
	std::unordered_map<int, Eigen::Vector3d> local_cloud;	// 新增项
	//std::vector<Eigen::Vector3d> point_cloud;
	std::vector<Eigen::Vector3d> margin_cloud;
	std::vector<Eigen::Vector3d> key_poses;
	double initial_timestamp;

	double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
	double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
	double para_Feature[NUM_OF_F][SIZE_FEATURE];
	double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];
	double para_Retrive_Pose[SIZE_POSE];

	RetriveData retrive_pose_data, front_pose;
	std::vector<RetriveData> retrive_data_vector;
	int loop_window_index;
	bool relocalize;
	Eigen::Vector3d relocalize_t;
	Eigen::Matrix3d relocalize_r;

	MarginalizationInfo *last_marginalization_info;
	std::vector<double *> last_marginalization_parameter_blocks;

	std::map<double, ImageFrame> all_image_frame;
	IntegrationBase *tmp_pre_integration;
};