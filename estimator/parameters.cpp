#include "parameters.h"
#include <string>
#include <iostream>
#include "utility/print.h"

#ifdef WIN32
#include <direct.h>
#endif

#define FILENAMEPATH_MAX 80

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
double TD;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
int LOOP_CLOSURE = 0;
int MIN_LOOP_NUM;
std::string CAM_NAMES_ESTIMATOR;   //add
std::string PATTERN_FILE;
std::string VOC_FILE;
std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
int IMAGE_ROW, IMAGE_COL;
std::string VINS_FOLDER_PATH;
int MAX_KEYFRAME_NUM;

//feature tracker section
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE_FEATURE_TRACKER;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;
float VISUALLOOKATX;
float VISUALLOOKATY;
float VISUALLOOKATZ;

void readParameters(const std::string & config_file)
{
	cv::FileStorage fsSettings(config_file.c_str(), cv::FileStorage::READ);
	if (!fsSettings.isOpened()) {
		console::print_error("ERROR:Wrong path to settings: \n", config_file.c_str());
	}

#ifdef __WIN32__
    VINS_FOLDER_PATH = _getcwd(NULL, FILENAMEPATH_MAX);
	console::print_info("INFO: VINS_FOLDER_PATH: %s\n", VINS_FOLDER_PATH.c_str());
#elif __APPLE__
    VINS_FOLDER_PATH = "/Users/xiongxinxin/Desktop/Remove_ROS_VINS/vins_estimator/src";
#endif

	VINS_FOLDER_PATH = "../../";
	fsSettings["image_topic"] >> IMAGE_TOPIC;
	fsSettings["imu_topic"] >> IMU_TOPIC;
	fsSettings["visualLookAtX"] >> VISUALLOOKATX;
	fsSettings["visualLookAtY"] >> VISUALLOOKATY;
	fsSettings["visualLookAtZ"] >> VISUALLOOKATZ;

	IMAGE_COL = fsSettings["image_width"];
	IMAGE_ROW = fsSettings["image_height"];

	SOLVER_TIME = fsSettings["max_solver_time"];
	NUM_ITERATIONS = fsSettings["max_num_iterations"];
	MIN_PARALLAX = fsSettings["keyframe_parallax"];

	fsSettings["output_path"] >> VINS_RESULT_PATH;
	VINS_RESULT_PATH = VINS_FOLDER_PATH + VINS_RESULT_PATH;
	console::print_highlight("VINS_RESULT_PATH:%s\n", VINS_RESULT_PATH.c_str());
	std::ofstream foutC(VINS_RESULT_PATH, std::ios::out);
	foutC.close();

	ACC_N = fsSettings["acc_n"];
	ACC_W = fsSettings["acc_w"];
	GYR_N = fsSettings["gyr_n"];
	GYR_W = fsSettings["gyr_w"];
	G.z() = fsSettings["g_norm"];

	TD = fsSettings["td"];
	ESTIMATE_TD = fsSettings["estimate_td"];
	ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
	console::print_highlight("Estimate_Extrinsic: ");
	console::print_value("%d\n", ESTIMATE_EXTRINSIC);

	if (ESTIMATE_EXTRINSIC == 2) {
		console::print_highlight("INFO: have no prior about extrinsic param, calibrate extrinsic param\n");
		RIC.emplace_back(Eigen::Matrix3d::Identity());
		TIC.emplace_back(Eigen::Vector3d::Zero());
		fsSettings["ex_calib_result_path"] >> EX_CALIB_RESULT_PATH;
		EX_CALIB_RESULT_PATH = VINS_FOLDER_PATH + EX_CALIB_RESULT_PATH;
	}
	else {
		if (ESTIMATE_EXTRINSIC == 1) {
			console::print_highlight("INFO: Optimize extrinsic param around initial guess!\n");
			fsSettings["ex_calib_result_path"] >> EX_CALIB_RESULT_PATH;
			EX_CALIB_RESULT_PATH = VINS_FOLDER_PATH + EX_CALIB_RESULT_PATH;
		}
		if (ESTIMATE_EXTRINSIC == 0)
			console::print_highlight("INFO: fix extrinsic param\n");

		cv::Mat cv_R, cv_T;
		fsSettings["extrinsicRotation"] >> cv_R;
		fsSettings["extrinsicTranslation"] >> cv_T;
		Eigen::Matrix3d eigen_R;
		Eigen::Vector3d eigen_T;
		cv::cv2eigen(cv_R, eigen_R);
		cv::cv2eigen(cv_T, eigen_T);
		Eigen::Quaterniond Q(eigen_R);
		eigen_R = Q.normalized();
		RIC.emplace_back(eigen_R);
		TIC.emplace_back(eigen_T);

		console::print_highlight("Extrinsic_R:\n");
		console::print_matrix(RIC[0].data(), 3, 3);
		console::print_highlight("Extrinsic_T:\n");
		console::print_matrix(TIC[0].data(), 3, 1);
	}

	LOOP_CLOSURE = fsSettings["loop_closure"];
	console::print_highlight("Loop_Closure: ");
	console::print_value("%d\n", LOOP_CLOSURE);

	if (LOOP_CLOSURE == 1) {
		fsSettings["voc_file"] >> VOC_FILE;
		fsSettings["pattern_file"] >> PATTERN_FILE;
		// TODO 闭环检测加载词袋文件有问题，先给一个错误的路径
		VOC_FILE = "../" + VINS_FOLDER_PATH + VOC_FILE;
		PATTERN_FILE = VINS_FOLDER_PATH + PATTERN_FILE;
		MIN_LOOP_NUM = fsSettings["min_loop_num"];
		CAM_NAMES_ESTIMATOR = config_file;
	}

	INIT_DEPTH = 5.0;
	BIAS_ACC_THRESHOLD = 0.1;
	BIAS_GYR_THRESHOLD = 0.1;
	MAX_KEYFRAME_NUM = 1000;

	// feature tracker
	fsSettings["image_topic"] >> IMAGE_TOPIC;
	fsSettings["imu_topic"] >> IMU_TOPIC;
	MAX_CNT = fsSettings["max_cnt"];
	MIN_DIST = fsSettings["min_dist"];
	ROW = fsSettings["image_height"];
	COL = fsSettings["image_width"];
	FREQ = fsSettings["freq"];
	F_THRESHOLD = fsSettings["F_threshold"];
	SHOW_TRACK = fsSettings["show_track"];
	EQUALIZE = fsSettings["equalize"];
	FISHEYE = fsSettings["fisheye"];
	if (FISHEYE == 1)
		FISHEYE_MASK = VINS_FOLDER_PATH + "/config/fisheye_mask.jpg";
	CAM_NAMES.emplace_back(config_file);
	WINDOW_SIZE_FEATURE_TRACKER = 20;
	STEREO_TRACK = false;
	FOCAL_LENGTH = 460;
	PUB_THIS_FRAME = false;

	MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;
	if (FREQ == 0)
		FREQ = 100;

	fsSettings.release();
}