#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <iostream>
#include <fstream>
#include <chrono>

#include <json/json.h>
#include <condition_variable>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "include/PointCloud.h"
#include "include/Imu.h"
#include "utility/print.h"
#include "utility/tic_toc.h"
#include "utility/producer.h"
#include "utility/visualization.h"
#include "utility/camera_factory.h"
#include "utility/pinhole_camera.h"
#include "loop_closure/loop_closure.h"
#include "loop_closure/keyframe.h"
#include "loop_closure/keyframe_database.h"
#include "feature_tracker/feature_tracker.h"

#include <pangolin/pangolin.h>

#define SHOW_UNDISTORTION 0

std::vector<uchar> r_status;
std::vector<float> r_err;
FeatureTracker trackerData[NUM_OF_CAM];
double first_image_time;
double last_image_time = 0;
int pub_count = 1;
bool init_pub = false;
bool first_image_flag = true;
bool running_flag = true;
bool view_done = false;

Estimator estimator;
LoopClosure *loop_closure;

RabbitMQProducer imuProducer;
RabbitMQProducer imageProducer;

std::condition_variable con;
double current_time = -1;
std::queue<sensor_msgs::ImuConstPtr> imu_buf;
std::queue<sensor_msgs::PointCloudConstPtr> feature_buf;
std::mutex m_posegraph_buf;
std::queue<int> optimize_posegraph_buf;
std::queue<KeyFrame*> keyframe_buf;
std::queue<RetriveData> retrive_data_buf;

// 闭环检测相关成员变量
int sum_of_wait = 0;
std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_loop_drift;
std::mutex m_keyframedatabase_resample;
std::mutex m_update_visualization;
std::mutex m_keyframe_buf;
std::mutex m_retrive_data_buf;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;

std::queue<std::pair<cv::Mat, double>> image_buf;
KeyFrameDatabase keyframe_database;

int global_frame_cnt = 0;
PinholeCameraPtr m_camera;
std::vector<int> erase_index;
std_msgs::Header cur_header;
Eigen::Vector3d relocalize_t{Eigen::Vector3d(0, 0, 0)};
Eigen::Matrix3d relocalize_r{Eigen::Matrix3d::Identity()};
nav_msgs::Path  loop_path;

std::mutex m_img_vis;
cv::Mat img_visualization;
std::mutex m_real_freq;
float process_freq = -1;
float show_freq = -1;
float feature_freq = -1;

std::mutex mutex_key_pose;
double td = 0;
std::vector<Eigen::Vector3d> global_keypose;
std::unordered_map<int, Eigen::Vector3d> local_landmarks;
std::unordered_map<int, Eigen::Vector3d> global_landmarks;

void saveGlobalLandmarks()
{
	std::string filename = "C:\\Users\\jackchen\\Desktop\\debug\\landmark.txt";
	std::ofstream foutC(filename.c_str(), std::ios::app);
	foutC.setf(std::ios::fixed, std::ios::floatfield);
	foutC.precision(9);
	for (const auto& pairs : global_landmarks) {
		const auto& landmark = pairs.second;
		foutC << landmark.x() << " "
			<< landmark.y() << " "
			<< landmark.z() << std::endl;
	}
	foutC.close();
}

void updateLoopPath(nav_msgs::Path _loop_path)
{
    loop_path = _loop_path;
}

void ViewCameraLandmark()
{
	std::unique_lock<std::mutex> lock_key_pose(mutex_key_pose);
    glPointSize(1.5f);
	glBegin(GL_POINTS);
	glColor3f(0.0, 0.0, 1.0);
	for (const auto& pairs : local_landmarks) {
		const auto& landmark = pairs.second;
		glVertex3f(landmark.x(), landmark.y(), landmark.z());
    }
	glEnd();

    glPointSize(1.5f);
    glBegin(GL_POINTS);
	glColor3f(0.0, 0.0, 0.0);
    for (const auto& pairs: global_landmarks) {
		const auto& landmark = pairs.second;
		glVertex3f(landmark.x(), landmark.y(), landmark.z());
    }
    glEnd();

	lock_key_pose.unlock();
}

void ViewCameraPath()
{
	std::unique_lock<std::mutex> lock_key_pose(mutex_key_pose);
	GLfloat line_width = 1.2;
	Eigen::Vector3d tmp_path;
	glColor3f(1.0f, 0.0f, 0.0f);
	glLineWidth(line_width);
	glBegin(GL_LINE_STRIP);

	for (const auto& it : global_keypose) {
		glVertex3f(it.x(), it.y(), it.z());
	}
	glEnd();
}

void ViewCameraPose(Eigen::Vector3d loop_correct_t, 
	Eigen::Matrix3d loop_correct_r, pangolin::OpenGlMatrix &M)
{
	int i = WINDOW_SIZE;
	if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR) {
		Eigen::Vector3d P = (loop_correct_r * estimator.Ps[i] + loop_correct_t) +
			(loop_correct_r * estimator.Rs[i]) * estimator.tic[0];
		Eigen::Matrix3d R = (loop_correct_r * estimator.Rs[i]) * estimator.ric[0];

		M.m[0] = R(0, 0);
		M.m[1] = R(1, 0);
		M.m[2] = R(2, 0);
		M.m[3] = 0.0;

		M.m[4] = R(0, 1);
		M.m[5] = R(1, 1);
		M.m[6] = R(2, 1);
		M.m[7] = 0.0;

		M.m[8] = R(0, 2);
		M.m[9] = R(1, 2);
		M.m[10] = R(2, 2);
		M.m[11] = 0.0;


		M.m[12] = P.x();
		M.m[13] = P.y();
		M.m[14] = P.z();
		M.m[15] = 1.0;
	}
	else {
		M.SetIdentity();
	}
}

void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
	const float &w = 0.08f;
	const float h = w * 0.75;
	const float z = w * 0.6;

	glPushMatrix();

#ifdef HAVE_GLES
	glMultMatrixf(Twc.m);
#else
	glMultMatrixd(Twc.m);
#endif

	glLineWidth(2);				//set line width
	glColor3f(0.0f,0.0f,1.0f);	//blue
	glBegin(GL_LINES);			//draw camera 
	glVertex3f(0,0,0);
	glVertex3f(w,h,z);
	glVertex3f(0,0,0);
	glVertex3f(w,-h,z);
	glVertex3f(0,0,0);
	glVertex3f(-w,-h,z);
	glVertex3f(0,0,0);
	glVertex3f(-w,h,z);

	glVertex3f(w,h,z);
	glVertex3f(w,-h,z);
	glVertex3f(-w,h,z);
	glVertex3f(-w,-h,z);
	glVertex3f(-w,h,z);
	glVertex3f(w,h,z);
	glVertex3f(-w,-h,z);
	glVertex3f(w,-h,z);
	glEnd();
	glPopMatrix();
}

void visualization()
{
	int img_rows = 480;
	int img_cols = 752;
	float mViewpointX = -0;
	float mViewpointY = -5;
	float mViewpointZ = -10;
	float mViewpointF = 500;
	pangolin::CreateWindowAndBind("VINS: Map Visualization",1024,768);
	glEnable(GL_DEPTH_TEST);
	pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, float(300.0 / 1024.0));
 	pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
 	pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
	pangolin::Var<bool> menuShowPath("menu.Show Path", true, true);

	// 添加外参曲线图
	pangolin::DataLog logTic;
	std::vector<std::string> labelTic;
	labelTic.emplace_back(std::string("Tic.x"));
	labelTic.emplace_back(std::string("Tic.y"));
	labelTic.emplace_back(std::string("Tic.z"));
	logTic.SetLabels(labelTic);

	pangolin::DataLog logRic;
	std::vector<std::string> labelRic;
	labelRic.emplace_back(std::string("yaw"));
	labelRic.emplace_back(std::string("pitch"));
	labelRic.emplace_back(std::string("roll"));
	logRic.SetLabels(labelRic);

	// 添加系统帧率统计曲线图
	pangolin::DataLog logFREQ;
	std::vector<std::string> labelFREQ;
	labelFREQ.emplace_back(std::string("feature_freq"));
	labelFREQ.emplace_back(std::string("process_freq"));
	labelFREQ.emplace_back(std::string("show_freq"));
	logFREQ.SetLabels(labelFREQ);

	pangolin::Plotter plotterRic(&logRic, 0.0f, 100.0f, -0.02f, 0.02f, 10.0f, 0.001f);
	plotterRic.SetBounds(float(240.0 / 768.0), float(440.0 / 768.0), float(10.0 / 1024.0), float(290.0 / 1024.0));                                                                             
	plotterRic.Track("$i");

	pangolin::Plotter plotterTic(&logTic, 0.0f, 100.0f, -0.02f, 0.02f, 10.0f, 0.001f);
	plotterTic.SetBounds(float(20.0 / 768.0), float(220.0 / 768.0), float(10.0 / 1024.0), float(290.0 / 1024.0));
	plotterTic.Track("$i");

	pangolin::Plotter plotterFREQ(&logFREQ, 0.0f, 100.0f, 0.0f, 0.5f, 10.0f, 0.001f);
	plotterFREQ.SetBounds(float(460.0 / 768.0), float(660.0 / 768.0), float(10.0 / 1024.0), float(290.0 / 1024.0));
	plotterFREQ.Track("$i");
	
	pangolin::DisplayBase().AddDisplay(plotterRic);
	pangolin::DisplayBase().AddDisplay(plotterTic);
	pangolin::DisplayBase().AddDisplay(plotterFREQ);

   	// Define Camera Render Object (for view / scene browsing)
	pangolin::OpenGlRenderState s_cam(
		pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000),
		pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, VISUALLOOKATX, VISUALLOOKATY, VISUALLOOKATZ)
	);

	// Add named OpenGL viewport to window and provide 3D Handler
	pangolin::View& d_cam = pangolin::CreateDisplay()
		.SetBounds(0.0, 1.0, float(300.0 / 1024.0), 1.0, -1024.0f / 768.0f)
		.SetHandler(new pangolin::Handler3D(s_cam));

	float ratio = float(img_rows) / float(img_cols);
	int img_width = 280;
	int img_height = 280.0*ratio;
	pangolin::View& d_image = pangolin::CreateDisplay()
		.SetBounds(float(768 - img_height) / 768.0, 1.0f, float(300.0 / 1024.0), float(300 + img_width) / 1024.0, float(img_width) / img_height)
		.SetLock(pangolin::LockLeft, pangolin::LockTop);

	unsigned char* imageArray = new unsigned char[3 * img_rows*img_cols];
	pangolin::GlTexture imageTexture(img_cols, img_rows, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
	
	pangolin::OpenGlMatrix Twc;
	Eigen::Vector3d tic;
	Eigen::Vector3d ypr;
	Twc.SetIdentity();

	while (!pangolin::ShouldQuit() & running_flag) {
		TicToc t_show_static;

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		ViewCameraPose(relocalize_t, relocalize_r, Twc);
		glClearColor(1.0f, 1.0f, 1.0f, 0.5f);
		
		if (keyframe_database.viewNewestKeyFrameEx(ypr, tic)) {
			logRic.Log(ypr(0), ypr(1), ypr(2));
			logTic.Log(tic(0), tic(1), tic(2));
		}

		std::unique_lock<std::mutex> lock_freq(m_real_freq);
		//logFREQ.Log(feature_freq, process_freq, show_freq);
		logFREQ.Log(td, td, td);
		/*if (process_freq != -1 && feature_freq != -1 && show_freq != -1) {
			feature_freq = -1;
			process_freq = -1;
			show_freq = -1;
		}*/
		lock_freq.unlock();

		if (menuFollowCamera) {
			s_cam.Follow(Twc);
		}
			
		d_cam.Activate(s_cam);
		DrawCurrentCamera(Twc);
		if (menuShowPoints) {
			ViewCameraLandmark();
		}
		
		if (menuShowPath) {
			ViewCameraPath();
		}

		d_image.Activate();
		glColor3f(1.0f, 1.0f, 1.0f);
		std::unique_lock<std::mutex> lock_img(m_img_vis);
		if (!img_visualization.empty()) {
			memcpy(imageArray, img_visualization.data, sizeof(uchar) * 3 * img_rows*img_cols);
			imageTexture.Upload(imageArray, GL_RGB, GL_UNSIGNED_BYTE);
			imageTexture.RenderToViewport();
		}
		lock_img.unlock();

		pangolin::FinishFrame();

		float t_show = t_show_static.toc();
		if (lock_freq.try_lock()) {
			show_freq = 1000.0 / t_show;
			lock_freq.unlock();
		}

		console::print_highlight("INFO: visualization thread time: %.1f ms\n", t_show);
	}
	saveGlobalLandmarks();
	console::print_highlight("Visualization thread end.\n");
	view_done = true;
}

void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
	Eigen::Vector3d linear_acceleration{ dx, dy, dz };

	double rx = imu_msg->angular_velocity.x;
	double ry = imu_msg->angular_velocity.y;
	double rz = imu_msg->angular_velocity.z;
	Eigen::Vector3d angular_velocity{ rx, ry, rz };

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba - tmp_Q.inverse() * estimator.g);

	Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
	tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba - tmp_Q.inverse() * estimator.g);

	Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

	tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
	tmp_V = tmp_V + dt * un_acc;

	acc_0 = linear_acceleration;
	gyr_0 = angular_velocity;
}

void update()
{
    latest_time = current_time;
    tmp_P = relocalize_r * estimator.Ps[WINDOW_SIZE] + relocalize_t;
    tmp_Q = relocalize_r * estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
	acc_0 = estimator.acc_0;
	gyr_0 = estimator.gyr_0;

	std::queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
	for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
		predict(tmp_imu_buf.front());
}

std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;

	while (true)
	{
		if (imu_buf.empty() || feature_buf.empty())
			return measurements;
		if (!(imu_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td)) {
			console::print_warn("WARN:wait for imu,only should happen at the beginning.\n");
			sum_of_wait++;
			return measurements;
		}

		if (!(imu_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator.td)) {
			console::print_warn("WARN:throw img,only should happen at the beginning.\n");
			feature_buf.pop();
			continue;
		}
		sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
		feature_buf.pop();

		std::vector<sensor_msgs::ImuConstPtr> IMUs;
		while (imu_buf.front()->header.stamp.toSec() <= img_msg->header.stamp.toSec() + estimator.td) {
			IMUs.emplace_back(imu_buf.front());
			imu_buf.pop();
		}
		measurements.emplace_back(IMUs, img_msg);
	}
    return measurements;
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
	m_buf.lock();
	imu_buf.push(imu_msg);
	m_buf.unlock();

	Json::Value JSON_imu;
	JSON_imu["timestamp"] = imu_msg->header.stamp.toSec();
	JSON_imu["acc_x"] = imu_msg->linear_acceleration.x;
	JSON_imu["acc_y"] = imu_msg->linear_acceleration.y;
	JSON_imu["acc_z"] = imu_msg->linear_acceleration.z;
	JSON_imu["gyr_x"] = imu_msg->angular_velocity.x;
	JSON_imu["gyr_y"] = imu_msg->angular_velocity.y;
	JSON_imu["gyr_z"] = imu_msg->angular_velocity.z;

	try {
		imuProducer.send(JSON_imu.toStyledString());
	}
	catch (...) {
		std::cout << "IMU_PRODUCER exception." << std::endl;
	}
	
	std::unique_lock<std::mutex> lock_imu(m_state);
	predict(imu_msg);
	/*std_msgs::Header header = imu_msg->header;
	header.frame_id = "world";
	if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
		pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);*/
	lock_imu.unlock();
}

void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
	con.notify_one();
}

void send_imu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
	if (current_time < 0)
		current_time = t;
    double dt = t - current_time;
    current_time = t;

    double ba[]{0.0, 0.0, 0.0};
    double bg[]{0.0, 0.0, 0.0};

	double dx = imu_msg->linear_acceleration.x - ba[0];
	double dy = imu_msg->linear_acceleration.y - ba[1];
	double dz = imu_msg->linear_acceleration.z - ba[2];

	double rx = imu_msg->angular_velocity.x - bg[0];
	double ry = imu_msg->angular_velocity.y - bg[1];
	double rz = imu_msg->angular_velocity.z - bg[2];
    //ROS_DEBUG("IMU %f, dt: %f, acc: %f %f %f, gyr: %f %f %f", t, dt, dx, dy, dz, rx, ry, rz);

	estimator.processIMU(dt, Eigen::Vector3d(dx, dy, dz), Eigen::Vector3d(rx, ry, rz));
}

// 闭环检测线程主函数
void process_loop_detection()
{
	if (loop_closure == NULL)
	{
		const char *voc_file = VOC_FILE.c_str();
		TicToc t_load_voc;
		console::print_highlight("Loop thread start loop.\n");
		console::print_info("INFO: Loop voc file: %s\n", voc_file);
		loop_closure = new LoopClosure(voc_file, IMAGE_COL, IMAGE_ROW);
		console::print_value("Loop load vocabulary: %d\n", int(t_load_voc.toc()));
		loop_closure->initCameraModel(CAM_NAMES_ESTIMATOR);
	}

	//while (LOOP_CLOSURE)
	//{
		// 取最近的一帧关键帧作为当前帧
		KeyFrame* cur_kf = NULL;
		m_keyframe_buf.lock();
		while (!keyframe_buf.empty())
		{
			if (cur_kf != NULL)
				delete cur_kf;
			cur_kf = keyframe_buf.front();
			keyframe_buf.pop();
		}
		m_keyframe_buf.unlock();

		if (cur_kf != NULL)
		{
			// 将当前关键帧加入到关键帧数据库中
			cur_kf->global_index = global_frame_cnt;
			m_keyframedatabase_resample.lock();
			keyframe_database.add(cur_kf);
			m_keyframedatabase_resample.unlock();

			cv::Mat current_image;
			current_image = cur_kf->image;

			// 提取当前帧的描述子
			bool loop_succ = false;
			int old_index = -1;
			std::vector<cv::Point2f> cur_pts;
			std::vector<cv::Point2f> old_pts;
			cur_kf->extractBrief(current_image);

			// 当前帧在关键帧数据库中寻找可能的回环帧
			TicToc t_loop_detect;
			loop_succ = loop_closure->startLoopClosure(cur_kf->keypoints, cur_kf->descriptors, cur_pts, old_pts, old_index);
			int t_loop_detect_spend = t_loop_detect.toc();
			console::print_info("INFO: detect loop status %d time: %d ms.\n", int(loop_succ), t_loop_detect_spend);
			
			if (loop_succ)
			{
				KeyFrame* old_kf = keyframe_database.getKeyframe(old_index);
				if (old_kf == NULL)
				{
					console::print_warn("WARN: no such frame in keyframe database.\n");
					return;
				}
				assert(old_index != -1);

				// 使用PnP计算闭环帧的姿态
				Eigen::Vector3d T_w_i_old, PnP_T_old;
				Eigen::Matrix3d R_w_i_old, PnP_R_old;
				old_kf->getPose(T_w_i_old, R_w_i_old);
				std::vector<cv::Point2f> measurements_old;
				std::vector<cv::Point2f> measurements_old_norm;
				std::vector<cv::Point2f> measurements_cur;
				std::vector<int> features_id_matched;
				cur_kf->findConnectionWithOldFrame(old_kf, measurements_old, measurements_old_norm, PnP_T_old, PnP_R_old, m_camera);
				measurements_cur = cur_kf->measurements_matched;
				features_id_matched = cur_kf->features_id_matched;

				//send loop info to VINS relocalization
				int loop_fusion = 0;
				if ((int)measurements_old_norm.size() > MIN_LOOP_NUM && global_frame_cnt - old_index > 35 && old_index > 30)
				{
					Eigen::Quaterniond PnP_Q_old(PnP_R_old);
					RetriveData retrive_data;
					retrive_data.cur_index = cur_kf->global_index;
					retrive_data.header = cur_kf->header;
					retrive_data.P_old = T_w_i_old;
					retrive_data.R_old = R_w_i_old;
					retrive_data.relative_pose = false;
					retrive_data.relocalized = false;
					retrive_data.measurements = measurements_old_norm;
					retrive_data.features_ids = features_id_matched;
					retrive_data.loop_pose[0] = PnP_T_old.x();
					retrive_data.loop_pose[1] = PnP_T_old.y();
					retrive_data.loop_pose[2] = PnP_T_old.z();
					retrive_data.loop_pose[3] = PnP_Q_old.x();
					retrive_data.loop_pose[4] = PnP_Q_old.y();
					retrive_data.loop_pose[5] = PnP_Q_old.z();
					retrive_data.loop_pose[6] = PnP_Q_old.w();

					m_retrive_data_buf.lock();
					retrive_data_buf.push(retrive_data);
					m_retrive_data_buf.unlock();

					cur_kf->detectLoop(old_index);
					old_kf->is_looped = 1;
					loop_fusion = 1;

					m_update_visualization.lock();
					keyframe_database.addLoop(old_index);
					//CameraPoseVisualization* posegraph_visualization = keyframe_database.getPosegraphVisualization();
					//pubPoseGraph(posegraph_visualization, cur_header);  
					m_update_visualization.unlock();
				}

				// 绘制闭环匹配关系
				if (0 && loop_fusion)
				{
					int imgCols = current_image.cols;
					cv::Mat gray_img, loop_match_img;
					cv::Mat old_img = old_kf->image;
					cv::hconcat(old_img, current_image, gray_img);
					cv::cvtColor(gray_img, loop_match_img, cv::COLOR_GRAY2RGB);
					
					for (int i = 0; i < (int)measurements_cur.size(); i++)
					{
						cv::Point2f cur_pt = measurements_cur[i];
						cur_pt.x += imgCols;
						cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
					}
					for (int i = 0; i < (int)measurements_old.size(); i++)
					{
						cv::circle(loop_match_img, measurements_old[i], 5, cv::Scalar(0, 255, 0));
					}
					for (int i = 0; i < (int)measurements_old.size(); i++)
					{
						cv::Point2f cur_pt = measurements_cur[i];
						cur_pt.x += imgCols;
						cv::line(loop_match_img, measurements_old[i], cur_pt, cv::Scalar(0, 255, 0), 1, 8, 0);
					}

					std::ostringstream convert;
					convert << "C:\\Users\\jackchen\\Desktop\\debug\\"
						<< cur_kf->global_index << "-"
						<< old_index << "-" << loop_fusion << ".jpg";
					cv::imwrite(convert.str().c_str(), loop_match_img);
				}

			}

			cur_kf->image.release();
			global_frame_cnt++;

			// 当关键帧数据库比较大时，此时需要对关键帧数据库进行下采样以满足实时需求
			if (t_loop_detect_spend > 1000 || keyframe_database.size() > MAX_KEYFRAME_NUM)
			{
				m_keyframedatabase_resample.lock();
				erase_index.clear();
				keyframe_database.downsample(erase_index);
				m_keyframedatabase_resample.unlock();
				if (!erase_index.empty()) {
					loop_closure->eraseIndex(erase_index);
				}
			}
		}
		//std::chrono::milliseconds dura(10);
		//std::this_thread::sleep_for(dura);
	//}
}

void process_pose_graph()
{
    //while(true)            
    //{
		// 取最新的闭环信息进行闭环优化
        m_posegraph_buf.lock();
        int index = -1;
        while (!optimize_posegraph_buf.empty())
        {
            index = optimize_posegraph_buf.front();
            optimize_posegraph_buf.pop();
        }
        m_posegraph_buf.unlock();

		if (index != -1)
		{
			Eigen::Vector3d correct_t = Eigen::Vector3d::Zero();
			Eigen::Matrix3d correct_r = Eigen::Matrix3d::Identity();

			TicToc t_posegraph;
			keyframe_database.optimize4DoFLoopPoseGraph(index, correct_t, correct_r);
			console::print_info("INFO: 4DoF loop pose graph optimized %d ms.\n", t_posegraph.toc());

			m_loop_drift.lock();
			relocalize_r = correct_r;
			relocalize_t = correct_t;
			m_loop_drift.unlock();

			m_update_visualization.lock();
			keyframe_database.updateVisualization();
			//CameraPoseVisualization* posegraph_visualization = keyframe_database.getPosegraphVisualization();
			m_update_visualization.unlock();

			pubOdometry(estimator, cur_header, relocalize_t, relocalize_r);
			//pubPoseGraph(posegraph_visualization, cur_header); 
			nav_msgs::Path refine_path = keyframe_database.getPath();
			updateLoopPath(refine_path);
		}

		//std::chrono::milliseconds dura(50);
		//std::this_thread::sleep_for(dura);
    //}
}

// 视觉惯导里程计主线程
void process()
{
    while (true)
    {
		// 获取IMU与相机同一时间戳的数据对
        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
		std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]{
            return (measurements = getMeasurements()).size() != 0;
		});
		lk.unlock();
		//measurements = getMeasurements();

		for (const auto &measurement : measurements) {
			TicToc t_process_static;

			// 加入惯导信息到系统中进行预积分
			for (const auto &imu_msg : measurement.first)
				send_imu(imu_msg);

			// 从配对的惯导-图像信息中取出图像特征信息
			const auto& img_msg = measurement.second;
			std::map<int, std::vector<std::pair<int, Eigen::Matrix<double,7,1>>>> image;
			for (unsigned int i = 0; i < img_msg->points.size(); ++i) {
				int v = img_msg->channels[0].values[i] + 0.5;
				int feature_id = v / NUM_OF_CAM;
				int camera_id = v % NUM_OF_CAM;
				double x = img_msg->points[i].x;
				double y = img_msg->points[i].y;
				double z = img_msg->points[i].z;
				double p_u = img_msg->channels[1].values[i];
				double p_v = img_msg->channels[2].values[i];
				double velocity_x = img_msg->channels[3].values[i];
				double velocity_y = img_msg->channels[4].values[i];
				assert(z == 1);

				Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
				xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
				image[feature_id].emplace_back(camera_id, xyz_uv_velocity);
			}

			// 加入图像特征信息到系统中进行优化
			estimator.processImage(image, img_msg->header);

            if (estimator.solver_flag == estimator.NON_LINEAR) {
                std::unique_lock<std::mutex> key_pose_lock(mutex_key_pose);
				global_keypose.emplace_back(estimator.Ps[WINDOW_SIZE]);
				local_landmarks = std::move(estimator.local_cloud);
				global_landmarks = estimator.global_cloud;
				td = estimator.td;
				key_pose_lock.unlock();
            }

			if (LOOP_CLOSURE) {
				// 移除之前的闭环信息
				std::vector<RetriveData>::iterator it = estimator.retrive_data_vector.begin();
				for (; it != estimator.retrive_data_vector.end(); ) {
					if ((*it).header < estimator.Headers[0].stamp.toSec()) {
						it = estimator.retrive_data_vector.erase(it);
					}
					else
						it++;
				}

				m_retrive_data_buf.lock();
				while (!retrive_data_buf.empty()) {
					estimator.retrive_data_vector.emplace_back(retrive_data_buf.front());
					retrive_data_buf.pop();
				}
				m_retrive_data_buf.unlock();

				// 次新帧为关键帧时
				if (estimator.marginalization_flag == estimator.MARGIN_OLD && 
					estimator.solver_flag == estimator.NON_LINEAR) {
					Eigen::Vector3d vio_T_w_i = estimator.Ps[WINDOW_SIZE - 2];
					Eigen::Matrix3d vio_R_w_i = estimator.Rs[WINDOW_SIZE - 2];
					i_buf.lock();
					while (!image_buf.empty() && image_buf.front().second < estimator.Headers[WINDOW_SIZE - 2].stamp.toSec())
					{
						image_buf.pop();
					}
					i_buf.unlock();

					cv::Mat KeyFrame_image;
					KeyFrame_image = image_buf.front().first;

					// 使用滑窗中的次新帧构造关键帧，包含次新帧的前端位姿，之前的回环信息以及回环纠正后的位姿
					const char *pattern_file = PATTERN_FILE.c_str();
					Eigen::Vector3d cur_T;
					Eigen::Matrix3d cur_R;
					cur_T = relocalize_r * vio_T_w_i + relocalize_t;
					cur_R = relocalize_r * vio_R_w_i;
					KeyFrame* keyframe = new KeyFrame(estimator.Headers[WINDOW_SIZE - 2].stamp.toSec(), vio_T_w_i, vio_R_w_i, cur_T, cur_R, image_buf.front().first, pattern_file, relocalize_t, relocalize_r);
					keyframe->setExtrinsic(estimator.tic[0], estimator.ric[0]);
					keyframe->buildKeyFrameFeatures(estimator, m_camera);

					m_keyframe_buf.lock();
					keyframe_buf.push(keyframe);
					m_keyframe_buf.unlock();

					// 填充optimize_posegraph_buf数据，为位姿图优化提供数据来源
					if (!estimator.retrive_data_vector.empty() && estimator.retrive_data_vector[0].relative_pose)
					{
						if (estimator.Headers[0].stamp.toSec() == estimator.retrive_data_vector[0].header)
						{
							KeyFrame* cur_kf = keyframe_database.getKeyframe(estimator.retrive_data_vector[0].cur_index);
							if (std::abs(estimator.retrive_data_vector[0].relative_yaw) > 30.0 || estimator.retrive_data_vector[0].relative_t.norm() > 20.0)
							{
								console::print_info("INFO: wrong loop.\n");
								cur_kf->removeLoop();
							}
							else
							{
								cur_kf->updateLoopConnection(estimator.retrive_data_vector[0].relative_t,
									estimator.retrive_data_vector[0].relative_q,
									estimator.retrive_data_vector[0].relative_yaw);

								m_posegraph_buf.lock();
								optimize_posegraph_buf.push(estimator.retrive_data_vector[0].cur_index);
								m_posegraph_buf.unlock();
							}
						}
					}
				}
			}

			std_msgs::Header header = img_msg->header;
			header.frame_id = "world";
			cur_header = header;

			m_loop_drift.lock();
			if (estimator.relocalize) {
				relocalize_t = estimator.relocalize_t;
				relocalize_r = estimator.relocalize_r;
			}
			pubOdometry(estimator, header, relocalize_t, relocalize_r);
			m_loop_drift.unlock();

			float t_process = t_process_static.toc();
			console::print_highlight("INFO: process thread time: %.1f ms\n", t_process);

			std::unique_lock<std::mutex> lock_freq(m_real_freq);
			process_freq = 1000.0 / t_process;
			lock_freq.unlock();
		}
	
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
		m_state.unlock();
		m_buf.unlock();
    }
}

void img_callback(const cv::Mat &show_img, const ros::Time &timestamp)
{
	TicToc t_feature_static;
	try {
		//imageProducer.send(show_img, ".jpg");
		imageProducer.send(show_img, timestamp.toSec(), ".jpg");
	}
	catch (...){
		std::cout << "IMAGE_PRODUCER exception." << std::endl;
	}
	
	if (LOOP_CLOSURE) {
		i_buf.lock();
		image_buf.push(std::make_pair(show_img, timestamp.toSec()));
		i_buf.unlock();
	}
	if (first_image_flag) {
		first_image_flag = false;
		first_image_time = timestamp.toSec();
		last_image_time = timestamp.toSec();
		return;
	}

	if (timestamp.toSec() - last_image_time > 1.0 ||
		timestamp.toSec() < last_image_time) {
		console::print_warn("WARN: image discontinue! reset the feature track.");
		first_image_flag = true;
		last_image_time = 0;
		pub_count = 1;
		return;
	}
	last_image_time = timestamp.toSec();

	// 控制图像输入频率
	if (std::round(1.0 * pub_count / (timestamp.toSec() - first_image_time)) <= FREQ) {
		PUB_THIS_FRAME = true;
		// 时间间隔内发布频率接近设定频率时，更新时间间隔起始时刻，并将数据发布次数置0
		if (std::abs(1.0 * pub_count / (timestamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ) {
			first_image_time = timestamp.toSec();
			pub_count = 0;
		}
	}
	else {
		PUB_THIS_FRAME = false;
	}

	// 光流跟踪特征坐标
	for (int i = 0; i < NUM_OF_CAM; ++i) {
		if (i != 1 || !STEREO_TRACK)
			trackerData[i].readImage(show_img.rowRange(ROW * i, ROW * (i + 1)), timestamp.toSec());
		else {
			if (EQUALIZE) {
				cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
				clahe->apply(show_img.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
			}
			else
				trackerData[i].cur_img = show_img.rowRange(ROW * i, ROW * (i + 1));
		}

#if SHOW_UNDISTORTION
		trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
	}

    // 更新特征点ID
	for (unsigned int i = 0;; i++) {
		bool completed = false;
		for (int j = 0; j < NUM_OF_CAM; j++)
			if (j != 1 || !STEREO_TRACK)
				completed |= trackerData[j].updateID(i);
		if (!completed)
			break;
	}

	if (PUB_THIS_FRAME) {
		pub_count++;

		if (!init_pub) {
			init_pub = true;
		}
		else {
			sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
			sensor_msgs::ChannelFloat32 id_of_point;
			sensor_msgs::ChannelFloat32 u_of_point;
			sensor_msgs::ChannelFloat32 v_of_point;
			sensor_msgs::ChannelFloat32 velocity_x_of_point;
			sensor_msgs::ChannelFloat32 velocity_y_of_point;

			feature_points->header.stamp = timestamp;
			feature_points->header.frame_id = "world";

			std::vector<std::set<int>> hash_ids(NUM_OF_CAM);
			for (int i = 0; i < NUM_OF_CAM; i++) {
				if (i != 1 || !STEREO_TRACK) {
					auto &un_pts = trackerData[i].cur_un_pts;
					auto &cur_pts = trackerData[i].cur_pts;
					auto &ids = trackerData[i].ids;
					auto &pts_velocity = trackerData[i].pts_velocity;
					for (int j = 0; j < int(ids.size()); ++j) {
						if (trackerData[i].track_cnt[j] > 1) {
							int p_id = ids[j];
							geometry_msgs::Point32 p;
							p.x = un_pts[j].x;
							p.y = un_pts[j].y;
							p.z = 1;

							feature_points->points.emplace_back(p);
							id_of_point.values.emplace_back(p_id);
							u_of_point.values.emplace_back(cur_pts[j].x);
							v_of_point.values.emplace_back(cur_pts[j].y);
							velocity_x_of_point.values.emplace_back(pts_velocity[j].x);
							velocity_y_of_point.values.emplace_back(pts_velocity[j].y);
						}
					}
				}
			}
			feature_points->channels.emplace_back(id_of_point);
			feature_points->channels.emplace_back(u_of_point);
			feature_points->channels.emplace_back(v_of_point);
			feature_points->channels.emplace_back(velocity_x_of_point);
			feature_points->channels.emplace_back(velocity_y_of_point);
			feature_callback(feature_points);

			// 可视化显示图像特征点
			cv::Mat tmp_img;
			cv::cvtColor(show_img, tmp_img, cv::COLOR_GRAY2RGB);
			for (unsigned int j = 0; j < trackerData[0].cur_pts.size(); ++j) {
				double len = std::min(1.0, 1.0 * trackerData[0].track_cnt[j] / WINDOW_SIZE_FEATURE_TRACKER);
				cv::circle(tmp_img, trackerData[0].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
			}
			std::unique_lock<std::mutex> lock(m_img_vis);
			img_visualization = tmp_img;
			cv::flip(img_visualization, img_visualization, 0);
			lock.unlock();
		}
	}
	
	float t_feature = t_feature_static.toc();
	console::print_highlight("INFO: feature track thread time: %.1f ms\n", t_feature);

	std::unique_lock<std::mutex> lock_freq(m_real_freq);
	feature_freq = 1000.0 / t_feature;
	lock_freq.unlock();
}

void LoadImages(const std::string &strImagePath, const std::string &strTimesStampsPath,
	std::vector<std::string> &strImagesFileNames, std::vector<double> &timeStamps)
{
	std::ifstream fTimes;
	fTimes.open(strTimesStampsPath.c_str());
	timeStamps.reserve(5000);
	strImagesFileNames.reserve(5000);
	while (!fTimes.eof())
	{
		std::string s;
		getline(fTimes, s);
		if (!s.empty())
		{
			std::stringstream ss;
			ss << s;
			strImagesFileNames.emplace_back(strImagePath + "/" + ss.str() + ".png");
			double t;
			ss >> t;
			timeStamps.emplace_back(t / 1e9);
		}
	}
}

void LoadImages(const std::string& path, std::vector<std::string>& imageFileNames,
	std::vector<double>& timeStamps)
{
	std::ifstream fImages;
	std::string filename = path + "/data.csv";
	fImages.open(filename.c_str());
	timeStamps.reserve(5000);
	imageFileNames.reserve(5000);
	while (!fImages.eof())
	{
		std::string s;
		getline(fImages, s);
		if (!s.empty())
		{
			char c = s.at(0);
			if (c < '0' || c > '9')
				continue;
			std::stringstream ss;
			ss << s;
			double t;
			ss >> t;
			timeStamps.emplace_back(t / 1e9);

			if (ss.peek() == ',' || ss.peek() == ' ')
				ss.ignore();

			std::string name;
			ss >> name;
			imageFileNames.emplace_back(path + "/data/" + name.c_str());
		}
	}
}

void LoadImus(std::ifstream & fImus, const ros::Time &imageTimestamp)
{
	while (!fImus.eof())
	{
		std::string s;
		getline(fImus, s);
		if (!s.empty())
		{
			// 去除文件首行
			char c = s.at(0);
			if (c < '0' || c > '9')
				continue;
			std::stringstream ss;
			ss << s;
			double tmpd;
			int cnt = 0;
			double data[7];
			while (ss >> tmpd)
			{
				data[cnt] = tmpd;
				cnt++;
				if (cnt == 7)
					break;
				if (ss.peek() == ',' || ss.peek() == ' ')
					ss.ignore();
			}

			data[0] *= 1e-9;
			sensor_msgs::ImuPtr imudata(new sensor_msgs::Imu);
			imudata->angular_velocity.x = data[1];
			imudata->angular_velocity.y = data[2];
			imudata->angular_velocity.z = data[3];
			imudata->linear_acceleration.x = data[4];
			imudata->linear_acceleration.y = data[5];
			imudata->linear_acceleration.z = data[6];
			uint32_t  sec = data[0];
			uint32_t nsec = (data[0] - sec)*1e9;
			//nsec = (nsec / 1000) * 1000 + 500;
			imudata->header.stamp = ros::Time(sec, nsec);
			imu_callback(imudata);

			// 获取imageTimestamp之前的所有IMU信息
			if (imudata->header.stamp > imageTimestamp)
				break;
		}
	}
}

int main(int argc, char **argv)
{
	if (argc != 4) {
		console::print_error("Usage: ./vins_estimator path_to_setting_file path_to_image_folder path_to_times_file path_to_imu_data_file\n");
		return -1;
	}

	std::ifstream fImus;
	fImus.open(argv[3]);

	cv::Mat image;
	int ni = 0;

	readParameters(argv[1]);

	estimator.setParameter();
	for (int i = 0; i < NUM_OF_CAM; i++)
		trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

	imuProducer = RabbitMQProducer("IMU_PRODUCER", "localhost", 5672, "imu_queue", "imu_exchange", "direct", "imu");
	imuProducer.credential("xinxin", "xhl6457398yy");
	imuProducer.run();

	imageProducer = RabbitMQProducer("IMAGE_PRODUCER", "localhost", 5672, "image_queue", "image_exchange", "direct", "image");
	imageProducer.credential("xinxin", "xhl6457398yy");
	imageProducer.run();

	std::vector<double> vTimeStamps;
	std::vector<std::string> vStrImagesFileNames;
	LoadImages(std::string(argv[2]), vStrImagesFileNames, vTimeStamps);
	m_camera = CameraFactory::instance()->generateCameraFromYamlFile(CAM_NAMES_ESTIMATOR);

	int imageNum = vStrImagesFileNames.size();
	if (imageNum <= 0){
		console::print_error("ERROR: Failed to load images\n");
		return 1;
	}
	else {
		console::print_highlight("Load image num: ");
		console::print_value("%d\n", imageNum);
	}

    std::thread callback_thread([&](){
        for (ni = 0; ni < imageNum; ++ni) {
            double tframe = vTimeStamps[ni];   //timestamp
            uint32_t sec = tframe;
            uint32_t nsec = (tframe - sec)*1e9;
            //nsec = (nsec / 1000) * 1000 + 500;
			ros::Time image_timestamp = ros::Time(sec, nsec);

            // 读取IMU数据以及对应的相机数据
            LoadImus(fImus, image_timestamp);
            image = cv::imread(vStrImagesFileNames[ni], cv::IMREAD_GRAYSCALE);

            if (image.empty()) {
                console::print_error("ERROR: failed to load image: %s\n", vStrImagesFileNames[ni].c_str());
				return -1;
            }

            img_callback(image, image_timestamp);
        }
	});
    callback_thread.detach();

	std::thread process_thread(process);
	process_thread.detach();
	
	visualization();

	return 0;
}
