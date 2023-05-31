#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"

#include "include/PointCloud.h"
#include "include/Imu.h"
#include "utility/print.h"
#include "utility/tic_toc.h"
#include "utility/camera_factory.h"
#include "utility/pinhole_camera.h"
#include "feature_tracker/feature_tracker.h"

#include <pangolin/pangolin.h>

FeatureTracker tracker;
Estimator estimator;
PinholeCameraPtr m_camera;

int pub_count = 1;
int sum_of_wait = 0;
double first_image_time;
bool first_image_flag = true;

std::mutex mutex_buf;
std::mutex mutex_state;
std::queue<sensor_msgs::ImuConstPtr> imu_buf;
std::queue<sensor_msgs::PointCloudConstPtr> feature_buf;

std::condition_variable con;

double current_time = -1;
double latest_time = -1;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;

std::mutex mutex_player;
bool player_status = true;
cv::Mat img_player;
Eigen::Vector3d tcur;
Eigen::Matrix3d rcur;
Eigen::Matrix3d ric;
Eigen::Vector3d ypr;
Eigen::Vector3d tic;
float process_freq = -1;
float player_freq = -1;
float feature_freq = -1;
std::vector<Eigen::Vector3d> global_keypose;
std::unordered_map<int, Eigen::Vector3d> local_landmarks;
std::unordered_map<int, Eigen::Vector3d> global_landmarks;

void saveGlobalLandmarks();
void viewCameraLandmark();
void viewCameraPath();
void viewCameraPose(pangolin::OpenGlMatrix& M);
void drawCurrentCamera(pangolin::OpenGlMatrix& Twc);
void player();

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

void viewCameraLandmark()
{
	glPointSize(1.5f);
	glBegin(GL_POINTS);
	glColor3f(0.0, 0.0, 1.0);
	for (const auto& pairs : local_landmarks) {
		const auto& landmark = pairs.second;
		if (std::abs(landmark.x()) > 10 ||
			std::abs(landmark.y()) > 10 ||
			std::abs(landmark.z()) > 10) {
			continue;
		}
		glVertex3f(landmark.x(), landmark.y(), landmark.z());
	}
	glEnd();

	glPointSize(1.5f);
	glBegin(GL_POINTS);
	glColor3f(0.0, 0.0, 0.0);
	for (const auto& pairs : global_landmarks) {
		const auto& landmark = pairs.second;
		if (std::abs(landmark.x()) > 10 ||
			std::abs(landmark.y()) > 10 ||
			std::abs(landmark.z()) > 10) {
			continue;
		}
		glVertex3f(landmark.x(), landmark.y(), landmark.z());
	}
	glEnd();
}
void viewCameraPath()
{
	GLfloat line_width = 1.2;
	glColor3f(1.0f, 0.0f, 0.0f);
	glLineWidth(line_width);
	glBegin(GL_LINE_STRIP);

	for (const auto& it : global_keypose) {
		glVertex3f(it.x(), it.y(), it.z());
	}
	glEnd();
}
void viewCameraPose(pangolin::OpenGlMatrix& M)
{
	Eigen::Vector3d P = tcur + rcur * tic;
	Eigen::Matrix3d R = rcur * ric;

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
void drawCurrentCamera(pangolin::OpenGlMatrix& Twc)
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
	glColor3f(0.0f, 0.0f, 1.0f);	//blue
	glBegin(GL_LINES);			//draw camera 
	glVertex3f(0, 0, 0);
	glVertex3f(w, h, z);
	glVertex3f(0, 0, 0);
	glVertex3f(w, -h, z);
	glVertex3f(0, 0, 0);
	glVertex3f(-w, -h, z);
	glVertex3f(0, 0, 0);
	glVertex3f(-w, h, z);

	glVertex3f(w, h, z);
	glVertex3f(w, -h, z);
	glVertex3f(-w, h, z);
	glVertex3f(-w, -h, z);
	glVertex3f(-w, h, z);
	glVertex3f(w, h, z);
	glVertex3f(-w, -h, z);
	glVertex3f(w, -h, z);
	glEnd();
	glPopMatrix();
}
void player()
{
	int img_rows = 480;
	int img_cols = 752;
	float mViewpointX = -0;
	float mViewpointY = -5;
	float mViewpointZ = -10;
	float mViewpointF = 500;
	img_player = cv::Mat::zeros(img_rows, img_cols, CV_8UC3);
	pangolin::CreateWindowAndBind("VINS: Map Visualization", 1024, 768);
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

	pangolin::Plotter plotterFREQ(&logFREQ, 0.0f, 100.0f, 0.0f, 20.0f, 10.0f, 0.5f);
	plotterFREQ.SetBounds(float(460.0 / 768.0), float(660.0 / 768.0), float(10.0 / 1024.0), float(290.0 / 1024.0));
	plotterFREQ.Track("$i");

	pangolin::DisplayBase().AddDisplay(plotterRic);
	pangolin::DisplayBase().AddDisplay(plotterTic);
	pangolin::DisplayBase().AddDisplay(plotterFREQ);

	pangolin::OpenGlRenderState s_cam(
		pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000),
		pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, VISUALLOOKATX, VISUALLOOKATY, VISUALLOOKATZ)
	);

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
	Twc.SetIdentity();

	while (!pangolin::ShouldQuit() & player_status) {
		TicToc t_show_static;

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		viewCameraPose(Twc);
		glClearColor(1.0f, 1.0f, 1.0f, 0.5f);

		std::unique_lock<std::mutex> player_lock(mutex_player);

		logRic.Log(ypr(0), ypr(1), ypr(2));
		logTic.Log(tic(0), tic(1), tic(2));
		logFREQ.Log(feature_freq, process_freq, player_freq);

		if (menuFollowCamera) {
			s_cam.Follow(Twc);
		}

		d_cam.Activate(s_cam);
		drawCurrentCamera(Twc);
		if (menuShowPoints)
			viewCameraLandmark();

		if (menuShowPath) 
			viewCameraPath();

		d_image.Activate();
		glColor3f(1.0f, 1.0f, 1.0f);
		
		memcpy(imageArray, img_player.data, sizeof(uchar) * 3 * img_rows*img_cols);
		imageTexture.Upload(imageArray, GL_RGB, GL_UNSIGNED_BYTE);
		imageTexture.RenderToViewport();

		player_lock.unlock();
		pangolin::FinishFrame();
		//console::print_highlight("INFO: visualization thread time: %.1f ms\n", t_show);
	}

	saveGlobalLandmarks();
	console::print_highlight("Visualization thread end.\n");
}

void getLatestPose();
void predict(const sensor_msgs::ImuConstPtr& imu_msg);

void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg);
void feature_callback(const sensor_msgs::PointCloudConstPtr& feature_msg);
void image_callback(const cv::Mat& img_msg, const ros::Time& timestamp);

void track_imu(const sensor_msgs::ImuConstPtr& imu_msg);
std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>>
getMeasurements();

void load_images(const std::string& path, std::vector<std::string>& imageFileNames, 
	std::vector<double>& timeStamps);

void load_imus(std::ifstream& fImus, const ros::Time& imageTimestamp);

void frontend_track();

void getLatestPose()
{
	latest_time = current_time;
	tmp_P = estimator.Ps[WINDOW_SIZE];
	tmp_Q = estimator.Rs[WINDOW_SIZE];
	tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
	tmp_Bg = estimator.Bgs[WINDOW_SIZE];

	acc_0 = estimator.acc_0;
	gyr_0 = estimator.gyr_0;

	auto tmp_imu_buf = imu_buf;
	for (; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
		predict(tmp_imu_buf.front());
}

void predict(const sensor_msgs::ImuConstPtr& imu_msg)
{
	double t = imu_msg->header.stamp.toSec();
	double dt = t - latest_time;
	latest_time = t;

	double dx = imu_msg->linear_acceleration.x;
	double dy = imu_msg->linear_acceleration.y;
	double dz = imu_msg->linear_acceleration.z;
	Eigen::Vector3d linear_acceleration{ dx,dy,dz };
	double rx = imu_msg->angular_velocity.x;
	double ry = imu_msg->angular_velocity.y;
	double rz = imu_msg->angular_velocity.z;
	Eigen::Vector3d angular_velocity{ rx,ry,rz };

	Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba - tmp_Q.inverse()*estimator.g);
	Eigen::Vector3d un_gyr = 0.5*(gyr_0 + angular_velocity) - tmp_Bg;
	
	tmp_Q = tmp_Q * Utility::deltaQ(un_gyr*dt);

	Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba - tmp_Q.inverse()*estimator.g);
	Eigen::Vector3d un_acc = 0.5*(un_acc_0 + un_acc_1);

	tmp_P = tmp_P + tmp_V * dt + 0.5*dt*dt*un_acc;
	tmp_V = tmp_V + un_acc * dt;

	acc_0 = linear_acceleration;
	gyr_0 = angular_velocity;
}

void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg)
{
	std::unique_lock<std::mutex> buf_lock(mutex_buf);
	imu_buf.push(imu_msg);
	buf_lock.unlock();
}

void feature_callback(const sensor_msgs::PointCloudConstPtr& feature_msg)
{
	std::unique_lock<std::mutex> buf_lock(mutex_buf);
	feature_buf.push(feature_msg);
	buf_lock.unlock();

	con.notify_one();
}

void image_callback(const cv::Mat& img_msg, const ros::Time& timestamp)
{
	if (first_image_flag)
	{
		first_image_flag = false;
		first_image_time = timestamp.toSec();
	}

	if (std::round(1.0*pub_count / (timestamp.toSec() - first_image_time)) <= FREQ)
	{
		PUB_THIS_FRAME = true;
		if (std::abs(1.0*pub_count / (timestamp.toSec() - first_image_time) - FREQ) < 0.01*FREQ)
		{
			first_image_time = timestamp.toSec();
			pub_count = 0;
		}
	}
	else 
	{
		PUB_THIS_FRAME = false;
	}

	tracker.readImage(img_msg, timestamp.toSec());

	for (unsigned int i = 0;; i++)
	{
		bool completed = false;
		completed |= tracker.updateID(i);
		if (!completed)
			break;
	}

	if (PUB_THIS_FRAME)
	{
		pub_count++;
		sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
		sensor_msgs::ChannelFloat32 id_of_point;
		sensor_msgs::ChannelFloat32 u_of_point;
		sensor_msgs::ChannelFloat32 v_of_point;

		feature_points->header.stamp = timestamp;
		feature_points->header.frame_id = "world";

		std::set<int> hash_ids;
		auto &un_pts = tracker.cur_un_pts;
		auto &cur_pts = tracker.cur_pts;
		auto &ids = tracker.ids;
		for (int j = 0; j < int(ids.size()); ++j) {
			int p_id = ids[j];
			hash_ids.insert(p_id);
			geometry_msgs::Point32 p;
			p.x = un_pts[j].x;
			p.y = un_pts[j].y;
			p.z = 1;

			feature_points->points.emplace_back(p);
			id_of_point.values.emplace_back(p_id);
			u_of_point.values.emplace_back(cur_pts[j].x);
			v_of_point.values.emplace_back(cur_pts[j].y);
		}

		feature_points->channels.emplace_back(id_of_point);
		feature_points->channels.emplace_back(u_of_point);
		feature_points->channels.emplace_back(v_of_point);
		feature_callback(feature_points);

		std::unique_lock<std::mutex> player_lock(mutex_player);
		cv::cvtColor(img_msg, img_player, cv::COLOR_GRAY2RGB);
		for (unsigned int i = 0; i < tracker.cur_pts.size(); ++i) {
			double len = std::min(1.0, 1.0 * tracker.track_cnt[i] / WINDOW_SIZE_FEATURE_TRACKER);
			cv::circle(img_player, tracker.cur_pts[i], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
		}
		cv::flip(img_player, img_player, 0);
		player_lock.unlock();
	}
}

void track_imu(const sensor_msgs::ImuConstPtr& imu_msg)
{
	double t = imu_msg->header.stamp.toSec();
	if (current_time < 0)
		current_time = t;
	double dt = t - current_time;
	current_time = t;
	
	double ba[]{ 0.0,0.0,0.0 };
	double bg[]{ 0.0,0.0,0.0 };

	double dx = imu_msg->linear_acceleration.x - ba[0];
	double dy = imu_msg->linear_acceleration.y - ba[1];
	double dz = imu_msg->linear_acceleration.z - ba[2];

	double rx = imu_msg->angular_velocity.x - bg[0];
	double ry = imu_msg->angular_velocity.y - bg[1];
	double rz = imu_msg->angular_velocity.z - bg[2];
	
	estimator.processIMU(dt, Eigen::Vector3d(dx, dy, dz), Eigen::Vector3d(rx, ry, rz));
}

std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>>
getMeasurements()
{
	std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;

	while (true)
	{
		if (imu_buf.empty() || feature_buf.empty())
			return measurements;
		if (!(imu_buf.back()->header.stamp > feature_buf.front()->header.stamp)) {
			console::print_warn("WARN:wait for imu,only should happen at the beginning.\n");
			sum_of_wait++;
			return measurements;
		}

		if (!(imu_buf.front()->header.stamp < feature_buf.front()->header.stamp)) {
			console::print_warn("WARN:throw img,only should happen at the beginning.\n");
			feature_buf.pop();
			continue;
		}
		sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
		feature_buf.pop();

		std::vector<sensor_msgs::ImuConstPtr> IMUs;
		while (imu_buf.front()->header.stamp <= img_msg->header.stamp) {
			IMUs.emplace_back(imu_buf.front());
			imu_buf.pop();
		}
		measurements.emplace_back(IMUs, img_msg);
	}
	return measurements;
}

void frontend_track()
{
	while (true)
	{
		std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
		std::unique_lock<std::mutex> buf_lock(mutex_buf);
		con.wait(buf_lock, [&] {
			return (measurements = getMeasurements()).size() != 0;
		});
		buf_lock.unlock();

		for (const auto& measurement : measurements)
		{
			for (const auto& imu_msg : measurement.first)
				track_imu(imu_msg);

			const auto& img_msg = measurement.second;
			std::map<int, std::vector<std::pair<int, Eigen::Matrix<double,7,1>>>> image;
			for (unsigned int i = 0; i < img_msg->points.size(); ++i) {
				int feature_id = img_msg->channels[0].values[i];
				double x = img_msg->points[i].x;
				double y = img_msg->points[i].y;
				double z = img_msg->points[i].z;
				double u = img_msg->channels[1].values[i];
				double v = img_msg->channels[2].values[i];
				double vx = img_msg->channels[3].values[i];
				double vy = img_msg->channels[4].values[i];
				assert(z == 1);
				Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
				xyz_uv_velocity << x, y, z, u, v, vx, vy;
				image[feature_id].emplace_back(0, xyz_uv_velocity);
			}

			estimator.processImage(image, img_msg->header);

			if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
			{
				std::unique_lock<std::mutex> player_lock(mutex_player);
				global_keypose.emplace_back(estimator.Ps[WINDOW_SIZE]);
				local_landmarks = std::move(estimator.local_cloud);
				global_landmarks = estimator.global_cloud;
				tic = estimator.tic[0];
				ric = estimator.ric[0];
				tcur = estimator.Ps[WINDOW_SIZE];
				rcur = estimator.Rs[WINDOW_SIZE];
				ypr = Utility::R2ypr(estimator.ric[0]);
				player_lock.unlock();
			}
		}

		mutex_buf.lock();
		mutex_state.lock();
		if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
			getLatestPose();
		mutex_state.unlock();
		mutex_buf.unlock();
	}
}

void load_images(const std::string& path, std::vector<std::string>& imageFileNames,
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

void load_imus(std::ifstream& fImus, const ros::Time& imageTimestamp)
{
	while (!fImus.eof())
	{
		std::string s;
		getline(fImus, s);
		if (!s.empty())
		{
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

			if (imudata->header.stamp > imageTimestamp)
				break;
		}
	}
}

int main(int argc, char **argv)
{
	if (argc != 4)
	{
		console::print_error("Usage: ./vins_estimator path_to_setting_file path_to_image_folder path_to_imu_data_file\n");
		return -1;
	}

	std::ifstream fImus;
	fImus.open(argv[3]);
	readParameters(argv[1]);

	estimator.setParameter();
	tracker.readIntrinsicParameter(CAM_NAMES[0]);

	std::cout << "a" << std::endl;

	std::vector<std::string> vStrImagesFileNames;
	std::vector<double> vTimeStamps;
	load_images(std::string(argv[2]), vStrImagesFileNames, vTimeStamps);
	m_camera = CameraFactory::instance()->generateCameraFromYamlFile(CAM_NAMES_ESTIMATOR);

	int imageNum = vStrImagesFileNames.size();
	if (imageNum <= 0) {
		console::print_error("ERROR: Failed to load images\n");
		return 1;
	}
	else {
		console::print_highlight("Load image num: ");
		console::print_value("%d\n", imageNum);
	}

	std::thread callback_thread([&]() {
		for (int it = 0; it < imageNum; ++it) {
			double tframe = vTimeStamps[it];
			uint32_t sec = tframe;
			uint32_t nsec = (tframe - sec)*1e9;
			//nsec = (nsec / 1000) * 1000 + 500;
			ros::Time image_timestamp = ros::Time(sec, nsec);

			load_imus(fImus, image_timestamp);
			cv::Mat image = cv::imread(vStrImagesFileNames[it], cv::IMREAD_GRAYSCALE);

			if (image.empty()) {
				console::print_error("ERROR: failed to load image: %s\n", vStrImagesFileNames[it].c_str());
				return -1;
			}

			image_callback(image, image_timestamp);
		}
	});
	callback_thread.detach();

	std::thread frontend_thread(frontend_track);
	frontend_thread.detach();

	player();

	return 0;
}
