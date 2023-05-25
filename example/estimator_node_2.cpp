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

bool pub_this_frame = false;

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
cv::Mat img_player;
Eigen::Vector3d ypr;
Eigen::Vector3d tic;
float process_freq = -1;
float player_freq = -1;
float feature_freq = -1;
std::vector<Eigen::Vector3d> global_keypose;
std::unordered_map<int, Eigen::Vector3d> local_landmarks;
std::unordered_map<int, Eigen::Vector3d> global_landmarks;

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
		pub_this_frame = true;
		if (std::abs(1.0*pub_count / (timestamp.toSec() - first_image_time) - FREQ) < 0.01*FREQ)
		{
			first_image_time = timestamp.toSec();
			pub_count = 0;
		}
	}
	else 
	{
		pub_this_frame = false;
	}

	tracker.readImage(img_msg);

	for (unsigned int i = 0;; i++)
	{
		bool completed = false;
		completed |= tracker.updateID(i);
		if (!completed)
			break;
	}

	if (pub_this_frame)
	{
		pub_count++;
		sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
		sensor_msgs::ChannelFloat32 id_of_point;
		sensor_msgs::ChannelFloat32 u_of_point;
		sensor_msgs::ChannelFloat32 v_of_point;

		feature_points->header.stamp = timestamp;
		feature_points->header.frame_id = "world";

		std::set<int> hash_ids;
		auto un_pts = tracker.undistortedPoints();
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
			std::map<int, std::vector<std::pair<int, Eigen::Vector3d>>> image;
			for (unsigned int i = 0; i < img_msg->points.size(); ++i) {
				int v = img_msg->channels[0].values[i] + 0.5;
				int feature_id = v / NUM_OF_CAM;
				int camera_id = v % NUM_OF_CAM;
				double x = img_msg->points[i].x;
				double y = img_msg->points[i].y;
				double z = img_msg->points[i].z;
				assert(z == 1);
				image[feature_id].emplace_back(camera_id, Eigen::Vector3d(x, y, z));
			}

			estimator.processImage(image, img_msg->header);

			if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
			{
				std::unique_lock<std::mutex> player_lock(mutex_player);
				global_keypose.emplace_back(estimator.Ps[WINDOW_SIZE]);
				local_landmarks = std::move(estimator.local_cloud);
				global_landmarks = estimator.global_cloud;
				tic = estimator.tic[0];
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
	frontend_thread.join();
}