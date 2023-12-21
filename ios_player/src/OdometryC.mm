//
//  ODOMETRY.m
//  vins
//
//  Created by 熊鑫鑫 on 2023/5/22.
//

#import <queue>
#import <map>
#import <thread>
#import <mutex>
#import <condition_variable>

#import <opencv2/core/mat.hpp>
#import <opencv2/opencv.hpp>
#import <opencv2/imgcodecs/ios.h>

#import "estimator.h"
#import "parameters.h"
#import "feature_tracker/feature_tracker.h"

#import "include/Imu.h"
#import "include/PointCloud.h"

#import "utility/print.h"
#import "utility/tic_toc.h"
#import "utility/camera_factory.h"
#import "utility/pinhole_camera.h"

#import "OdometryC.h"

#import <UIKit/UIKit.h>
#import <SceneKit/SceneKit.h>
#import <Foundation/Foundation.h>

@implementation ODOMETRY
{
    UIImage *uiimage;
    cv::Mat cvimage;
    Estimator estimator;
    FeatureTracker tracker;
    PinholeCameraPtr cameraModel;
    
    int pub_count;
    int sum_of_wait;
    double first_image_time;
    bool first_image_flag;
    
    std::condition_variable con;
    
    std::mutex mutex_buf;
    std::mutex mutex_state;
    std::queue<sensor_msgs::ImuConstPtr> imu_buf;
    std::queue<sensor_msgs::PointCloudConstPtr> feature_buf;
    
    double current_time;
    double latest_time;
    
    Eigen::Vector3d tmp_P;
    Eigen::Quaterniond tmp_Q;
    Eigen::Vector3d tmp_V;
    Eigen::Vector3d tmp_Ba;
    Eigen::Vector3d tmp_Bg;
    Eigen::Vector3d acc_0;
    Eigen::Vector3d gyr_0;
    
    sensor_msgs::ImuConstPtr curr_acc;
    std::vector<sensor_msgs::ImuConstPtr> gyro_buf;
        
    std::mutex mutex_player;
    std::vector<Eigen::Vector3d> global_keypose;
    std::unordered_map<int, Eigen::Vector3d> local_landmarks;
    std::unordered_map<int, Eigen::Vector3d> global_landmarks;
}

std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>,sensor_msgs::PointCloudConstPtr>> getMeasurements(ODOMETRY const* odo)
{
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>,sensor_msgs::PointCloudConstPtr>> measurements;
    
    while(true)
    {
        if (odo->imu_buf.empty() || odo->feature_buf.empty())
            return measurements;
        if (!(odo->imu_buf.back()->header.stamp > odo->feature_buf.front()->header.stamp))
        {
            console::print_warn("WARN:wait for imu,only should happen at the beginning.\n");
            odo->sum_of_wait++;
            return measurements;
        }

        if (!(odo->imu_buf.front()->header.stamp < odo->feature_buf.front()->header.stamp))
        {
            console::print_warn("WARN:throw img,only should happen at the beginning.\n");
            odo->feature_buf.pop();
            continue;
        }
        sensor_msgs::PointCloudConstPtr img_msg = odo->feature_buf.front();
        odo->feature_buf.pop();

        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (odo->imu_buf.front()->header.stamp <= img_msg->header.stamp)
        {
            IMUs.emplace_back(odo->imu_buf.front());
            odo->imu_buf.pop();
        }
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

- (id)init
{
    self->pub_count = 1;
    self->sum_of_wait = 0;
    self->first_image_flag = true;
    
    self->current_time = -1;
    self->latest_time = -1;
    self->curr_acc = nullptr;
    
    return self;
}

- (void)process
{
    while(true)
    {
        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
        std::unique_lock<std::mutex> buf_lock(mutex_buf);
        con.wait(buf_lock,[&]{
            return (measurements = getMeasurements(self)).size() != 0;
        });
        buf_lock.unlock();
        
        for (const auto& measurement : measurements)
        {
            for (const auto& imu_msg : measurement.first)
                [self track_imu:imu_msg->header.stamp.toSec() ax:imu_msg->linear_acceleration.x ay:imu_msg->linear_acceleration.y az:imu_msg->linear_acceleration.z gx:imu_msg->angular_velocity.x gy:imu_msg->angular_velocity.y gz:imu_msg->angular_velocity.z];
            
            const auto& img_msg = measurement.second;
            std::map<int,std::vector<std::pair<int,Eigen::Vector3d>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); ++i)
            {
                int feature_id = img_msg->channels[0].values[i];
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                assert(z == 1);
                image[feature_id].emplace_back(0, Eigen::Vector3d(x, y, z));
            }
            
            //estimator.processImage(image, img_msg->header);
            
            if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            {
                std::unique_lock<std::mutex> player_lock(mutex_player);
                global_keypose.emplace_back(estimator.Ps[WINDOW_SIZE]);
                local_landmarks = std::move(estimator.local_cloud);
                global_landmarks = estimator.global_cloud;
                player_lock.unlock();
            }
        }
        
        mutex_buf.lock();
        mutex_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
        {
            [self latestPose_callback];
        }
        mutex_state.unlock();
        mutex_buf.unlock();
    }
}

- (void)processBuffer:(CMSampleBufferRef)buffer
{
    CVImageBufferRef pixelBuffer = CMSampleBufferGetImageBuffer(buffer);
    CVPixelBufferLockBaseAddress(pixelBuffer, 0);
    
    int w = (int)CVPixelBufferGetWidth(pixelBuffer);
    int h = (int)CVPixelBufferGetHeight(pixelBuffer);
    int pixelPerRow = (int)CVPixelBufferGetBytesPerRow(pixelBuffer);
    unsigned char *baseAddress = (unsigned char*)CVPixelBufferGetBaseAddress(pixelBuffer);
    
    cv::Mat raw_image = cv::Mat(cv::Size(w, h), CV_8UC4, baseAddress, pixelPerRow);
    
    CVPixelBufferUnlockBaseAddress(pixelBuffer, 0);
    
    cv::Mat rgb_image;
    cv::cvtColor(raw_image, self->cvimage, cv::COLOR_BGRA2GRAY);
    cv::cvtColor(raw_image, rgb_image, cv::COLOR_BGRA2RGB);
    
    cv::flip(rgb_image,rgb_image,0);
    self->uiimage = MatToUIImage(rgb_image);
}

- (void)gyr_callback:(double)t x:(double)x y:(double)y z:(double)z
{
    sensor_msgs::ImuPtr gyr_msg(new sensor_msgs::Imu);
    gyr_msg->header.stamp = ros::Time(t);
    gyr_msg->angular_velocity.x = x;
    gyr_msg->angular_velocity.y = y;
    gyr_msg->angular_velocity.z = z;
    
    if (self->gyro_buf.empty()) {
        self->gyro_buf.emplace_back(gyr_msg);
        self->gyro_buf.emplace_back(gyr_msg);
    } else {
        self->gyro_buf[0] = self->gyro_buf[1];
        self->gyro_buf[1] = gyr_msg;
    }
    
    if (!self->curr_acc)
        return;
    
    sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu);
    if (self->curr_acc->header.stamp >= self->gyro_buf[0]->header.stamp &&
        self->curr_acc->header.stamp <= self->gyro_buf[1]->header.stamp)
    {
        imu_msg->header.stamp = self->curr_acc->header.stamp;
        imu_msg->linear_acceleration = self->curr_acc->linear_acceleration;
        
        imu_msg->angular_velocity.x = gyro_buf[0]->angular_velocity.x +(self->curr_acc->header.stamp.toSec() - gyro_buf[0]->header.stamp.toSec())*(gyro_buf[1]->angular_velocity.x-gyro_buf[0]->angular_velocity.x)/(gyro_buf[1]->header.stamp.toSec()-gyro_buf[0]->header.stamp.toSec());
        imu_msg->angular_velocity.y = gyro_buf[0]->angular_velocity.y +(self->curr_acc->header.stamp.toSec() - gyro_buf[0]->header.stamp.toSec())*(gyro_buf[1]->angular_velocity.y-gyro_buf[0]->angular_velocity.y)/(gyro_buf[1]->header.stamp.toSec()-gyro_buf[0]->header.stamp.toSec());
        imu_msg->angular_velocity.z = gyro_buf[0]->angular_velocity.z +(self->curr_acc->header.stamp.toSec() - gyro_buf[0]->header.stamp.toSec())*(gyro_buf[1]->angular_velocity.z-gyro_buf[0]->angular_velocity.z)/(gyro_buf[1]->header.stamp.toSec()-gyro_buf[0]->header.stamp.toSec());
        
        [self imu_callback:imu_msg->header.stamp.toSec() ax:imu_msg->linear_acceleration.x ay:imu_msg->linear_acceleration.y az:imu_msg->linear_acceleration.z gx:imu_msg->angular_velocity.x gy:imu_msg->angular_velocity.y gz:imu_msg->angular_velocity.z];
        
        std::cout << "imu_msg: " << std::setprecision(8)
                  << imu_msg->header.stamp.toSec() << ", "
                  << imu_msg->linear_acceleration.x << ", "
                  << imu_msg->linear_acceleration.y << ", "
                  << imu_msg->linear_acceleration.z << ", "
                  << imu_msg->angular_velocity.x << ", "
                  << imu_msg->angular_velocity.y << ", "
                  << imu_msg->angular_velocity.z << ", " << std::endl;
    }
}

- (void)acc_callback:(double)t x:(double)x y:(double)y z:(double)z
{
    sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu);
    imu_msg->header.stamp = ros::Time(t);
    imu_msg->linear_acceleration.x = x;
    imu_msg->linear_acceleration.y = y;
    imu_msg->linear_acceleration.z = z;
    self->curr_acc = imu_msg;
}

- (void)imu_callback:(double)t ax:(double)ax ay:(double)ay az:(double)az gx:(double)gx gy:(double)gy gz:(double)gz
{
    sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu);
    imu_msg->header.stamp = ros::Time(t);
    imu_msg->angular_velocity.x = ax;
    imu_msg->angular_velocity.y = ay;
    imu_msg->angular_velocity.z = az;
    imu_msg->linear_acceleration.x = ax;
    imu_msg->linear_acceleration.y = ay;
    imu_msg->linear_acceleration.z = az;
    
    std::unique_lock<std::mutex> buf_lock(mutex_buf);
    imu_buf.push(imu_msg);
    buf_lock.unlock();
}

- (void)latestPose_callback
{
    self->latest_time = self->current_time;
    self->tmp_P = self->estimator.Ps[WINDOW_SIZE];
    self->tmp_Q = self->estimator.Rs[WINDOW_SIZE];
    self->tmp_V = self->estimator.Vs[WINDOW_SIZE];
    self->tmp_Ba = self->estimator.Bas[WINDOW_SIZE];
    self->tmp_Bg = self->estimator.Bgs[WINDOW_SIZE];

    self->acc_0 = self->estimator.acc_0;
    self->gyr_0 = self->estimator.gyr_0;

    auto& tmp_imu_buf = self->imu_buf;
    for (; !tmp_imu_buf.empty(); tmp_imu_buf.pop()) {
        auto& imu_msg = tmp_imu_buf.front();
        [self track_imu:imu_msg->header.stamp.toSec() ax:imu_msg->linear_acceleration.x ay:imu_msg->linear_acceleration.y az:imu_msg->linear_acceleration.z gx:imu_msg->angular_velocity.x gy:imu_msg->angular_velocity.y gz:imu_msg->angular_velocity.z];
    }
}
        

- (void)predict:(double)t ax:(double)ax ay:(double)ay az:(double)az gx:(double)gx gy:(double)gy gz:(double)gz
{
    double dt = t - latest_time;
    latest_time = t;

    Eigen::Vector3d linear_acceleration{ax,ay,az};
    Eigen::Vector3d angular_velocity{gx,gy,gz};

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


- (void)track_imu:(double)t ax:(double)ax ay:(double)ay az:(double)az gx:(double)gx gy:(double)gy gz:(double)gz
{
    if (self->current_time < 0) {
        self->current_time = t;
    }
    double dt = t - self->current_time;
    self->current_time = t;
    
    double ba[]{0.0,0.0,0.0};
    double bg[]{0.0,0.0,0.0};
    
    double dx = ax - ba[0];
    double dy = ay - ba[1];
    double dz = az - ba[2];
    
    double rx = gx - bg[0];
    double ry = gy - bg[1];
    double rz = gz - bg[2];
    
    estimator.processIMU(dt, Eigen::Vector3d(dx,dy,dz), Eigen::Vector3d(rx,ry,rz));
}

- (void)image_callback:(double)t buffer:(CMSampleBufferRef)buffer
{
    [self processBuffer:buffer];
    
    if (self->first_image_flag)
    {
        self->first_image_flag = false;
        self->first_image_time = t;
    }
    
    if (std::round(1.0*pub_count/(t-self->first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        if (std::abs(1.0*pub_count/(t-self->first_image_time)) - FREQ < 0.01*FREQ)
        {
            self->first_image_time = t;
            self->pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;
    
    tracker.readImage(self->cvimage);
    
    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        completed |= tracker.updateID(i);
        if (!completed)
            break;
    }
    
    if (PUB_THIS_FRAME)
    {
        self->pub_count++;
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_point;
        sensor_msgs::ChannelFloat32 u_of_point;
        sensor_msgs::ChannelFloat32 v_of_point;

        feature_points->header.stamp = ros::Time(t);
        feature_points->header.frame_id = "world";

        const auto &un_pts = tracker.undistortedPoints();
        const auto &cur_pts = tracker.cur_pts;
        const auto &ids = tracker.ids;
        for (int j = 0; j < int(ids.size()); ++j) {
            int p_id = ids[j];
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
        
        std::unique_lock<std::mutex> buf_lock(mutex_buf);
        self->feature_buf.push(feature_points);
        buf_lock.unlock();
        
        con.notify_one();
    }
}

- (SCNVector3)getCameraPosition {
    Eigen::Vector3d pos = estimator.Ps[WINDOW_SIZE];
    return SCNVector3Make(pos(0), pos(1), pos(2));
}

- (SCNQuaternion)getCameraRotation {
    Eigen::Quaterniond rot = Eigen::Quaterniond(estimator.Rs[WINDOW_SIZE]);
    return SCNVector4Make(rot.x(), rot.y(), rot.z(), rot.w());
}

- (SysState)getSystemState {
    return SysState();
}

- (UIImage*)getUIImage {
    return self->uiimage;
}


@end
