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

@implementation ODOMETRY {
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
    
    std::mutex mutex_player;
    std::vector<Eigen::Vector3d> global_keypose;
    std::unordered_map<int, Eigen::Vector3d> local_landmarks;
    std::unordered_map<int, Eigen::Vector3d> global_landmarks;
}

- (id)init {
    self->pub_count = 1;
    self->sum_of_wait = 0;
    self->first_image_flag = true;
    
    self->current_time = -1;
    self->latest_time = -1;
    
    return self;
}

- (void)processBuffer:(CMSampleBufferRef)buffer {
    CVImageBufferRef pixelBuffer = CMSampleBufferGetImageBuffer(buffer);
    CVPixelBufferLockBaseAddress(pixelBuffer, 0);
    
    int w = (int)CVPixelBufferGetWidth(pixelBuffer);
    int h = (int)CVPixelBufferGetHeight(pixelBuffer);
    int pixelPerRow = (int)CVPixelBufferGetBytesPerRow(pixelBuffer);
    unsigned char *baseAddress = (unsigned char*)CVPixelBufferGetBaseAddress(pixelBuffer);
    
    cv::Mat raw_image = cv::Mat(cv::Size(w, h), CV_8UC4, baseAddress, pixelPerRow);
    
    CVPixelBufferUnlockBaseAddress(pixelBuffer, 0);
    
    cv::Mat rgb_image;
    cv::cvtColor(raw_image, cvimage, cv::COLOR_BGRA2GRAY);
    cv::cvtColor(raw_image, rgb_image, cv::COLOR_BGRA2RGB);
    
    cv::flip(rgb_image,rgb_image,0);
    uiimage = MatToUIImage(rgb_image);
}

- (void)trackGyroscope:(double)t x:(double)x y:(double)y z:(double)z {
    
}

- (void)trackAccelerometer:(double)t x:(double)x y:(double)y z:(double)z {
    
}

- (void)imu_callback:(double)t ax:(double)ax ay:(double)ay az:(double)az gx:(double)gx gy:(double)gy gz:(double)gz {
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

- (void)image_callback:(double)t buffer:(CMSampleBufferRef)buffer {
    [self processBuffer:buffer];
    
    if (self->first_image_flag) {
        self->first_image_flag = false;
        self->first_image_time = t;
    }
    
    if (std::round(1.0*pub_count/(t-self->first_image_time)) <= FREQ) {
        PUB_THIS_FRAME = true;
        if (std::abs(1.0*pub_count/(t-self->first_image_time)) - FREQ < 0.01*FREQ) {
            self->first_image_time = t;
            self->pub_count = 0;
        }
    } else {
        PUB_THIS_FRAME = false;
    }
    
    tracker.readImage(self->cvimage);
    
    for (unsigned int i = 0;; i++) {
        bool completed = false;
        completed |= tracker.updateID(i);
        if (!completed)
            break;
    }
    
    if (PUB_THIS_FRAME) {
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
