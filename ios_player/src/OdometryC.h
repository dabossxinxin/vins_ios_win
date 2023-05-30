//
//  ODOMETRY.h
//  vins
//
//  Created by 熊鑫鑫 on 2023/5/22.
//

#ifndef ODOMETRYC_H
#define ODOMETRYC_H

#ifdef __OBJC__

#import <Foundation/Foundation.h>
#import <SceneKit/SceneKit.h>
#import <UIKit/UIKit.h>
#import <AVFoundation/AVFoundation.h>

#endif

@interface ODOMETRY : NSObject

typedef NS_ENUM(NSUInteger, SysState) {
    SYS_INITIALIZING = 0,
    SYS_TRACKING,
    SYS_CRASH,
    SYS_UNKNOWN
};
- (id)init;
- (void)process;
- (void)processBuffer:(CMSampleBufferRef)buffer;
- (void)latestPose_callback;
- (void)gyr_callback:(double)t x:(double)x y:(double)y z:(double)z;
- (void)acc_callback:(double)t x:(double)x y:(double)y z:(double)z;
- (void)imu_callback:(double)t ax:(double)ax ay:(double)ay az:(double)az gx:(double)gx gy:(double)gy gz:(double)gz;
- (void)track_imu:(double)t ax:(double)ax ay:(double)ay az:(double)az gx:(double)gx gy:(double)gy gz:(double)gz;
- (void)predict:(double)t ax:(double)ax ay:(double)ay az:(double)az gx:(double)gx gy:(double)gy gz:(double)gz;
- (void)image_callback:(double)t buffer:(CMSampleBufferRef)buffer;
- (SCNVector3)getCameraPosition;
- (SCNQuaternion)getCameraRotation;
- (SysState)getSystemState;
- (UIImage*)getUIImage;

@end

#endif
