//
//  ODOMETRY.h
//  vins
//
//  Created by 熊鑫鑫 on 2023/5/22.
//
#import <Foundation/Foundation.h>
#import <SceneKit/SceneKit.h>
#import <UIKit/UIKit.h>
#import <AVFoundation/AVFoundation.h>

@interface ODOMETRY : NSObject

typedef NS_ENUM(NSUInteger, SysState) {
    SYS_INITIALIZING = 0,
    SYS_TRACKING,
    SYS_CRASH,
    SYS_UNKNOWN
};

- (void)processBuffer:(CMSampleBufferRef)buffer;
- (void)trackGyroscope:(double)t x:(double)x y:(double)y z:(double)z;
- (void)trackAccelerometer:(double)t x:(double)x y:(double)y z:(double)z;
- (void)trackCamera:(double)t buffer:(CMSampleBufferRef)buffer;
- (SCNVector3)getCameraPosition;
- (SCNQuaternion)getCameraRotation;
- (SysState)getSystemState;
- (UIImage*)getUIImage;

@end
