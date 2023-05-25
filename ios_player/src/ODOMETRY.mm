//
//  ODOMETRY.m
//  vins
//
//  Created by 熊鑫鑫 on 2023/5/22.
//

#import <UIKit/UIKit.h>
#import <SceneKit/SceneKit.h>
#import <Foundation/Foundation.h>
#import "ODOMETRY.h"

#import <opencv2/opencv.hpp>
#import <opencv2/imgcodecs/ios.h>

@implementation ODOMETRY {
    UIImage *uiimage;
    cv::Mat cvimage;
}

- (void)processBuffer:(CMSampleBufferRef)buffer {
    CVImageBufferRef pixelBuffer = CMSampleBufferGetImageBuffer(buffer);
    CVPixelBufferLockBaseAddress(pixelBuffer, 0);
    
    int w = (int)CVPixelBufferGetWidth(pixelBuffer);
    int h = (int)CVPixelBufferGetHeight(pixelBuffer);
    int pixelPerRow = (int)CVPixelBufferGetBytesPerRow(pixelBuffer);
    unsigned char *baseAddress = (unsigned char*)CVPixelBufferGetBaseAddress(pixelBuffer);
    
    cv::Mat raw_image = cv::Mat(h, w, CV_8UC4, baseAddress, pixelPerRow);
    
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

- (void)trackCamera:(double)t buffer:(CMSampleBufferRef)buffer {
    
}

- (SCNVector3)getCameraPosition {
    return SCNVector3Make(0, 0, 0);
}

- (SCNQuaternion)getCameraRotation {
    return SCNVector4Make(0, 0, 0, 1);
}

- (SysState)getSystemState {
    return SysState();
}

- (UIImage*)getUIImage {
    return self->uiimage;
}


@end
