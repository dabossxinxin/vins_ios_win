//
//  Odometry.swift
//  vins
//
//  Created by 熊鑫鑫 on 2023/5/21.
//

import Foundation
import AVFoundation
import SceneKit
import UIKit

class Odometry : CameraDelegate, MotionDelegate {
    
    let slamer: ODOMETRY
    weak var imageView: UIImageView? = nil
    
    var position: SCNVector3
    var rotation: SCNQuaternion
    
    init() {
        slamer = ODOMETRY()
        position = SCNVector3Make(0, 0, 0)
        rotation = SCNVector4Make(0, 0, 0, 1)
    }
    
    func cameraDidOutput(timestamp: CMTime, sampleBuffer: CMSampleBuffer) {
        NSLog("camera timestamp: %f", timestamp.seconds)
        slamer.processBuffer(sampleBuffer)
        self.imageView?.image = slamer.getUIImage()
        NSLog("camera width: %f, height: %f", slamer.getUIImage().size.width, slamer.getUIImage().size.height)
    }
    
    func motionDidGyroscopeUpdate(timestamp: Double, rotationRateX: Double, rotationRateY: Double, rotationRateZ: Double) {
        slamer.gyr_callback(timestamp, x: rotationRateX, y: rotationRateY, z: rotationRateZ)
    }
    
    func motionDidAccelerometerUpdate(timestamp: Double, accelerationX: Double, accelerationY: Double, accelerationZ: Double) {
        slamer.acc_callback(timestamp, x: accelerationX, y: accelerationY, z: accelerationZ)
    }
}

