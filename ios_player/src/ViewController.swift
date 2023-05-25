//
//  ViewController.swift
//  vins
//
//  Created by 熊鑫鑫 on 2023/5/17.
//

import UIKit
import SceneKit

class ViewController: UIViewController, SCNSceneRendererDelegate {

    var camera: Camera? = nil
    var motion: Motion? = nil
    var slamer: Odometry? = nil
    
    var scene: SCNScene!
    var sceneCameraNode: SCNNode!
    
    @IBOutlet weak var previewView: UIView!
    @IBOutlet weak var sceneView: SCNView!
    
    @IBOutlet weak var loggerText: UILabel!
    @IBOutlet weak var textUI: UIStackView!
    
    @IBOutlet weak var message: UITextView!
    
    @IBOutlet weak var button_start: UIButton!
    @IBOutlet weak var button_reset: UIButton!
    @IBOutlet weak var button_debug: UIButton!
    @IBOutlet weak var button_switch: UISegmentedControl!
    
    override func viewDidLoad() {
        super.viewDidLoad()
                
        setupScene()
        message.isEditable = false
        
        self.sceneView.delegate = self
        self.sceneView.isPlaying = true
        self.sceneView.loops = true
        
        NotificationCenter.default.addObserver(self, selector: #selector(applicationDidBecomeActive), name: UIApplication.didBecomeActiveNotification, object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(applicationWillResignActive), name: UIApplication.willResignActiveNotification, object: nil)

        textUI.clipsToBounds = true
        textUI.layer.cornerRadius = 10
        textUI.layer.maskedCorners = [.layerMaxXMinYCorner, .layerMinXMinYCorner]
        loggerText.textAlignment = .center
        
        message.layer.cornerRadius = 10
        message.layer.maskedCorners = [.layerMinXMaxYCorner, .layerMaxXMaxYCorner]
        
        button_start.backgroundColor = UIColor.systemRed
        button_debug.backgroundColor = UIColor.systemRed
        button_reset.backgroundColor = UIColor.systemRed
        button_start.layer.cornerRadius = 10
        button_debug.layer.cornerRadius = 10
        button_reset.layer.cornerRadius = 10
    }
    
    @objc func applicationDidBecomeActive() {
        startCamera()
    }

    @objc func applicationWillResignActive() {
        stopCamera()
    }
    
    func startCamera() {
        guard let camera = Camera() else {
            NSLog("Cannot access camera.")
            return
        }
        
        camera.setFps(20)
        camera.setFocus(0.835)
        
        guard let motion = Motion() else {
            NSLog("Cannot access motion.")
            return
        }
        
        self.slamer = Odometry()
        self.camera = camera
        self.motion = motion
        self.motion?.delegate = slamer
        self.camera?.delegate = slamer
        
        let imageView: UIImageView = UIImageView()
        imageView.frame = self.previewView.bounds
        self.previewView.addSubview(imageView)
        self.slamer?.imageView = imageView
    }
    
    func stopCamera() {
        self.camera = nil
        self.camera = nil
    }
    
    func setupScene() {
        scene = SCNScene()
        scene.background.contentsTransform = SCNMatrix4MakeRotation(Float.pi/2, 0.0, 0.0, 1.0)
        
        self.sceneView.scene = scene
        self.sceneView.autoenablesDefaultLighting = true
        self.sceneView.allowsCameraControl = false
        self.sceneView.preferredFramesPerSecond = 30
        
        sceneCameraNode = SCNNode()
        sceneCameraNode.camera = SCNCamera()
        sceneCameraNode.camera?.zNear = 0.0001
        sceneCameraNode.camera?.zFar = 20
        sceneCameraNode.position = SCNVector3(0, 0, 0)
        sceneCameraNode.orientation = SCNQuaternion(0, 0, 0, 1)
        
        self.sceneView.pointOfView = sceneCameraNode
        self.sceneView.scene?.rootNode.addChildNode(sceneCameraNode)
        self.sceneView.backgroundColor = UIColor.systemGray6
    }
    
    @IBAction func clickStartbutton(_ sender: UIButton) {
        
    }
    
    @IBAction func clickResetButton(_ sender: UIButton) {
        
    }
    
    @IBAction func clickDebugButton(_ sender: UIButton) {
        
    }
    
    @IBAction func clickSegmentedControl(_ sender: UISegmentedControl) {
        
    }
    
}

