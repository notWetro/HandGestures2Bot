//
//  CameraViewController.swift
//  
//
//  Created by Okan Demirbilek on 02.12.25.
//

import UIKit
import AVFoundation
import MediaPipeTasksVision
import Flutter

class CameraViewController: UIViewController,
                            AVCaptureVideoDataOutputSampleBufferDelegate,
                            HandLandmarkerServiceLiveStreamDelegate {

    // CAMERA SYSTEM
    private let captureSession = AVCaptureSession()
    private var previewLayer: AVCaptureVideoPreviewLayer!

    // HAND OVERLAY
    let overlay = HandOverlayView()

    // GESTURE ENGINE
    let recognizer = GestureStore.shared         
    private var lastGesture: String = "UNKNOWN"  

    // FLUTTER CHANNEL
    var methodChannel: FlutterMethodChannel?

    // MEDIAPIPE HAND TRACKING
    private var handService: HandLandmarkerService!

    override func viewDidLoad() {
        super.viewDidLoad()

        setupHandLandmarker()
        setupCamera()
        setupOverlay()
    }

    // MediaPipe Hand Landmarker

    private func setupHandLandmarker() {
        guard let modelPath = Bundle.main.path(forResource: "hand_landmarker", ofType: "task")
        else {
            print("ERROR: Could not load hand_landmarker.task file!")
            return
        }

        handService = HandLandmarkerService(
            modelPath: modelPath,
            numHands: 1,
            minHandDetectionConfidence: 0.5,
            minHandPresenceConfidence: 0.5,
            minTrackingConfidence: 0.5,
            liveStreamDelegate: self
        )
    }

    // CAMERA

    private func setupCamera() {
        captureSession.sessionPreset = .high

        guard let camera = AVCaptureDevice.default(.builtInWideAngleCamera, for: .video, position: .back),
              let input = try? AVCaptureDeviceInput(device: camera)
        else {
            print("ERROR: Unable to open iPhone camera.")
            return
        }

        captureSession.addInput(input)

        let output = AVCaptureVideoDataOutput()
        output.videoSettings = [
            kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA
        ]

        // MediaPipe processing queue
        output.setSampleBufferDelegate(self, queue: DispatchQueue(label: "camera.stream"))
        captureSession.addOutput(output)

        // Preview layer
        previewLayer = AVCaptureVideoPreviewLayer(session: captureSession)
        previewLayer.frame = view.bounds
        previewLayer.videoGravity = .resizeAspectFill
        view.layer.addSublayer(previewLayer)

        // START CAMERA ON BACKGROUND THREAD
        DispatchQueue.global(qos: .userInitiated).async {
            self.captureSession.startRunning()
        }
    }

    // OVERLAY UI

    private func setupOverlay() {
        overlay.frame = view.bounds
        overlay.backgroundColor = .clear
        overlay.autoresizingMask = [.flexibleWidth, .flexibleHeight]
        overlay.attachPreviewLayer(previewLayer)
        view.addSubview(overlay)
    }

    // CAMERA -> MEDIAPIPE

    func captureOutput(_ output: AVCaptureOutput,
                       didOutput sampleBuffer: CMSampleBuffer,
                       from connection: AVCaptureConnection) {

        handService.detectAsync(sampleBuffer: sampleBuffer, orientation: .up)
    }

    // MP RESULT -> LANDMARKS

    func handLandmarkerService(
        _ service: HandLandmarkerService,
        didFinishDetection result: ResultBundle?,
        error: Error?
    ) {
        guard
            let res = result,
            let mpResult = res.handLandmarkerResults.first as? HandLandmarkerResult,
            let landmarks = mpResult.landmarks.first
        else { return }

        DispatchQueue.main.async {

            self.overlay.update(with: mpResult)

            let gesture = self.recognizer.recognize(landmarks: landmarks)

            if gesture != self.lastGesture {
                self.lastGesture = gesture
                print("iOS Gesture Detected → \(gesture)")
                self.methodChannel?.invokeMethod("onGesture", arguments: gesture)
            }
        }
    }

    // GESTURE SAVE LOGIC

    func saveGesture(name: String, landmarks: [NormalizedLandmark]) {

        recognizer.saveTemplate(name: name, landmarks: landmarks)

        print("iOS: Gesture '\(name)' saved successfully.")

        methodChannel?.invokeMethod("onSaveSuccess", arguments: name)
    }
}

