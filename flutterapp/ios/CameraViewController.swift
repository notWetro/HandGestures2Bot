//
//  CameraViewController.swift
//  Runner
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

    // MARK: - CAMERA SYSTEM
    private let captureSession = AVCaptureSession()
    private var previewLayer: AVCaptureVideoPreviewLayer!

    // MARK: - HAND OVERLAY
    let overlay = HandOverlayView()

    // MARK: - GESTURE ENGINE
    let recognizer = GestureStore.shared         // Shared gesture engine
    private var lastGesture: String = "UNKNOWN"  // Prevents spamming Flutter

    // MARK: - FLUTTER CHANNEL
    var methodChannel: FlutterMethodChannel?

    // MARK: - MEDIAPIPE HAND TRACKING
    private var handService: HandLandmarkerService!

    override func viewDidLoad() {
        super.viewDidLoad()

        setupHandLandmarker()
        setupCamera()
        setupOverlay()
    }

    // MARK: -------------------------
    // MARK: MediaPipe Hand Landmarker
    // MARK: -------------------------

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

    // MARK: --------
    // MARK: CAMERA
    // MARK: --------

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

        // START CAMERA ON BACKGROUND THREAD (Apple requirement)
        DispatchQueue.global(qos: .userInitiated).async {
            self.captureSession.startRunning()
        }
    }

    // MARK: ----------
    // MARK: OVERLAY UI
    // MARK: ----------

    private func setupOverlay() {
        overlay.frame = view.bounds
        overlay.backgroundColor = .clear
        overlay.autoresizingMask = [.flexibleWidth, .flexibleHeight]
        overlay.attachPreviewLayer(previewLayer)
        view.addSubview(overlay)
    }

    // MARK: ---------------------
    // MARK: CAMERA → MEDIAPIPE
    // MARK: ---------------------

    func captureOutput(_ output: AVCaptureOutput,
                       didOutput sampleBuffer: CMSampleBuffer,
                       from connection: AVCaptureConnection) {

        handService.detectAsync(sampleBuffer: sampleBuffer, orientation: .up)
    }

    // MARK: ------------------------
    // MARK: MP RESULT → LANDMARKS
    // MARK: ------------------------

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

            // 1️⃣ Draw Overlay
            self.overlay.update(with: mpResult)

            // 2️⃣ Recognize Gesture
            let gesture = self.recognizer.recognize(landmarks: landmarks)

            // 3️⃣ Send to Flutter only if changed
            if gesture != self.lastGesture {
                self.lastGesture = gesture
                print("iOS Gesture Detected → \(gesture)")
                self.methodChannel?.invokeMethod("onGesture", arguments: gesture)
            }
        }
    }

    // MARK: ----------------------
    // MARK: GESTURE SAVE LOGIC
    // MARK: ----------------------

    func saveGesture(name: String, landmarks: [NormalizedLandmark]) {

        recognizer.saveTemplate(name: name, landmarks: landmarks)

        print("iOS: Gesture '\(name)' saved successfully.")

        // Notify Flutter UI
        methodChannel?.invokeMethod("onSaveSuccess", arguments: name)
    }
}

