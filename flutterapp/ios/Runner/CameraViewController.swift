//
//  CameraViewController.swift
//  Runner
//
//  Created by Okan Demirbilek on 02.12.25.
//

import UIKit
import AVFoundation
import MediaPipeTasksVision

class CameraViewController: UIViewController,
                            AVCaptureVideoDataOutputSampleBufferDelegate,
                            HandLandmarkerServiceLiveStreamDelegate {

    private let captureSession = AVCaptureSession()
    private var previewLayer: AVCaptureVideoPreviewLayer!
    private let overlay = HandOverlayView()
    private var handService: HandLandmarkerService!

    override func viewDidLoad() {
        super.viewDidLoad()

        setupHandLandmarker()
        setupCamera()
        setupOverlay()
    }

    // MARK: - Setup Hand Landmarker
    private func setupHandLandmarker() {
        let modelPath = Bundle.main.path(
            forResource: "hand_landmarker",
            ofType: "task"
        )

        handService = HandLandmarkerService(
            modelPath: modelPath,
            numHands: 1,
            minHandDetectionConfidence: 0.5,
            minHandPresenceConfidence: 0.5,
            minTrackingConfidence: 0.5,
            liveStreamDelegate: self
        )
    }

    // MARK: - Setup Camera
    private func setupCamera() {
        captureSession.sessionPreset = .high

        guard let camera = AVCaptureDevice.default(
            .builtInWideAngleCamera,
            for: .video,
            position: .back
        ),
        let input = try? AVCaptureDeviceInput(device: camera) else {
            return
        }

        captureSession.addInput(input)

        let output = AVCaptureVideoDataOutput()
        output.videoSettings = [
            kCVPixelBufferPixelFormatTypeKey as String : kCVPixelFormatType_32BGRA
        ]
        output.setSampleBufferDelegate(self, queue: DispatchQueue(label: "camera.stream"))
        captureSession.addOutput(output)

        previewLayer = AVCaptureVideoPreviewLayer(session: captureSession)
        previewLayer.frame = view.bounds
        previewLayer.videoGravity = .resizeAspectFill

        view.layer.addSublayer(previewLayer)
        captureSession.startRunning()
    }

    // MARK: - Setup Overlay
    private func setupOverlay() {
        overlay.frame = view.bounds
        overlay.backgroundColor = .clear
        overlay.attachPreviewLayer(previewLayer)
        view.addSubview(overlay)
    }

    // MARK: - Capture Output → MediaPipe Input
    func captureOutput(
        _ output: AVCaptureOutput,
        didOutput sampleBuffer: CMSampleBuffer,
        from connection: AVCaptureConnection
    ) {
        handService.detectAsync(
            sampleBuffer: sampleBuffer,
            orientation: .up
        )
    }

    // MARK: - MediaPipe Output → Overlay Update
    func handLandmarkerService(
        _ service: HandLandmarkerService,
        didFinishDetection result: ResultBundle?,
        error: Error?
    ) {
        guard let mpResult = result?.handLandmarkerResults.first as? HandLandmarkerResult else {
            return
        }

        DispatchQueue.main.async {
            self.overlay.update(with: mpResult)
        }
    }
}
