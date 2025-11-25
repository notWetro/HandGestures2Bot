//
//  MyCameraView.swift
//  Runner
//
//  Created by Okan Demirbilek on 24.11.25.
//

import UIKit
import AVFoundation
import MediaPipeTasksVision

class MyCameraView: UIView {

    private var previewLayer: AVCaptureVideoPreviewLayer!
    private let session = AVCaptureSession()
    private var handLandmarker: HandLandmarkerHelper!
    private let overlayView = OverlayView()

    override init(frame: CGRect) {
        super.init(frame: frame)

        setupCamera()
        setupOverlay()
        setupLandmarker()
    }

    required init?(coder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }

    private func setupCamera() {
        session.sessionPreset = .high

        guard let device = AVCaptureDevice.default(.builtInWideAngleCamera, for: .video, position: .back),
              let input = try? AVCaptureDeviceInput(device: device)
        else { return }

        session.addInput(input)

        let output = AVCaptureVideoDataOutput()
        output.setSampleBufferDelegate(self, queue: DispatchQueue(label: "cameraQueue"))
        output.alwaysDiscardsLateVideoFrames = true
        session.addOutput(output)

        previewLayer = AVCaptureVideoPreviewLayer(session: session)
        previewLayer.videoGravity = .resizeAspectFill
        layer.addSublayer(previewLayer)

        session.startRunning()
    }

    private func setupOverlay() {
        overlayView.backgroundColor = .clear
        addSubview(overlayView)
    }

    override func layoutSubviews() {
        super.layoutSubviews()
        previewLayer.frame = bounds
        overlayView.frame = bounds
    }

    private func setupLandmarker() {
        handLandmarker = HandLandmarkerHelper(onResults: { bundle in
            DispatchQueue.main.async {
                self.overlayView.setResults(bundle)
            }
        })
    }
}

extension MyCameraView: AVCaptureVideoDataOutputSampleBufferDelegate {
    func captureOutput(
        _ output: AVCaptureOutput,
        didOutput sampleBuffer: CMSampleBuffer,
        from connection: AVCaptureConnection
    ) {
        handLandmarker.process(sampleBuffer: sampleBuffer)
    }
}
