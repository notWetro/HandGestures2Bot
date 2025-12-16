import Foundation
import MediaPipeTasksVision
import AVFoundation

protocol HandLandmarkerServiceLiveStreamDelegate: AnyObject {
    func handLandmarkerService(
        _ service: HandLandmarkerService,
        didFinishDetection result: ResultBundle?,
        error: Error?
    )
}

class HandLandmarkerService: NSObject {

    private var handLandmarker: HandLandmarker?
    weak var liveStreamDelegate: HandLandmarkerServiceLiveStreamDelegate?

    private var modelPath: String
    private var numHands: Int
    private var minHandDetectionConfidence: Float
    private var minHandPresenceConfidence: Float
    private var minTrackingConfidence: Float

    init?(
        modelPath: String?,
        numHands: Int,
        minHandDetectionConfidence: Float,
        minHandPresenceConfidence: Float,
        minTrackingConfidence: Float,
        liveStreamDelegate: HandLandmarkerServiceLiveStreamDelegate?
    ) {
        guard let path = modelPath else { return nil }
        self.modelPath = path
        self.numHands = numHands
        self.minHandDetectionConfidence = minHandDetectionConfidence
        self.minHandPresenceConfidence = minHandPresenceConfidence
        self.minTrackingConfidence = minTrackingConfidence
        self.liveStreamDelegate = liveStreamDelegate

        super.init()
        setupHandLandmarker()
    }

    private func setupHandLandmarker() {
        let options = HandLandmarkerOptions()
        options.baseOptions.modelAssetPath = modelPath
        options.runningMode = .liveStream
        options.numHands = self.numHands
        options.minHandDetectionConfidence = self.minHandDetectionConfidence
        options.minHandPresenceConfidence = self.minHandPresenceConfidence
        options.minTrackingConfidence = self.minTrackingConfidence
        options.handLandmarkerLiveStreamDelegate = self

        handLandmarker = try? HandLandmarker(options: options)
    }

    func detectAsync(sampleBuffer: CMSampleBuffer, orientation: UIImage.Orientation) {
        let ts = Int(Date().timeIntervalSince1970 * 1000)
        guard let mpImg = try? MPImage(sampleBuffer: sampleBuffer, orientation: orientation) else { return }
        try? handLandmarker?.detectAsync(image: mpImg, timestampInMilliseconds: ts)
    }
}

extension HandLandmarkerService: HandLandmarkerLiveStreamDelegate {
    func handLandmarker(
        _ handLandmarker: HandLandmarker,
        didFinishDetection result: HandLandmarkerResult?,
        timestampInMilliseconds: Int,
        error: Error?
    ) {
        let bundle = ResultBundle(
            inferenceTime: 0,
            handLandmarkerResults: [result]
        )
        liveStreamDelegate?.handLandmarkerService(self, didFinishDetection: bundle, error: error)
    }
}

struct ResultBundle {
    let inferenceTime: Double
    let handLandmarkerResults: [HandLandmarkerResult?]
}

