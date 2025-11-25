//
//  HandLandmarkerHelper.swift
//  Runner
//
//  Created by Okan Demirbilek on 24.11.25.
//

import MediaPipeTasksVision
import AVFoundation

class HandLandmarkerHelper {

    struct ResultBundle {
        let results: [HandLandmarkerResult]
        let width: Int
        let height: Int
    }

    private let landmarker: HandLandmarker
    private let onResults: (ResultBundle) -> Void

    init(onResults: @escaping (ResultBundle) -> Void) {

        self.onResults = onResults

        let options = HandLandmarkerOptions()
        options.numHands = 1
        options.minHandDetectionConfidence = 0.5
        options.minTrackingConfidence = 0.5
        options.runningMode = .liveStream

        landmarker = try! HandLandmarker(options: options)
    }

    func process(sampleBuffer: CMSampleBuffer) {

        guard let image = try? MPImage(sampleBuffer: sampleBuffer) else { return }

        try? landmarker.detectAsync(
            mpImage: image,
            timestamp: Date().timeIntervalSince1970 * 1000,
            completion: { result, error in
                if let result {
                    let bundle = ResultBundle(
                        results: [result],
                        width: image.width,
                        height: image.height
                    )
                    self.onResults(bundle)
                }
            }
        )
    }
}
