//
//  HandOverlayView.swift
//  Runner
//
//  Created by Okan Demirbilek on 02.12.25.
//

import UIKit
import AVFoundation
import MediaPipeTasksVision

class HandOverlayView: UIView {

    private var landmarks: [NormalizedLandmark] = []
    private var previewLayer: AVCaptureVideoPreviewLayer?

    func attachPreviewLayer(_ layer: AVCaptureVideoPreviewLayer) {
        self.previewLayer = layer
    }

    override func draw(_ rect: CGRect) {
        guard let previewLayer = previewLayer else { return }
        guard !landmarks.isEmpty else { return }
        guard let ctx = UIGraphicsGetCurrentContext() else { return }

        ctx.setLineWidth(3)
        ctx.setStrokeColor(UIColor.systemGreen.cgColor)
        ctx.setFillColor(UIColor.systemGreen.cgColor)

        let connections = [
            (0,1),(1,2),(2,3),(3,4),
            (0,5),(5,6),(6,7),(7,8),
            (5,9),(9,10),(10,11),(11,12),
            (9,13),(13,14),(14,15),(15,16),
            (13,17),(17,18),(18,19),(19,20),
            (0,17)
        ]

        var points: [CGPoint] = []

        for lm in landmarks {
            let normalized = CGPoint(x: CGFloat(lm.x), y: CGFloat(lm.y))
            let converted = previewLayer.layerPointConverted(fromCaptureDevicePoint: normalized)
            points.append(converted)
        }

        for (start, end) in connections {
            ctx.move(to: points[start])
            ctx.addLine(to: points[end])
        }
        ctx.strokePath()

        for p in points {
            ctx.fillEllipse(in: CGRect(x: p.x - 4, y: p.y - 4, width: 8, height: 8))
        }
    }

    func update(with result: HandLandmarkerResult) {
        self.landmarks = result.landmarks.first ?? []
        setNeedsDisplay()
    }
}
