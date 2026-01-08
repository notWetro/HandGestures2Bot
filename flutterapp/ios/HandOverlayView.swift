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

    private var previewLayer: AVCaptureVideoPreviewLayer?
    private(set) var latestLandmarks: [NormalizedLandmark]? = nil

    func attachPreviewLayer(_ layer: AVCaptureVideoPreviewLayer) {
        self.previewLayer = layer
    }

    func update(with result: HandLandmarkerResult) {
        if let first = result.landmarks.first {
            latestLandmarks = first
        }
        setNeedsDisplay()
    }

    override func draw(_ rect: CGRect) {
        guard
            let preview = previewLayer,
            let lm = latestLandmarks,
            let ctx = UIGraphicsGetCurrentContext()
        else { return }

        let connections = [
            (0,1),(1,2),(2,3),(3,4),
            (0,5),(5,6),(6,7),(7,8),
            (5,9),(9,10),(10,11),(11,12),
            (9,13),(13,14),(14,15),(15,16),
            (13,17),(17,18),(18,19),(19,20),
            (0,17)
        ]

        let pts = lm.map { point -> CGPoint in
            preview.layerPointConverted(
                fromCaptureDevicePoint: CGPoint(x: CGFloat(point.x), y: CGFloat(point.y))
            )
        }

        ctx.setStrokeColor(UIColor.green.cgColor)
        ctx.setLineWidth(3)

        for (s, e) in connections {
            ctx.move(to: pts[s])
            ctx.addLine(to: pts[e])
        }
        ctx.strokePath()

        ctx.setFillColor(UIColor.green.cgColor)
        for p in pts {
            ctx.fillEllipse(in: CGRect(x: p.x - 4, y: p.y - 4, width: 8, height: 8))
        }
    }
}
