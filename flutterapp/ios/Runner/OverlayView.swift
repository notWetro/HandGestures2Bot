//
//  OverlayView.swift
//  Runner
//
//  Created by Okan Demirbilek on 24.11.25.
//

import UIKit
import MediaPipeTasksVision

class OverlayView: UIView {

    private var result: HandLandmarkerResult?

    func setResults(_ bundle: HandLandmarkerHelper.ResultBundle) {
        result = bundle.results.first
        setNeedsDisplay()
    }

    override func draw(_ rect: CGRect) {
        guard let result else { return }
        guard let landmarks = result.landmarks.first else { return }

        let context = UIGraphicsGetCurrentContext()!
        context.setLineWidth(4)
        context.setStrokeColor(UIColor.red.cgColor)
        context.setFillColor(UIColor.green.cgColor)

        // draw points
        for p in landmarks {
            let x = CGFloat(p.x) * rect.width
            let y = CGFloat(p.y) * rect.height
            context.fillEllipse(in: CGRect(x: x, y: y, width: 10, height: 10))
        }

        // draw connections (same pairs as Android)
        for conn in HandLandmarker.handConnections {
            let start = landmarks[Int(conn.start)]
            let end = landmarks[Int(conn.end)]

            context.move(to: CGPoint(x: CGFloat(start.x) * rect.width,
                                     y: CGFloat(start.y) * rect.height))
            context.addLine(to: CGPoint(x: CGFloat(end.x) * rect.width,
                                        y: CGFloat(end.y) * rect.height))
            context.strokePath()
        }
    }
}
