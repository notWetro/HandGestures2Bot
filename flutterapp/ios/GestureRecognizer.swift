//
//  GestureRecognizer.swift
//  
//
//  Created by Okan Demirbilek on 14.12.25.
//

import Foundation
import MediaPipeTasksVision

struct GestureTemplate: Codable {
    let name: String
    let fingerprint: [Float]
}

class GestureRecognizer {

    private var templates: [GestureTemplate] = []
    private let storageURL: URL

    private let MATCH_THRESHOLD: Float = 15.0

    init() {
        // Storagepath
        let dir = FileManager.default.urls(for: .applicationSupportDirectory, in: .userDomainMask).first!
        self.storageURL = dir.appendingPathComponent("gestures.json")
        loadFromStorage()
        if templates.isEmpty {
            print("iOS: No saved gestures yet → returning empty list.")
        }

    }

    // MAIN API

    func saveTemplate(name: String, landmarks: [NormalizedLandmark]) {
        let fp = createFingerprint(from: landmarks)

        templates.removeAll { $0.name == name }
        templates.append(GestureTemplate(name: name, fingerprint: fp))

        saveToStorage()
        print("GestureRecognizer.swift: Saved template '\(name)'")
    }

    func recognize(landmarks: [NormalizedLandmark]) -> String {
        guard !templates.isEmpty else { return "UNKNOWN" }

        let current = createFingerprint(from: landmarks)

        var bestName = "UNKNOWN"
        var bestScore = Float.greatestFiniteMagnitude

        for template in templates {
            let score = calculateDistance(a: current, b: template.fingerprint)
            if score < bestScore {
                bestScore = score
                bestName = template.name
            }
        }

        return bestScore < MATCH_THRESHOLD ? bestName : "UNKNOWN"
    }

    func getSavedNames() -> [String] {
        return templates.map { $0.name }
    }

    // STORAGE

    private func saveToStorage() {
        do {
            let encoder = JSONEncoder()
            let data = try encoder.encode(templates)
            try FileManager.default.createDirectory(
                at: storageURL.deletingLastPathComponent(),
                withIntermediateDirectories: true
            )
            try data.write(to: storageURL)
        } catch {
            print("GestureRecognizer.swift ERROR saving: \(error)")
        }
    }

    private func loadFromStorage() {
        do {
            let data = try Data(contentsOf: storageURL)
            let loaded = try JSONDecoder().decode([GestureTemplate].self, from: data)
            templates = loaded
            print("Loaded \(templates.count) gesture templates")
        } catch {
            print("GestureRecognizer.swift: No saved gestures yet")
        }
    }

    // FINGERPRINTING

    private func createFingerprint(from landmarks: [NormalizedLandmark]) -> [Float] {
        guard landmarks.count >= 21 else { return [] }

        var result: [Float] = []

        let wrist = landmarks[0]
        let middleTip = landmarks[12]

        let scale = distance(wrist, middleTip)

        for lm in landmarks {
            result.append((lm.x - wrist.x) / scale)
            result.append((lm.y - wrist.y) / scale)
            result.append((lm.z - wrist.z) / scale)
        }
        return result
    }

    private func distance(_ a: NormalizedLandmark, _ b: NormalizedLandmark) -> Float {
        let dx = a.x - b.x
        let dy = a.y - b.y
        let dz = a.z - b.z
        return sqrt(dx*dx + dy*dy + dz*dz)
    }

    private func calculateDistance(a: [Float], b: [Float]) -> Float {
        var sum: Float = 0
        for i in 0 ..< a.count {
            let diff = a[i] - b[i]
            sum += diff * diff
        }
        return sqrt(sum)
    }
}
