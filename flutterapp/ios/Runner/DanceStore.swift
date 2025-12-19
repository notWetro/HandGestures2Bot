//
//  DanceStore.swift
//  Runner
//
//  Created for dance moves storage
//

import Foundation

class DanceStore {
    static let shared = DanceStore()
    
    private let storageURL: URL
    
    init() {
        let dir = FileManager.default.urls(for: .applicationSupportDirectory, in: .userDomainMask).first!
        self.storageURL = dir.appendingPathComponent("dance_moves.json")
    }
    
    func saveDanceMoves(jsonString: String) {
        do {
            let data = jsonString.data(using: .utf8)!
            try FileManager.default.createDirectory(
                at: storageURL.deletingLastPathComponent(),
                withIntermediateDirectories: true
            )
            try data.write(to: storageURL)
            print("DanceStore: Saved dance moves successfully")
        } catch {
            print("DanceStore ERROR saving: \(error)")
        }
    }
    
    func loadDanceMoves() -> String? {
        do {
            let data = try Data(contentsOf: storageURL)
            let jsonString = String(data: data, encoding: .utf8)
            print("DanceStore: Loaded dance moves successfully")
            return jsonString
        } catch {
            print("DanceStore: No saved dance moves yet")
            return nil
        }
    }
}
