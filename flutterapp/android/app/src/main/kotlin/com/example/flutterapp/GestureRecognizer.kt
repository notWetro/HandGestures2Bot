package com.example.flutterapp

import com.google.mediapipe.tasks.components.containers.NormalizedLandmark
import kotlin.math.abs

class GestureRecognizer {

    // Define the gestures you care about
    enum class Gesture {
        UNKNOWN,
        THUMBS_UP,
        POINTING_LEFT,
        POINTING_RIGHT,
        NUMBER_TWO,
        NUMBER_THREE,
        FIST,
        OPEN_PALM
    }

    fun recognize(landmarks: List<NormalizedLandmark>): Gesture {
        
        // 1. Helper variables
        val thumbTip = landmarks[4]
        val indexTip = landmarks[8]
        val indexMCP = landmarks[5] // Knuckle
        val middleTip = landmarks[12]
        val ringTip = landmarks[16]
        val pinkyTip = landmarks[20]
        val wrist = landmarks[0]

        // 2. Count how many fingers are "Up" (excluding thumb for now)
        val fingersUpCount = countFingersUp(landmarks)

        // --- LOGIC TREE ---

        // CHECK 1: THUMBS UP
        // Thumb tip is high (low Y), other fingers are down
        if (thumbTip.y() < indexMCP.y() && fingersUpCount == 0) {
            return Gesture.THUMBS_UP
        }

        // CHECK 2: POINTING LEFT (Assuming simple X-axis check)
        // Index tip is to the LEFT of the knuckle (and significantly far)
        if (indexTip.x() < indexMCP.x() && abs(indexTip.x() - indexMCP.x()) > 0.05) {
            // Ensure other fingers are folded to distinguish from just an open hand
            if (fingersUpCount <= 1) return Gesture.POINTING_LEFT
        }

        // CHECK 3: POINTING RIGHT
        // Index tip is to the RIGHT of the knuckle
        if (indexTip.x() > indexMCP.x() && abs(indexTip.x() - indexMCP.x()) > 0.05) {
             if (fingersUpCount <= 1) return Gesture.POINTING_RIGHT
        }

        // CHECK 4: NUMBERS (Based on finger count)
        if (fingersUpCount == 2) return Gesture.NUMBER_TWO
        if (fingersUpCount == 3) return Gesture.NUMBER_THREE
        if (fingersUpCount == 4 || fingersUpCount == 5) return Gesture.OPEN_PALM
        if (fingersUpCount == 0) return Gesture.FIST

        return Gesture.UNKNOWN
    }

    // Helper to count extended fingers (Index, Middle, Ring, Pinky)
    private fun countFingersUp(landmarks: List<NormalizedLandmark>): Int {
        var count = 0
        // Index (8) above knuckle (5)
        if (landmarks[8].y() < landmarks[5].y()) count++
        // Middle (12) above knuckle (9)
        if (landmarks[12].y() < landmarks[9].y()) count++
        // Ring (16) above knuckle (13)
        if (landmarks[16].y() < landmarks[13].y()) count++
        // Pinky (20) above knuckle (17)
        if (landmarks[20].y() < landmarks[17].y()) count++
        return count
    }
}