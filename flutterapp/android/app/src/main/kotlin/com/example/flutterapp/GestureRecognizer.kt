package com.example.flutterapp

import android.content.Context
import android.content.SharedPreferences
import android.util.Log
import com.google.gson.Gson
import com.google.gson.reflect.TypeToken
import com.google.mediapipe.tasks.components.containers.NormalizedLandmark
import kotlin.math.sqrt

// 1. Data Model
data class GestureTemplate(
    val name: String,
    val fingerprint: List<Float>
)

class GestureRecognizer(private val context: Context) {

    private val templates = mutableListOf<GestureTemplate>()
    private val gson = Gson()
    private val PREFS_NAME = "GesturePrefs"
    private val KEY_GESTURES = "SavedGestures"

    // LOWER = Stricter match. 
    // 15.0 is a good starting point for normalized coordinates.
    private val MATCH_THRESHOLD = 15.0f 

    init {
        loadFromStorage()
    }

    // --- MAIN API ---

    fun saveTemplate(name: String, landmarks: List<NormalizedLandmark>) {
        val fingerprint = createFingerprint(landmarks)
        
        // Overwrite if name exists
        templates.removeAll { it.name == name }
        templates.add(GestureTemplate(name, fingerprint))
        
        Log.d("GestureRecognizer", "Saved '$name'. Total templates: ${templates.size}")
        saveToStorage()
    }

    fun recognize(landmarks: List<NormalizedLandmark>): String {
        if (templates.isEmpty()) return "UNKNOWN"

        val currentFingerprint = createFingerprint(landmarks)
        
        var bestMatchName = "UNKNOWN"
        var bestScore = Float.MAX_VALUE

        for (template in templates) {
            val score = calculateDistance(currentFingerprint, template.fingerprint)
            
            if (score < bestScore) {
                bestScore = score
                bestMatchName = template.name
            }
        }

        return if (bestScore < MATCH_THRESHOLD) bestMatchName else "UNKNOWN"
    }

    // --- STORAGE LOGIC ---

    private fun saveToStorage() {
        val prefs = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
        val json = gson.toJson(templates)
        prefs.edit().putString(KEY_GESTURES, json).apply()
    }

    private fun loadFromStorage() {
        val prefs = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
        val json = prefs.getString(KEY_GESTURES, null)
        if (json != null) {
            val type = object : TypeToken<MutableList<GestureTemplate>>() {}.type
            val savedList: MutableList<GestureTemplate> = gson.fromJson(json, type)
            templates.clear()
            templates.addAll(savedList)
        }
    }

    // --- MATH & NORMALIZATION ---

    private fun createFingerprint(landmarks: List<NormalizedLandmark>): List<Float> {
        val fingerprint = mutableListOf<Float>()
        
        // 1. Use Wrist (Index 0) as Origin
        val wrist = landmarks[0]

        // 2. Scale by hand size (Wrist to Middle Finger Tip)
        val middleTip = landmarks[12]
        val scale = getDistance(wrist, middleTip)

        // 3. Normalize all points
        for (landmark in landmarks) {
            fingerprint.add((landmark.x() - wrist.x()) / scale)
            fingerprint.add((landmark.y() - wrist.y()) / scale)
            fingerprint.add((landmark.z() - wrist.z()) / scale)
        }
        return fingerprint
    }

    private fun calculateDistance(listA: List<Float>, listB: List<Float>): Float {
        var sum = 0.0f
        for (i in listA.indices) {
            val diff = listA[i] - listB[i]
            sum += diff * diff
        }
        return sqrt(sum)
    }

    private fun getDistance(p1: NormalizedLandmark, p2: NormalizedLandmark): Float {
        val dx = p1.x() - p2.x()
        val dy = p1.y() - p2.y()
        val dz = p1.z() - p2.z()
        return sqrt(dx*dx + dy*dy + dz*dz)
    }

    fun getSavedGestureNames(): List<String> {
        return templates.map { it.name }
    }
}