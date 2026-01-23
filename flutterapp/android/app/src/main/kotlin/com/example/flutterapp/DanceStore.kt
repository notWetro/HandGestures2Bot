package com.example.flutterapp

import android.content.Context
import java.io.File
import android.util.Log

class DanceStore private constructor(private val context: Context) {

    private val storageFile: File by lazy {
        File(context.filesDir, "dance_moves.json")
    }

    companion object {
        @Volatile
        private var INSTANCE: DanceStore? = null

        fun getInstance(context: Context): DanceStore {
            return INSTANCE ?: synchronized(this) {
                INSTANCE ?: DanceStore(context.applicationContext).also {
                    INSTANCE = it
                }
            }
        }
    }

    fun saveDanceMoves(jsonString: String) {
        try {
            storageFile.writeText(jsonString)
            println("DanceStore: Saved dance moves successfully")
        } catch (e: Exception) {
            println("DanceStore ERROR saving: ${e.message}")
        }
    }

    fun loadDanceMoves(): String? {
        return try {
            if (!storageFile.exists()) return null
            val json = storageFile.readText()
            println("DanceStore: Loaded dance moves successfully")
            json
        } catch (e: Exception) {
            println("DanceStore: No saved dance moves yet")
            null
        }
    }
}
