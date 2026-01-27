package com.google.mediapipe.examples.poselandmarker

import android.util.Log
import okhttp3.*
import okhttp3.MediaType.Companion.toMediaType
import okhttp3.RequestBody.Companion.toRequestBody
import org.json.JSONObject
import java.io.IOException

/**
 * Push metrics to laptop server via HTTP
 */
class MetricsPusher(private val laptopIP: String) {
    private val client = OkHttpClient()
    private val mediaType = "application/json".toMediaType()
    
    fun push() {
        if (laptopIP.isEmpty()) return
        
        Thread {
            try {
                val metrics = MetricsCollector.getSnapshot()
                val json = JSONObject().apply {
                    put("fps", metrics.fps.toDouble())
                    put("inferenceTime", metrics.inferenceTime)
                    put("frameDropRate", metrics.frameDropRate.toDouble())
                    put("panAngle", metrics.panAngle.toDouble())
                    put("tiltAngle", metrics.tiltAngle.toDouble())
                    put("rollAngle", metrics.rollAngle.toDouble())  // [NEW] Roll from ESP32
                    put("bleConnected", metrics.bleConnected)
                    put("isTracking", metrics.isTracking)
                    put("bleLatency", metrics.bleLatency)
                    put("lastReceivedData", metrics.lastReceivedData)
                    put("lastReceivedDataTime", metrics.lastReceivedDataTime)
                    put("errorCount", metrics.errorCount)
                    put("lastError", metrics.lastError)
                    put("uptime", metrics.uptime)
                    put("timestamp", System.currentTimeMillis())
                }
                
                val requestBody = json.toString().toRequestBody(mediaType)
                val request = Request.Builder()
                    .url("http://$laptopIP:5000/push")
                    .post(requestBody)
                    .build()
                    
                client.newCall(request).enqueue(object : Callback {
                    override fun onFailure(call: Call, e: IOException) {
                        Log.e("MetricsPusher", "Push failed: ${e.message}")
                    }
                    
                    override fun onResponse(call: Call, response: Response) {
                        response.close()
                    }
                })
            } catch (e: Exception) {
                Log.e("MetricsPusher", "Error creating metrics: ${e.message}")
            }
        }.start()
    }
    
    fun pushErrors() {
        if (laptopIP.isEmpty()) return
        
        Thread {
            try {
                val errors = MetricsCollector.getRecentErrors(20)
                val json = JSONObject().apply {
                    put("errors", org.json.JSONArray(errors.map {
                        JSONObject().apply {
                            put("code", it.code)
                            put("message", it.message)
                            put("timestamp", it.timestamp)
                        }
                    }))
                }
                
                val requestBody = json.toString().toRequestBody(mediaType)
                val request = Request.Builder()
                    .url("http://$laptopIP:5000/push_errors")
                    .post(requestBody)
                    .build()
                    
                client.newCall(request).execute().close()
            } catch (e: Exception) {
                Log.e("MetricsPusher", "Error pushing errors: ${e.message}")
            }
        }.start()
    }
}
