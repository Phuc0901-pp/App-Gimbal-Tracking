package com.google.mediapipe.examples.poselandmarker

import org.junit.Test
import org.junit.Assert.*
import org.junit.Before

/**
 * Unit tests for PIDController.
 * 
 * Tests the PID control algorithm implementation including:
 * - Zero error handling
 * - Positive and negative errors
 * - Integral windup prevention
 * - Reset functionality
 * - Tuning parameter updates
 */
class PIDControllerTest {
    
    private lateinit var pid: PIDController
    
    @Before
    fun setup() {
        // Initialize with default values
        pid = PIDController(
            kp = Constants.PID.DEFAULT_KP,
            ki = Constants.PID.DEFAULT_KI,
            kd = Constants.PID.DEFAULT_KD
        )
    }
    
    @Test
    fun `test zero error returns zero output`() {
        // First call initializes, should return 0
        val output1 = pid.update(0f)
        assertEquals(0f, output1, 0.01f)
        
        // Subsequent calls with zero error should also return 0
        Thread.sleep(10) // Ensure dt > 0
        val output2 = pid.update(0f)
        assertEquals(0f, output2, 0.01f)
    }
    
    @Test
    fun `test positive error produces positive output`() {
        pid.update(0f) // Initialize
        Thread.sleep(10)
        
        val error = 10f
        val output = pid.update(error)
        
        // With positive Kp and positive error, output should be positive
        assertTrue("Output should be positive for positive error", output > 0)
    }
    
    @Test
    fun `test negative error produces negative output`() {
        pid.update(0f) // Initialize
        Thread.sleep(10)
        
        val error = -10f
        val output = pid.update(error)
        
        // With positive Kp and negative error, output should be negative
        assertTrue("Output should be negative for negative error", output < 0)
    }
    
    @Test
    fun `test output is clamped to min max range`() {
        // Create PID with high gains to force clamping
        val highGainPid = PIDController(
            kp = 100f,
            ki = 0f,
            kd = 0f,
            minOutput = -50f,
            maxOutput = 50f
        )
        
        highGainPid.update(0f) // Initialize
        Thread.sleep(10)
        
        // Large error should clamp to max
        val largeError = 1000f
        val output = highGainPid.update(largeError)
        assertEquals(50f, output, 0.01f)
        
        // Large negative error should clamp to min
        Thread.sleep(10)
        val largeNegativeError = -1000f
        val outputNeg = highGainPid.update(largeNegativeError)
        assertEquals(-50f, outputNeg, 0.01f)
    }
    
    @Test
    fun `test proportional term only`() {
        val pOnlyPid = PIDController(kp = 0.5f, ki = 0f, kd = 0f)
        pOnlyPid.update(0f) // Initialize
        Thread.sleep(10)
        
        val error = 20f
        val output = pOnlyPid.update(error)
        
        // Output should be approximately kp * error = 0.5 * 20 = 10
        assertEquals(10f, output, 0.5f)
    }
    
    @Test
    fun `test integral accumulation`() {
        val iOnlyPid = PIDController(kp = 0f, ki = 0.1f, kd = 0f)
        iOnlyPid.update(0f) // Initialize
        
        val error = 10f
        var lastOutput = 0f
        
        // Integral should accumulate over time
        for (i in 1..5) {
            Thread.sleep(100) // 100ms intervals
            val output = iOnlyPid.update(error)
            
            if (i > 1) {
                // Each iteration should increase the output (integral accumulation)
                assertTrue("Integral should accumulate: iteration $i", output > lastOutput)
            }
            lastOutput = output
        }
    }
    
    @Test
    fun `test derivative term responds to error change`() {
        val dOnlyPid = PIDController(kp = 0f, ki = 0f, kd = 1f)
        dOnlyPid.update(0f) // Initialize
        Thread.sleep(100)
        
        // Small error change
        dOnlyPid.update(5f)
        Thread.sleep(100)
        
        // Large error change should produce larger derivative output
        val output = dOnlyPid.update(15f) // Change of 10
        
        // Derivative should be non-zero when error changes
        assertNotEquals(0f, output, 0.01f)
    }
    
    @Test
    fun `test reset clears internal state`() {
        pid.update(0f) // Initialize
        Thread.sleep(10)
        
        // Build up some integral
        for (i in 1..5) {
            pid.update(10f)
            Thread.sleep(10)
        }
        
        // Reset should clear state
        pid.reset()
        
        // After reset, first call should return 0
        val output = pid.update(0f)
        assertEquals(0f, output, 0.01f)
    }
    
    @Test
    fun `test setTunings updates parameters`() {
        pid.update(0f) // Initialize
        Thread.sleep(10)
        
        val error = 10f
        val output1 = pid.update(error)
        
        // Change tunings
        pid.setTunings(kp = 1.0f, ki = 0f, kd = 0f)
        pid.reset() // Reset to clear state
        pid.update(0f) // Re-initialize
        Thread.sleep(10)
        
        val output2 = pid.update(error)
        
        // With higher Kp, output should be larger
        assertTrue("Higher Kp should produce larger output", output2 > output1)
    }
    
    @Test
    fun `test consistent output for consistent error`() {
        val pOnlyPid = PIDController(kp = 0.5f, ki = 0f, kd = 0f)
        pOnlyPid.update(0f) // Initialize
        Thread.sleep(10)
        
        val error = 20f
        val output1 = pOnlyPid.update(error)
        Thread.sleep(10)
        val output2 = pOnlyPid.update(error)
        
        // With only P term and same error, outputs should be similar
        assertEquals(output1, output2, 0.5f)
    }
}
