# ESP32 Gimbal - Conservative PID Starting Values
## For GM5802 (Tilt) & GM2208 (Pan) Motors

## 🎯 Ready-to-Use Code

### Copy-Paste vào ESP32:

```cpp
// ============================================================
// CONSERVATIVE PID - SAFE STARTING VALUES
// Motors: GM5802 (Tilt), GM2208 (Pan)
// ============================================================

// Tilt Motor (GM5802 - 11 pole pairs)
PID_Controller pidTilt(
    8.0f,   // Kp - Moderate response
    0.0f,   // Ki - Disabled (add later if needed)
    0.3f,   // Kd - High damping to prevent overshoot
    90.0f * DEG_TO_RAD  // Start at balanced position
);

// Pan Motor (GM2208 - 7 pole pairs)
PID_Controller pidPan(
    6.0f,   // Kp - Moderate response
    0.0f,   // Ki - Disabled
    0.3f,   // Kd - High damping
    0.0f    // Start at center
);
```

### Motor Configuration (Already in your code):

```cpp
// Tilt (Motor 2 - GM5802)
motor2.voltage_limit = 7.0f;      // OK
motor2.velocity_limit = 2.0f;     // Increase from 1.5 → 2.0
motor2.current_limit = 1;         // OK

// Pan (Motor 1 - GM2208)  
motor1.voltage_limit = 7.0f;      // OK
motor1.velocity_limit = 2.0f;     // Increase from 1.5 → 2.0
motor1.current_limit = 1;         // OK
```

**IMPORTANT:** Tăng `velocity_limit` lên **2.0** để PID có thể điều khiển nhanh hơn!

## 🧪 Testing Procedure

### Phase 1: Static Test (No Tracking)

```cpp
void loop() {
    motor1.loopFOC();
    motor2.loopFOC();
    motor3.loopFOC();
    
    // Test 1: Hold position
    target_tilt = 90.0f * DEG_TO_RAD;  // Balanced
    target_pan = 0.0f;                  // Center
    
    float smooth_tilt = pidTilt.compute(target_tilt);
    float smooth_pan = pidPan.compute(target_pan);
    
    motor2.move(smooth_tilt);
    motor1.move(smooth_pan);
    motor3.move(90.0f * DEG_TO_RAD);
}
```

**Expected:** Gimbal should hold steady, no oscillation.

**If oscillates:**
- Reduce Kp by 20%
- Increase Kd to 0.4

### Phase 2: Small Step Test

```cpp
// Test 2: Small movements
unsigned long now = millis();
if (now % 3000 < 1500) {
    target_tilt = 95.0f * DEG_TO_RAD;  // +5° up
    target_pan = 10.0f * DEG_TO_RAD;   // +10° right
} else {
    target_tilt = 85.0f * DEG_TO_RAD;  // -5° down
    target_pan = -10.0f * DEG_TO_RAD;  // -10° left
}
```

**Expected:** Smooth movement, no overshoot > 2°

**If overshoots:**
- Increase Kd to 0.4-0.5
- Reduce Kp by 10%

**If too slow:**
- Increase Kp by 20%

### Phase 3: Tracking Test

Connect to Android app and test real tracking.

**Expected:** Smooth following, no jitter

## 🔧 Tuning Guide

### Symptom → Solution

| Problem | Kp | Ki | Kd | Notes |
|---------|----|----|----|----|
| **Oscillates** | ↓ 20% | - | ↑ to 0.4 | Too aggressive |
| **Overshoots** | - | - | ↑ to 0.5 | Need more damping |
| **Sluggish** | ↑ 20% | - | - | Too conservative |
| **Jittery** | ↓ 10% | - | ↑ to 0.4 | Reduce noise |
| **Steady-state error** | - | Add 0.01 | - | Doesn't reach target |
| **Drifts slowly** | - | Add 0.01 | - | Position creep |

### Tuning Steps

**Step 1: Find Kp**
```cpp
// Start: Kp=8.0, Ki=0.0, Kd=0.3
// Increase Kp until oscillation appears
// Then reduce by 30%
```

**Step 2: Tune Kd**
```cpp
// Increase Kd to eliminate overshoot
// Target: < 2° overshoot
// Typical range: 0.3 - 0.6
```

**Step 3: Add Ki (if needed)**
```cpp
// Only if steady-state error exists
// Start with Ki=0.01
// Increase slowly (max 0.05)
```

## ⚠️ Safety Limits

```cpp
// In PID_Controller::compute()
// Already has these protections:

integral = constrain(integral, -5.0f, 5.0f);  // Anti-windup
pid_output = constrain(pid_output, -50.0f, 50.0f);  // Limit output

// Mechanical limits (already in your code):
new_tilt_deg = constrain(new_tilt_deg, 45.0f, 135.0f);
```

## 📊 Expected Performance

**Good PID:**
- Tilt error: < 3°
- Pan error: < 5°
- No visible oscillation
- Smooth movement
- Settling time: < 0.5s

**Excellent PID:**
- Tilt error: < 1°
- Pan error: < 2°
- No overshoot
- Very smooth
- Settling time: < 0.3s

## 🐛 Common Issues

### Issue 1: Motor doesn't move

**Check:**
```cpp
Serial.printf("Tilt: %.2f -> %.2f\n", pidTilt.current_val, target_tilt);
```

**Solutions:**
- Increase `voltage_limit`
- Increase `velocity_limit` to 2.5
- Check motor wiring

### Issue 2: Jittery movement

**Solutions:**
```cpp
// Increase EMA smoothing in PID (if you add it)
// Or increase Kd
Kd = 0.5f;
```

### Issue 3: Different behavior Tilt vs Pan

**This is NORMAL!**
- GM5802 (Tilt): Heavier load → may need higher Kp
- GM2208 (Pan): Lighter load → may need lower Kp

Tune them independently!

## 📱 Integration with Android App

**App sends:**
```
{{[ax]:15;[ay]:-8}}
```

**ESP32 receives:**
```cpp
// Your onDataReceived() already handles this
angle_x = 15.0  // Pan
angle_y = -8.0  // Tilt

target_tilt = 90.0 + (-8.0) = 82.0°
target_pan = 15.0°
```

**PID smooths:**
```cpp
smooth_tilt = pidTilt.compute(82.0 * DEG_TO_RAD);
smooth_pan = pidPan.compute(15.0 * DEG_TO_RAD);
```

**Motors move smoothly!**

## ✅ Quick Start Checklist

- [ ] Update PID values (Kp=8/6, Kd=0.3)
- [ ] Increase velocity_limit to 2.0
- [ ] Upload to ESP32
- [ ] Test Phase 1 (static hold)
- [ ] Test Phase 2 (small steps)
- [ ] Tune if needed
- [ ] Test Phase 3 (tracking)
- [ ] Fine-tune based on real performance

## 🎯 Final Notes

**These values are CONSERVATIVE** - designed to be safe and stable.

After testing, you can:
- Increase Kp for faster response
- Adjust Kd for smoothness
- Add Ki if needed

**Don't expect perfection immediately!** Hardware tuning takes iteration.

Good luck! 🚀
