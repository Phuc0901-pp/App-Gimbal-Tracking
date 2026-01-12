# 📁 ESP32 Gimbal - All Files

## 📋 File Organization

### 🔧 **HARDWARE CODE (Upload to ESP32)**

#### 1. **Main Gimbal Code** (Your existing code)
```
📄 main.ino (your current code with PID)
```
- Tracking control
- PID implementation
- Bluetooth communication
- Motor control (SimpleFOC)

#### 2. **Auto-Tuning Code** ⭐ RECOMMENDED
```
📄 esp32_auto_tuning_hardware.ino
```
**Purpose:** Automatically find optimal PID gains
**Usage:** 
- Upload this code temporarily
- Run auto-tuning (30 sec/motor)
- Copy PID values
- Return to main code

#### 3. **Capture Button Addition**
```
📄 esp32_capture_button.cpp
```
**Purpose:** GPIO19 button → send CAPTURE to app
**Usage:** Copy code snippets into your main.ino

---

### 📚 **DOCUMENTATION & GUIDES**

#### 4. **Conservative PID Guide** ⭐ START HERE
```
📄 esp32_conservative_pid_guide.md
```
**Purpose:** Safe starting PID values
**Contains:**
- Ready-to-use PID: Kp=8/6, Kd=0.3
- 3-phase testing procedure
- Troubleshooting guide

#### 5. **Hardware Tuning Guide**
```
📄 esp32_hardware_tuning_guide.md
```
**Purpose:** How to use auto-tuning code
**Contains:**
- Step-by-step instructions
- Ziegler-Nichols explanation
- Expected results

---

### 🧪 **MATLAB SIMULATION (Optional)**

#### 6. **Basic Simulation**
```
📄 esp32_gimbal_pid_tuning.m
```
**Purpose:** Manual PID testing in MATLAB
**Note:** Model not accurate, use hardware tuning instead

#### 7. **Auto-Optimization**
```
📄 esp32_gimbal_auto_optimization.m
```
**Purpose:** MATLAB auto-optimization
**Note:** Results not reliable, use hardware method

#### 8. **Tuning Guide (MATLAB)**
```
📄 esp32_gimbal_tuning_guide.md
```
**Purpose:** How to use MATLAB simulation
**Note:** Hardware tuning is better

---

## 🚀 Quick Start Guide

### Option A: Conservative Start (Fastest)

1. **Open:** `esp32_conservative_pid_guide.md`
2. **Copy PID values:**
   ```cpp
   PID_Controller pidTilt(8.0f, 0.0f, 0.3f, 90.0f * DEG_TO_RAD);
   PID_Controller pidPan(6.0f, 0.0f, 0.3f, 0.0f);
   ```
3. **Paste** into your main.ino
4. **Upload** and test

### Option B: Auto-Tuning (Best Results)

1. **Upload:** `esp32_auto_tuning_hardware.ino`
2. **Serial Monitor:** 115200 baud
3. **Send:** `T` (tune Tilt), `P` (tune Pan)
4. **Copy** optimal PID values
5. **Paste** into your main.ino
6. **Upload** main code

### Option C: Add Capture Button

1. **Open:** `esp32_capture_button.cpp`
2. **Copy** code snippets
3. **Paste** into your main.ino:
   - Setup section
   - Loop section
4. **Wire:** GPIO19 → Button → GND
5. **Upload** and test

---

## 📊 File Priority

**Must Read:**
1. ✅ `esp32_conservative_pid_guide.md` - Start here
2. ✅ `esp32_hardware_tuning_guide.md` - If using auto-tune

**Code to Upload:**
1. ✅ `esp32_auto_tuning_hardware.ino` - For PID tuning
2. ✅ Your main.ino (with PID values)

**Optional:**
- `esp32_capture_button.cpp` - If adding button
- MATLAB files - Only if you want simulation

---

## 🎯 Recommended Workflow

```
1. Read: esp32_conservative_pid_guide.md
2. Test: Conservative PID values (Kp=8/6, Kd=0.3)
3. If not satisfied:
   → Upload: esp32_auto_tuning_hardware.ino
   → Run auto-tuning
   → Copy optimal PID
4. Add: Capture button (optional)
5. Done! ✅
```

---

## 📝 File Descriptions

| File | Type | Purpose | Priority |
|------|------|---------|----------|
| `esp32_conservative_pid_guide.md` | Guide | Starting PID values | ⭐⭐⭐ |
| `esp32_auto_tuning_hardware.ino` | Code | Auto-find PID | ⭐⭐⭐ |
| `esp32_hardware_tuning_guide.md` | Guide | Auto-tune instructions | ⭐⭐⭐ |
| `esp32_capture_button.cpp` | Code | GPIO button | ⭐⭐ |
| `esp32_gimbal_pid_tuning.m` | MATLAB | Manual simulation | ⭐ |
| `esp32_gimbal_auto_optimization.m` | MATLAB | Auto simulation | ⭐ |
| `esp32_gimbal_tuning_guide.md` | Guide | MATLAB guide | ⭐ |

---

## 💾 Where to Save

**Recommended folder structure:**
```
📁 ESP32_Gimbal/
├── 📁 Code/
│   ├── main.ino (your tracking code)
│   ├── esp32_auto_tuning_hardware.ino
│   └── esp32_capture_button.cpp
├── 📁 Guides/
│   ├── esp32_conservative_pid_guide.md
│   ├── esp32_hardware_tuning_guide.md
│   └── esp32_gimbal_tuning_guide.md
└── 📁 MATLAB/ (optional)
    ├── esp32_gimbal_pid_tuning.m
    └── esp32_gimbal_auto_optimization.m
```

---

## ✅ Checklist

**Before Testing:**
- [ ] Read conservative PID guide
- [ ] Understand 3-phase testing
- [ ] Know troubleshooting steps

**Hardware Setup:**
- [ ] GM5802 (Tilt) wired correctly
- [ ] GM2208 (Pan) wired correctly
- [ ] Power supply 12V
- [ ] Bluetooth module connected

**Software:**
- [ ] PID values updated
- [ ] Code uploaded
- [ ] Serial monitor ready (115200)

**Testing:**
- [ ] Phase 1: Static hold
- [ ] Phase 2: Small steps
- [ ] Phase 3: Real tracking

---

## 🆘 Support

**If issues:**
1. Check relevant guide
2. Review troubleshooting section
3. Try conservative values first
4. Then auto-tune if needed

**Common Issues:**
- Oscillation → Reduce Kp, increase Kd
- Sluggish → Increase Kp
- Overshoot → Increase Kd

---

**All files ready in artifacts folder!**
**Start with conservative guide → Test → Auto-tune if needed**

Good luck! 🚀
