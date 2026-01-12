# ✅ Fix: Gimbal Không Gập Xuống Khi Vào Tracking Mode

## Vấn đề:
Khi chuyển từ MANUAL → TRACKING mode, gimbal bị "gập xuống" về vị trí 0° thay vì giữ vị trí hiện tại.

## Nguyên nhân:
Code cũ reset `currentPanAngle = 0` và `currentTiltAngle = 0` trong CAPTURE_ZERO.

## Giải pháp:
**Khởi tạo từ góc target đầu tiên** thay vì reset về 0.

### Thay đổi:

**TRƯỚC:**
```cpp
case CAPTURE_ZERO: {
    currentPanAngle = 0.0f;      // ← Reset về 0
    currentTiltAngle = 0.0f;     // ← Gimbal gập xuống!
}
```

**SAU:**
```cpp
case CAPTURE_ZERO: {
    // KHÔNG reset về 0 - giữ nguyên
    // Sẽ khởi tạo ở BALANCE khi nhận target đầu tiên
}

case BALANCE: {
    static bool initialized = false;
    if (!initialized && trackingDataReceived) {
        currentPanAngle = targetPanAngle;    // ← Bắt đầu từ target
        currentTiltAngle = targetTiltAngle;
        initialized = true;
    }
}
```

## Kết quả:
✅ Vào TRACKING mode → Gimbal giữ nguyên vị trí  
✅ Nhận target đầu tiên → Bắt đầu track từ vị trí đó  
✅ Chuyển động mượt, không gập  

## Test:
1. MANUAL mode → Di chuyển gimbal lên vị trí bất kỳ
2. Nhấn BTN2 2 lần → Vào TRACKING mode
3. **Gimbal phải GIỮ NGUYÊN vị trí**, không gập xuống!
4. Start tracking → Bắt đầu follow từ vị trí hiện tại
