# 🚀 Quick Start - Web Server Monitoring

## Bước 1: Cài Flask

```bash
pip install flask flask-cors
```

## Bước 2: Chạy Server

```bash
cd C:\Pham_Phuc\Mobile_app\mediapipe-samples
python monitoring_server.py
```

**Output thành công:**
```
============================================================
📊 GIMBAL TRACKING MONITOR SERVER v2.0
============================================================
Server starting on http://0.0.0.0:5000

Database: monitoring_data\2026-01-09.db
Data directory: C:\...\monitoring_data

Access dashboard:
  - Local:  http://localhost:5000
  - Network: http://192.168.1.100:5000  ← Dùng IP này!

New Features:
  ✅ SQLite database persistence
  ✅ Auto-save every 1 second
  ✅ Historical data queries
  ✅ CSV export
  ✅ Session management

Android App Setup:
  1. Click '📊 MONITOR' button
  2. Enter laptop IP address
  3. Enable monitoring toggle
  4. Save
============================================================
```

## Bước 3: Lấy IP Laptop

**Windows:**
```bash
ipconfig
```
Tìm dòng:
```
IPv4 Address. . . . . . . . . . . : 192.168.1.100  ← Dùng IP này
```

## Bước 4: Cấu Hình App

1. Mở app Android
2. Nhấn nút **🖥️ Monitoring** (ở góc trên)
3. Nhập IP: `192.168.1.100` (thay bằng IP thật của bạn)
4. Bật **Enable Monitoring**
5. Click **Save**

## Bước 5: Xem Dashboard

Mở browser:
```
http://localhost:5000
```

**Bạn sẽ thấy:**
- 📊 Real-time FPS, latency
- 📈 Live chart pan/tilt angles
- 🔴 Error log
- 📶 BLE connection status

## ✅ Test Nhanh

Trong app:
1. Connect BLE → "ESP32_Tracker"  
2. Start tracking → Dashboard sẽ update real-time!  
3. Pan/Tilt chart sẽ hiển thị góc gimbal  

---

## Troubleshooting

**Lỗi: `ModuleNotFoundError: No module named 'flask'`**
```bash
pip install flask flask-cors
```

**Lỗi: Port 5000 đã dùng**
```bash
# Kill process trên port 5000
netstat -ano | findstr :5000
taskkill /PID <PID_NUMBER> /F
```

**App không gửi data lên server?**
- Check IP đúng chưa?
- Firewall có block port 5000?
- Laptop và phone cùng WiFi?

---

## Features Dashboard

### Real-time Metrics
- FPS (frames per second)
- Inference time
- Pan/Tilt angles (live chart)
- BLE latency

### Error Monitoring
- Error codes với timestamps
- Error messages
- Auto-refresh mỗi 100ms

### Data Export
- SQLite database tự động save
- Export CSV: `http://localhost:5000/export/csv`
- Historical queries

---

**Dashboard sẵn sàng!** 🎉
