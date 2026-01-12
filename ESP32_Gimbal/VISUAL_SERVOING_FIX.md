# 🔄 Chuyển Đổi Sang Visual Servoing (Fix Lỗi "Đi Luôn")

## 🔴 Nguyên Nhân "Đi Luôn":
Hệ thống cũ cố gắng "ước lượng vị trí" (`currentPanAngle`) bằng cách cộng dồn vận tốc (Tích phân).
- Sai số tích lũy khiến gimbal "nghĩ" nó đang ở sai vị trí.
- Code cố gắng quay về "0 ảo" thay vì quay về đối tượng thực.
- Kết quả: **Run away (Càng chạy càng sai)**.

## ✅ Giải Pháp Mới: Direct Visual Servoing
Bỏ hoàn toàn việc ước lượng vị trí. Dùng **Góc Lệch Từ Camera** làm tín hiệu điều khiển trực tiếp.

### Logic:
1. **Camera thấy lệch -4°** (Target = -4) -> Đó chính là **LỖI**.
2. **Velocity = Kp * Target**.
3. **Kp = 0.2** (Nhỏ, mượt).
4. **Deadzone = 2.0°** (Dừng khi gần đúng).

### Kết Quả Mong Đợi:
- **Không còn Drift**: Vì không tích phân vận tốc nữa.
- **Tự động dừng**: Khi target về ~0 (nhìn thẳng đối tượng).
- **Phản ứng nhanh hơn**: Không bị trễ do bộ lọc vị trí.

## 📋 Hướng Dẫn Test:
1. **Upload Code** (Ctrl + Alt + U).
2. **Mode 2 -> Start Tracking**.
3. Gimbal sẽ di chuyển **trực tiếp** về phía đối tượng.
4. Nếu gimbal quay **ngược chiều** (xa đối tượng hơn) -> Báo tôi để đảo dấu `Kp` (đổi `0.2` thành `-0.2`). 
   *(Theo phân tích log cũ thì dấu dương là đúng)*.
