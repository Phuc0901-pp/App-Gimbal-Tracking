# PHÂN TÍCH CHI TIẾT GIAO THỨC BLE VÀ PROTOCOL STACK

## 1. Cấu trúc BLE Protocol Stack trong Hệ thống

Hệ thống của bạn tuân thủ mô hình phân lớp chuẩn của Bluetooth Low Energy (BLE), nhưng có sự tùy biến mạnh mẽ ở tầng Ứng dụng (Application Layer) để đảm bảo tính tin cậy (reliability) cho việc truyền dữ liệu thời gian thực.

### Mô hình Phân lớp

| Tầng (Layer) | Chức năng Chuẩn | Thực thi trong Hệ thống |
| :--- | :--- | :--- |
| **Application** | Logic nghiệp vụ | **Custom Reliable Protocol:** Đóng gói `{seq:...,chk:...,data:...}` để kiểm soát lỗi. |
| **GATT** | Trao đổi dữ liệu | **Nordic UART Service (NUS):** Giả lập cổng Serial.<br>• UUID: `6e400001-b5a3-f393-e0a9-e50e24dcca9e` |
| **ATT** | Truy xuất thuộc tính | Dùng `Write Command` (Gửi nhanh) và `Notify` (Nhận liên tục). |
| **L2CAP** | Phân mảnh/Ghép kênh | MTU size = 512 bytes (tối ưu hóa payload). |
| **Link Layer** | Kết nối vật lý | Advertising & Scanning (2.4GHz). |

---

## 2. Kiến trúc Hệ thống và Luồng Dữ liệu (Architecture & Data Flow)

Hệ thống hoạt động theo mô hình **Central - Peripheral** (ngược với mô hình Client-Server của web, nhưng tương tự Master-Slave).

### 2.1. Vai trò (Roles)
*   **GATT Server (Peripheral - Thiết bị ngoại vi):** **ESP32**
    *   **Nhiệm vụ:** Quảng bá (Advertise) sự hiện diện của mình và chờ kết nối. Chứa dữ liệu (GATT Database) và thực thi lệnh.
    *   **Dịch vụ:** Chạy `Nordic UART Service`.
*   **GATT Client (Central - Thiết bị trung tâm):** **Android App**
    *   **Nhiệm vụ:** Quét (Scan), kết nối đến ESP32 và chủ động gửi lệnh/đọc dữ liệu.

### 2.2. Luồng Dữ liệu 1: Điều khiển (Android $\rightarrow$ ESP32)
Sử dụng đặc tính **RX Characteristic** (Write No Response). Đây là luồng dữ liệu quan trọng nhất để điều khiển Gimbal.

**Quy trình Gửi:**
1.  **Thu thập:** App lấy tọa độ lệch từ AI (`AngleCalculator`), hoặc lệnh người dùng (`CAPTURE`).
2.  **Đóng gói (Encoding):** Hàm `BleProtocol.wrapPacket()` tạo chuỗi `{seq:1,chk:XX,data:CMD}`.
3.  **Gửi BLE (Transmission):** Gọi `characteristic.setValue()` và `gatt.writeCharacteristic()`. Tầng L2CAP chia nhỏ nếu gói quá dài (ngắn hơn 512 bytes nên thường đi 1 gói).
4.  **Xử lý tại ESP32:**
    *   Callback `onWrite()` được kích hoạt.
    *   Hàm `BleProtocol::parsePacket()` tách vỏ, kiểm tra Checksum.
    *   Dữ liệu sạch được đưa vào `PID_Controller` để điều khiển động cơ.

### 2.3. Luồng Dữ liệu 2: Phản hồi (ESP32 $\rightarrow$ Android)
Sử dụng đặc tính **TX Characteristic** (Notify). Luồng này dùng để giám sát trạng thái hệ thống.

**Quy trình Gửi:**
1.  **Sự kiện:** ESP32 hoàn thành lệnh (ví dụ: đã chụp ảnh xong, hoặc báo lỗi).
2.  **Đóng gói:** Hàm `BleProtocol::wrapPacket()` tạo chuỗi phản hồi.
3.  **Gửi BLE:** Gọi `pTxCharacteristic->notify()`.
4.  **Nhận tại Android:**
    *   Callback `onCharacteristicChanged()` được kích hoạt.
    *   Hàm `BleProtocol.parsePacket()` kiểm tra tính toàn vẹn.
    *   Cập nhật giao diện (UI) hoặc ghi Log.

---

## 3. Phân tích Chuyên sâu: Cấu trúc Gói tin (Packet Structure)

Giao thức tùy biến (Custom Protocol) được thiết kế dạng **Text-based JSON-like**, ưu tiên tính dễ đọc (human-readable) và dễ gỡ lỗi (debug).

### 3.1. Định dạng Tổng quát
Mỗi gói tin là một chuỗi ký tự (String) tuân thủ định dạng nghiêm ngặt:

```text
{seq:SEQUENCE,chk:CHECKSUM,data:PAYLOAD}
```

### 3.2. Chi tiết từng trường (Field Breakdown)

| Trường (Field) | Kích thước (Bytes) | Kiểu dữ liệu | Mô tả |
| :--- | :--- | :--- | :--- |
| **Header** | 5 | String | Chuỗi cố định `{seq:` để nhận diện bắt đầu gói tin. |
| **Sequence** | 1 ~ 5 | Integer | Số thứ tự gói tin (0 - 65535). Dùng để phát hiện mất gói (Packet Loss). |
| **Separator 1** | 5 | String | Chuỗi cố định `,chk:` phân cách. |
| **Checksum** | 2 | Hex String | Mã kiểm tra lỗi 8-bit (2 ký tự Hex), ví dụ: `4A`, `0F`. |
| **Separator 2** | 6 | String | Chuỗi cố định `,data:` phân cách. |
| **Payload** | Variable | String | Dữ liệu thực tế (Lệnh hoặc Tọa độ). |
| **Footer** | 1 | Char | Ký tự đóng `}` để nhận diện kết thúc gói tin. |

**Tổng Overhead cố định:** Khoảng 19-24 bytes cho mỗi gói tin (tùy độ lớn của số Sequence).

### 3.3. Minh họa Xây dựng Gói tin (Example Construction)

Giả sử hệ thống cần gửi lệnh **"CAPTURE"** với số thứ tự **Sequence = 10**.

**Bước 1: Tính Checksum (CRC8-XOR)**
Thuật toán XOR tuần tự từng byte của Payload "CAPTURE":
*   `C` (0x43) XOR `A` (0x41) = **0x02**
*   `0x02` XOR `P` (0x50) = **0x52**
*   `0x52` XOR `T` (0x54) = **0x06**
*   `0x06` XOR `U` (0x55) = **0x53**
*   `0x53` XOR `R` (0x52) = **0x01**
*   `0x01` XOR `E` (0x45) = **0x44**

$\rightarrow$ Kết quả Decimal: 68
$\rightarrow$ Kết quả Hex: **44**

**Bước 2: Ghép chuỗi (String Concatenation)**
1.  Header: `{seq:`
2.  Sequence: `10`
3.  Separator: `,chk:`
4.  Checksum: `44`
5.  Separator: `,data:`
6.  Payload: `CAPTURE`
7.  Footer: `}`

$\rightarrow$ **Gói tin hoàn chỉnh:** `{seq:10,chk:44,data:CAPTURE}`

---

## 4. Cơ chế Đảm bảo Tin cậy (Reliability Mechanism)

Do BLE sử dụng sóng vô tuyến (2.4GHz) dễ bị nhiễu và mất mát dữ liệu, giao thức này cài đặt 2 lớp bảo vệ:

### 4.1. Tính Toàn vẹn (Integrity) - Lớp Chống Sai lệch
*   **Vấn đề:** Bit bị lật (0 thành 1) do nhiễu môi trường.
*   **Giải pháp:** Bên nhận (Receiver) tự tính lại Checksum của phần `data`.
*   **Logic:**
    ```text
    IF (Calculated_Checksum == Received_Checksum) -> Chấp nhận (Valid)
    ELSE -> Huỷ gói tin (Drop) & Báo lỗi Checksum Mismatch
    ```

### 4.2. Tính Liên tục (Continuity) - Lớp Chống Mất gói
*   **Vấn đề:** Gói tin bị mất hoàn toàn trên đường truyền.
*   **Giải pháp:** Kiểm tra số `seq` tăng dần đều.
*   **Logic:**
    ```text
    Expected_Seq = (Last_Seq + 1) % 65536
    IF (Current_Seq != Expected_Seq) -> Phát hiện khoảng trống (Gap)
    Lost_Packets = Current_Seq - Expected_Seq
    ```
*   **Hiệu quả:** Cho phép hệ thống đo lường chính xác chất lượng đường truyền (Packet Loss Rate) để cảnh báo người dùng.

---

## 5. Đánh giá Ưu/Nhược điểm

*   **Ưu điểm:**
    *   **Dễ gỡ lỗi:** Có thể đọc hiểu gói tin bằng mắt thường (Human-readable) qua Log Cat hoặc Serial Monitor.
    *   **Đơn giản:** Thuật toán XOR cực nhẹ, không tốn tài nguyên CPU của ESP32.
    *   **Tin cậy:** Khắc phục được nhược điểm "gửi rồi quên" (Fire-and-forget) của giao thức BLE Notify/WriteNoResponse.

*   **Nhược điểm:**
    *   **Tốn băng thông (Overhead):** Mỗi gói tin tốn thêm ~20 bytes cho header. Với Payload nhỏ (ví dụ "OK"), header còn nặng hơn dữ liệu thật. Tuy nhiên, với BLE 5.0 và MTU 512 bytes, overhead này là không đáng kể (chấp nhận được).
