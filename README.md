# Mô phỏng EEZYbotARM Mk2 — Python + MuJoCo

Bộ mã mô phỏng cho đồ án CE212 "Tay máy robot 3-DOF phân loại sản phẩm". Gói này gồm:

- Mô hình MuJoCo của Mk2 dựng từ **chính các file STL gốc**, có **3 vòng hình bình hành kín**.
- Mô hình servo MG996R.
- Thư viện FK/IK giải tích kèm lớp ánh xạ góc servo.
- Thí nghiệm M0/M2 vẽ ngôi sao + đường tròn, đúng như mô tả trong đề cương.

Đã chạy thử trọn quy trình trên **Python 3.8.20 + mujoco 3.2.3**, cài sạch bằng `pip install -r requirements.txt`.

---

## 0. Phương án đã chốt

| Lớp | Công cụ | Dùng cho |
|---|---|---|
| Động học + quỹ đạo | Python 3.8, NumPy, Ruckig 0.12.2 | FK/IK, vùng làm việc, q̇/q̈/jerk của lệnh, thời gian quỹ đạo, chọn kích thước hình vẽ |
| Động lực học | MuJoCo 3.2.3 | Hình bình hành kín (`equality/connect`), quán tính từ STL, servo bão hòa mô-men, độ rơ, lệnh giữ 20 ms |

## 1. Cấu trúc thư mục

```
eezybot_mk2_sim/
├── params.py            # MỌI tham số (kích thước, servo, khối lượng...) — có ghi nguồn
├── mk2_kinematics.py    # FK, IK, Jacobian, ánh xạ góc <-> servo (port sang ESP32)
├── build_model.py       # STL gốc -> model/mk2.xml + model/meshes/
├── sim_common.py        # nạp mô hình, giữ lệnh 20 ms, đọc góc thực
├── trajectories.py      # hình sao + tròn, M0 (tuyến tính), M2 (double-S/Ruckig)
├── check_kinematics.py  # kiểm chứng FK/IK, so MuJoCo vs giải tích, vùng làm việc
├── run_sim.py           # thí nghiệm M0 vs M2, đồ thị + CSV + chỉ số
├── snapshot.py          # chụp ảnh tư thế cho báo cáo
├── tools/holes.py       # đo tâm lỗ trục trên STL (nguồn số liệu hình học)
└── requirements.txt
```

## 2. Cài đặt trên Ubuntu 20.04

```bash
# 2.1 Kiểm tra Python (Ubuntu 20.04 mặc định 3.8.x)
python3 --version

# 2.2 Gói hệ thống cho venv
sudo apt update
sudo apt install -y python3-venv python3-pip unzip

# 2.3 Tạo môi trường ảo riêng
cd ~
python3 -m venv ~/venvs/mk2
source ~/venvs/mk2/bin/activate
pip install --upgrade pip

# 2.4 Cài thư viện (đặt thư mục eezybot_mk2_sim ở ~/eezybot_mk2_sim)
cd ~/eezybot_mk2_sim
pip install -r requirements.txt

# 2.5 Kiểm tra
python -c "import mujoco, ruckig; print('mujoco', mujoco.__version__)"   # -> mujoco 3.2.3
```

Mỗi lần mở terminal mới, chạy lại `source ~/venvs/mk2/bin/activate`.

## 3. Dựng mô hình từ STL

```bash
# Giải nén file zip part 1 tải từ Thingiverse (chỉ part 1 có STL)
mkdir -p ~/mk2_stl
unzip EEZYbotARM_MK2_-_1454048_-_part_1_of_2.zip -d ~/mk2_stl

cd ~/eezybot_mk2_sim
python build_model.py --stl-dir ~/mk2_stl/files
```

Kết quả mong đợi:
```
  kiem tra lo C cua EBAmk2_007_trialink: lech 0.01 mm
  kiem tra lo T cua EBAmk2_006_horarm__: lech 0.00 mm
  ...
TONG khoi luong chuyen dong: 552.3 g  (FILL_FACTOR=1.0)
Servo: tau_stall=0.922 N m, w0=5.51 rad/s, kp=10.563 N m/rad, b=0.1673 N m s/rad
```

Các dòng "lech" là phép kiểm tra hình học: lỗ thứ 3 của mỗi chi tiết phải rơi đúng khớp tính từ `params.py`. Nếu lệch quá 1 mm, script sẽ dừng.

**Quan trọng:** sau mỗi lần sửa `params.py`, phải chạy lại `build_model.py`.

## 4. Xem mô hình và điều khiển bằng tay

```bash
python -m mujoco.viewer --mjcf=model/mk2.xml
```

- Panel bên phải → mục **Control**: kéo 3 thanh `servo_base`, `servo_shoulder`, `servo_elbow` (đơn vị rad, 0 = tư thế home).
- Bạn sẽ thấy hình bình hành giữ cho cổ tay luôn nằm ngang.
- Phím tắt hữu ích: `Space` để tạm dừng; nhấp đúp vào một chi tiết để chọn nó; sau đó giữ `Ctrl` và kéo chuột để đẩy hoặc xoay chi tiết đó.

## 5. Bước kiểm chứng động học (mục "Mô phỏng" trong đề cương)

```bash
python check_kinematics.py
```

Kết quả với tham số mặc định:
```
[1] FK(IK(p)) tren 5000 diem: max |FK(IK(p))-p| = 1.04e-13 mm
[2] MuJoCo (vong kin, g=0) vs FK giai tich, 40 tu the: max sai lech = 0.005 mm
[3] Do vong do trong luc ... lech dau but = 0.93 .. 1.81 mm
[4] Hinh ve lon nhat ... z_giay = 60 mm: tam x = 180 mm, R_max = 84.3 mm
```

Ý nghĩa từng dòng:
1. Công thức IK là nghịch đảo đúng của FK.
2. Mô hình vật lý có vòng kín cho cùng vị trí đầu bút với công thức giải tích. Điều này chứng minh **lớp ánh xạ hình bình hành** là đúng.
3. Servo là bộ điều khiển P hữu hạn, nên dưới trọng lực đầu bút võng xuống khoảng 1–2 mm. Đây là hiệu ứng thật mà mô hình động học thuần không thấy được.
4. Bảng để **chọn R và độ cao mặt giấy**: chọn R nhỏ hơn hẳn R_max để còn khoảng dự phòng. Ảnh vùng làm việc được lưu ở `results/workspace_rz.png`.

## 6. Chạy thí nghiệm M0 vs M2

```bash
python run_sim.py                    # mặc định: R=40, tâm (180,0), giấy z=60 mm
python run_sim.py --view             # xem robot vẽ theo thời gian thực
python run_sim.py --R 30 --z 60 --vmax 150 --amax 1500 --jmax 30000
```

Kết quả với tham số mặc định:
```
Duong di L = 541.9 mm (ly thuyet 13.55R = 542.0); T(M2) = 5.44 s; v(M0) = 99.6 mm/s
[M2] toc do servo lon nhat = [2.14 0.82 0.5] rad/s (MG996R khong tai 5.51 rad/s) -> OK
PP    vot lo TB  vot lo max  sai lech dinh TB  do tron  lech thang max   (mm)
M0        -0.98       -0.61              1.16     0.68            0.65
M2         0.05        0.23              0.09     0.75            0.64
```

Các file trong `results/`:

| File | Nội dung |
|---|---|
| `cmd_derivatives.png` | **Bằng chứng 1**: q̇, q̈, jerk của lệnh M0 và M2 chồng lên nhau |
| `pen_trace.png` | Vết bút M0, M2 so với hình lý tưởng, kèm ảnh phóng to mũi sao |
| `log_M0.csv`, `log_M2.csv` | Lệnh 20 ms, góc servo, vị trí bút, góc thực, mô-men servo |

Cách đọc các chỉ số (đo giống cách đo trên giấy trong đề cương):
- **Vọt lố**: đo so với **giao điểm hai cạnh vẽ thật kéo dài**. Giá trị **âm** nghĩa là nét vẽ **cắt góc**, không chạm tới giao điểm.
- **Sai lệch đỉnh**: khoảng cách ngắn nhất từ nét vẽ tới giao điểm. Chỉ số này bắt được cả trường hợp vọt lố lẫn cắt góc.
- **Độ tròn**: D_max − D_min theo 4 hướng 0°, 45°, 90°, 135°.
- **Lệch thẳng**: khe hở lớn nhất trên đoạn 15–85% của mỗi cạnh.

> Phát hiện từ mô phỏng (cần kiểm chứng trên phần cứng): với mô hình servo hiện tại, M0 **cắt góc** khoảng 1 mm chứ không vọt lố. Vì vậy chỉ số "vọt lố" trong đề cương có thể ra 0 hoặc âm với M0. Nên đo thêm "sai lệch đỉnh". Ngoài ra, khi M2 chạy nhanh hơn trên cung tròn, độ tròn của M2 chưa chắc tốt hơn M0.

Lưu ý: M2 trong gói này dùng **Ruckig** làm chuẩn tham chiếu, như đề cương mô tả. Bộ double-S tự viết của bạn (để chạy trên ESP32) có thể đưa vào thay hàm `m2_path` trong `trajectories.py`, rồi so sánh với Ruckig.

## 7. Hiệu chỉnh cho giống thực tế (quan trọng nhất)

Các tham số đánh dấu `[GIA-DINH]` trong `params.py` là giá trị tạm. Hãy đo trên tay máy thật theo thứ tự sau, sửa `params.py`, rồi chạy lại `build_model.py`:

| # | Tham số | Cách đo |
|---|---|---|
| 1 | `TOOL_OFFSET`, `TOOL_MASS_G` | Đo khoảng cách ngang/dọc từ tâm trục cổ tay (lỗ M4 phía trước tay ngang) tới đầu bút; cân giá bút + bút |
| 2 | `H_SHOULDER` | Thước kẹp: từ mặt bàn tới tâm trục vai. Giá trị 93,3 mm là suy ra từ STL, có kể khe hở bi |
| 3 | `SERVO_OFFSET`, `SERVO_SIGN` | Ra lệnh 90/90/90, kiểm tra tay chính thẳng đứng, tay ngang nằm ngang, khớp đế nhìn thẳng; ghi độ lệch và chiều quay |
| 4 | `THETA2_LIM`, `REL_LIM`, `THETA1_LIM` | Quay từ từ từng servo tới khi chạm cơ khí, lùi lại 3–5° |
| 5 | `MEASURED_MASS_G` | Cân từng chi tiết sau khi in, điền vào dict. Mặc định đang giả định in đặc 100% (cận trên) |
| 6 | `SERVO_ERR_SAT_DEG` | Ra lệnh bước 30° và quay video 240 fps (hoặc đọc chiết áp bên trong servo, theo tài liệu Adafruit [12] trong đề cương); chỉnh tham số tới khi đáp ứng mô phỏng khớp với đo |
| 7 | `BACKLASH_DEG` | Giữ cố định servo, lắc nhẹ đầu tay và đo độ dơ ở đầu bút; chia cho cánh tay đòn để ra độ dơ góc |
| 8 | `BASE_GEAR_RATIO` | Đếm răng: gearmast 50 / gearservo 25 = 2,0 (bản `22DENTI` thì 50/22) |

Cách kiểm tra mô hình đã sát thực tế: chạy cùng một quỹ đạo trên tay máy thật, rồi so ảnh tờ vẽ với `pen_trace.png` và so 3 chỉ số đo.

## 8. Giới hạn của mô hình (ghi rõ trong báo cáo)

- Các chi tiết in là **vật rắn tuyệt đối**: không mô hình hóa độ đàn hồi của PLA. Rung do uốn khâu sẽ không xuất hiện.
- Servo được mô hình là bộ P bão hòa mô-men cộng đường đặc tính mô-men–tốc độ của động cơ DC (đặt bằng `damping`). Bộ điều khiển thật bên trong MG996R không được công bố. Không mô hình vùng chết và độ phân giải xung.
- Không có va chạm (giữa các khâu, bút–giấy). Lực lò xo của giá bút không được mô hình hóa.
- Vị trí các chi tiết theo phương ngang (dọc trục khớp) trong `build_model.py` chỉ là gần đúng. Sai lệch này chỉ ảnh hưởng hình hiển thị và rất ít đến quán tính.
- Khối lượng ốc vít, thanh ren và bi không được tính. Hai servo vai được đặt gần đúng hai bên trục vai.

## 9. Port sang ESP32

`mk2_kinematics.py` chỉ dùng `math`, nên có thể chép nguyên công thức sang C/C++ (dùng `float`). Các hàm cần port:

- `ik()`: atan2, acos, chọn nghiệm khuỷu trên.
- `to_servo()`: lớp ánh xạ với offset, chiều quay và hộp số 2:1.
- `within_limits()`: kiểm tra giới hạn trước khi gửi lệnh.

Để đối chiếu float (ESP32) với double (PC) như đề cương yêu cầu, hãy so cột `servo*_deg` trong `results/log_M2.csv` với log UART của ESP32 cho cùng một quỹ đạo.

## 10. Xử lý lỗi thường gặp

| Lỗi | Cách xử lý |
|---|---|
| `Khong tim thay ... .STL` | `--stl-dir` phải trỏ tới thư mục `files` bên trong zip part 1 |
| `IKError: ... ngoai gioi han` | Hình vẽ vượt vùng làm việc: giảm `--R` hoặc đổi `--z`, `--cx` theo bảng ở bước 5 |
| `toc do servo ... VUOT` | Giảm `--vmax`/`--amax` (quỹ đạo dài thời gian hơn) |
| Cửa sổ viewer không mở / lỗi GLFW | Kiểm tra driver NVIDIA (`nvidia-smi`); thử `sudo apt install libglfw3`; hoặc chạy không cửa sổ (bỏ `--view`) |
| Viewer chạy giật | Bình thường với lưới STL đầy đủ; tắt hiển thị bóng trong panel Rendering |

## 11. Nguồn

- STL và BOM: daGHIZmo, *EEZYbotARM Mk2*, Thingiverse thing:1454048 (CC BY-NC). Kích thước đo bằng `tools/holes.py`.
- MG996R: TowerPro, https://towerpro.com.tw/product/mg996r/ (9,4 kgf·cm và 0,19 s/60° ở 4,8 V; 11 kgf·cm và 0,15 s/60° ở 6 V; 55 g). Trang này ghi 0,17 s ở một bảng khác; gói này dùng 0,19 s như đề cương.
- Mật độ PLA 1,24 g/cm³: NatureWorks, *Ingeo 4043D Technical Data Sheet* (ASTM D792).
- MuJoCo: XML Reference (`equality/connect`, `actuator/position`, `armature`, `frictionloss`) và mục Backlash trong Modeling, https://mujoco.readthedocs.io
- Ruckig: Berscheid & Kröger, RSS 2021 (tài liệu [2] trong đề cương).
- Giới hạn khớp mặc định (độ rộng 90°): URDF trong https://github.com/HotBlackRobotics/ntbd
