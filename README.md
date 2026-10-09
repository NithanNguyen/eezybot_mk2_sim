# Mô phỏng EEZYbotARM Mk2 — Python + MuJoCo

### FAST RUN
```bash
source ~/venvs/mk2/bin/activate
cd ~/eezybot_mk2_sim

python run_sim.py --view      # xem tay máy cầm bút vẽ, vệt mực hiện dần trên giấy
python run_sim.py             # chạy không cửa sổ, ghi kết quả vào results/
python -m mujoco.viewer --mjcf=model/mk2.xml   # điều khiển tay và kẹp bằng thanh trượt
python check_kinematics.py    # kiểm chứng động học (khoảng 45 giây)
```



Bộ mã mô phỏng cho đồ án CE212 "Tay máy robot 3-DOF phân loại sản phẩm". Gói này gồm:

- Mô hình MuJoCo của Mk2 dựng từ **chính 19 file STL gốc**, gồm cả **kẹp** (gripper), có **3 vòng hình bình hành kín**.
  Mọi chi tiết đặt theo tâm lỗ trục và mặt tiếp giáp đo trên STL. Kiểm tra bằng `tools/check_assembly.py`: không chi tiết nào lồng vào nhau trong giới hạn khớp.
- Kẹp giữ **bút Ø10 thẳng đứng, ngòi hướng xuống, cách điểm kẹp 90 mm**. Góc mở ngón để kẹp vừa bút được tính từ hình dạng má kẹp.
- **Tờ giấy A4 dày 1 mm** nằm trên mặt bàn. **Vệt mực rộng 1 mm** chỉ được ghi khi MuJoCo báo ngòi chạm giấy, và hiện trong viewer.
- Mô hình servo MG996R (đế, vai, khuỷu) và SG90 (kẹp). Hộp servo vẽ theo kích thước datasheet.
- Thư viện FK/IK giải tích kèm lớp ánh xạ góc servo.
- Thí nghiệm M0/M2 vẽ ngôi sao + đường tròn, đúng như mô tả trong đề cương; chỉ số đo trên vết mực.

> **Trạng thái kiểm chứng (2026-10-09, bản có kẹp + bút + giấy).** Toàn bộ quy trình bước 3 → 9 đã được chạy lại từ đầu trong venv sạch
> **Python 3.8.20 + `pip install -r requirements.txt`**, với bộ STL `EBAmk2_*.STL` (xem bước 4). Mọi số liệu "Kết quả mong đợi"
> bên dưới là output thật của lần chạy đó. 2 đồ thị `cmd_derivatives.png` và `pen_trace.png` trong `assets/simulation_results/` trùng **từng byte** (md5) với ảnh vừa sinh ra.
> Lần chạy này **không** làm trên Ubuntu 20.04 thật mà trên Linux x86_64 khác; khả năng tương thích với Ubuntu 20.04 được suy ra
> từ thẻ wheel (mọi gói là `cp38 manylinux` cần glibc ≤ 2.28, Ubuntu 20.04 có glibc 2.31). Bước 2 giúp bạn tự xác nhận điều này trên máy mình.
> Cửa sổ viewer (bước 6) chỉ được chạy thử trên màn hình ảo (Xvfb), chưa thử với driver NVIDIA thật.

---

## Cách đọc hướng dẫn này

Mỗi bước có một khối **✅ Kiểm tra** gồm lệnh, kết quả mong đợi và tiêu chí **PASS**.
**Chỉ chuyển sang bước tiếp theo khi bước hiện tại PASS.** Nếu FAIL, xem dòng "Nếu FAIL" ngay dưới đó hoặc bảng lỗi ở mục 13.

Quy ước: `~` là thư mục home của bạn; mọi lệnh `python` (từ bước 3.3 trở đi) phải chạy **trong venv** (dấu nhắc có `(mk2)`).

---

## 0. Phương án đã chốt

| Lớp | Công cụ | Dùng cho |
|---|---|---|
| Động học + quỹ đạo | Python 3.8, NumPy, Ruckig 0.12.2 | FK/IK, vùng làm việc, q̇/q̈/jerk của lệnh, thời gian quỹ đạo, chọn kích thước hình vẽ |
| Động lực học | MuJoCo 3.2.3 | Hình bình hành kín (`equality/connect`), bánh răng kẹp (`equality/joint`), quán tính từ STL, servo bão hòa mô-men, độ rơ, lệnh giữ 20 ms, tiếp xúc ngòi–giấy |

Vì sao ghim đúng các phiên bản này: Ubuntu 20.04 dùng Python 3.8. Theo PyPI, **mujoco 3.2.3** là bản cuối có wheel `cp38`
(3.2.4 yêu cầu Python ≥ 3.9), **ruckig 0.12.2** là bản cuối có wheel `cp38` (với Python 3.8, bản 0.14.0 và 0.15.3 chỉ có mã nguồn, pip sẽ phải tự biên dịch).
NumPy 1.24.4, SciPy 1.10.1, Matplotlib 3.7.5, NetworkX 3.1 là các bản cuối hỗ trợ Python 3.8 (bản kế tiếp của mỗi gói yêu cầu Python ≥ 3.9).
**Không nâng cấp lẻ từng gói.**

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
├── tools/check_assembly.py  # kiểm chứng lắp ráp: không chi tiết nào lồng vào nhau; suy ra giới hạn khớp
├── assets/simulation_results/   # ảnh kết quả mẫu để đối chiếu
├── requirements.txt
├── model/               # (sinh ra ở bước 5, không có sẵn trong repo)
└── results/             # (sinh ra ở bước 7–9)
```

## 2. Kiểm tra máy trước khi cài

### 2.1 Hệ điều hành, glibc, Python

```bash
lsb_release -ds            # hệ điều hành
uname -m                   # kiến trúc CPU
ldd --version | head -1    # phiên bản glibc
python3 --version          # Python hệ thống
```

✅ **Kiểm tra — PASS khi:** dòng 1 là `Ubuntu 20.04.x LTS`; dòng 2 là `x86_64`; glibc **≥ 2.28** (Ubuntu 20.04 là `2.31`);
Python là `Python 3.8.x` (x bất kỳ).

- Nếu FAIL ở Python (ví dụ 3.10, 3.12): các phiên bản ghim trong `requirements.txt` sẽ **không cài được**. Dừng lại, không sửa `requirements.txt` theo cảm tính.
- Lý do kiểm tra glibc: wheel của `glfw` và `pillow` được đóng gói theo chuẩn `manylinux_2_28`, tức cần glibc ≥ 2.28.

### 2.2 GPU và OpenGL (chỉ cần cho viewer ở bước 6 và ảnh ở bước 9)

```bash
sudo apt update
sudo apt install -y mesa-utils           # cung cấp lệnh glxinfo
nvidia-smi                               # driver NVIDIA có hoạt động không
glxinfo -B | grep -E "direct rendering|OpenGL renderer"
```

✅ **Kiểm tra — PASS khi:**
- `nvidia-smi` in ra bảng có tên GPU (RTX 3050) và phiên bản driver.
- `direct rendering: Yes`.
- `OpenGL renderer` là GPU thật (NVIDIA hoặc Intel), **không phải `llvmpipe`**.

Nếu FAIL: `llvmpipe` nghĩa là đang vẽ bằng CPU (phần mềm). Bước 3, 5, 7, 8 (không có `--view`) **vẫn chạy được** vì không cần OpenGL,
nhưng viewer sẽ rất chậm hoặc không mở. Hãy sửa driver trước khi tới bước 6.

*Tùy chọn (laptop hai GPU):* nếu `glxinfo` báo GPU Intel và bạn muốn chạy viewer trên NVIDIA, thêm 2 biến môi trường của
NVIDIA PRIME Render Offload trước lệnh, ví dụ:

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia glxinfo -B | grep "OpenGL renderer"   # -> NVIDIA ...
```

Nếu dòng trên in ra NVIDIA, bạn có thể đặt cùng tiền tố đó trước các lệnh `python ...` ở bước 6. GPU Intel cũng đủ cho mô hình này.

## 3. Cài đặt

### 3.1 Gói hệ thống

```bash
sudo apt install -y python3-venv python3-pip unzip git
```

✅ **Kiểm tra:**

```bash
python3 -m venv --help > /dev/null && echo "venv OK"
git --version && unzip -v | head -1
```

PASS khi in ra `venv OK`, phiên bản git và phiên bản unzip.

### 3.2 Lấy mã nguồn

```bash
git clone https://github.com/NithanNguyen/eezybot_mk2_sim.git ~/eezybot_mk2_sim
```

✅ **Kiểm tra:**

```bash
ls ~/eezybot_mk2_sim/requirements.txt ~/eezybot_mk2_sim/build_model.py
```

PASS khi cả hai đường dẫn được in ra, không có `No such file or directory`.

### 3.3 Môi trường ảo riêng

```bash
python3 -m venv ~/venvs/mk2
source ~/venvs/mk2/bin/activate
pip install --upgrade pip
```

✅ **Kiểm tra:**

```bash
which python      # -> /home/<tên-bạn>/venvs/mk2/bin/python
python --version  # -> Python 3.8.x
pip --version     # -> pip 25.0.1 ... (python 3.8)
```

PASS khi `which python` trỏ vào `venvs/mk2`. Nếu nó trỏ vào `/usr/bin/python3` thì venv **chưa được kích hoạt**: chạy lại lệnh `source`.
(pip 25.0.1 là bản pip cuối hỗ trợ Python 3.8; pip tự chọn bản này, không cần làm gì thêm.)

**Mỗi lần mở terminal mới**, chạy lại `source ~/venvs/mk2/bin/activate` rồi kiểm tra lại bằng `which python`.

### 3.4 Cài thư viện

```bash
cd ~/eezybot_mk2_sim
pip install -r requirements.txt
```

✅ **Kiểm tra 1 — phụ thuộc nhất quán:**

```bash
pip check     # -> No broken requirements found.
```

✅ **Kiểm tra 2 — đúng phiên bản và import được** (dán nguyên khối vào terminal):

```bash
python - <<'EOF'
from importlib.metadata import version
want = {"mujoco": "3.2.3", "numpy": "1.24.4", "matplotlib": "3.7.5", "ruckig": "0.12.2",
        "trimesh": "4.4.9", "networkx": "3.1", "scipy": "1.10.1"}
bad = [k for k, v in want.items() if version(k) != v]
for k, v in want.items():
    print(f"{'OK ' if version(k) == v else 'SAI'} {k:10s} {version(k):8s} (can {v})")
import mujoco, ruckig, trimesh, scipy, matplotlib, PIL, glfw
print("PASS" if not bad else f"FAIL: sai phien ban {bad}")
EOF
```

Kết quả mong đợi: 7 dòng bắt đầu bằng `OK`, dòng cuối `PASS`.

Nếu FAIL với `No matching distribution found for mujoco==3.2.3`: Python không phải 3.8 hoặc máy không phải x86_64 → quay lại bước 2.1.

## 4. Lấy bộ STL gốc

`build_model.py` cần **19 file** sau, tên viết **đúng chữ hoa/thường** (Linux phân biệt `.STL` và `.stl`):

```
EBAmk2_001_base  002_mainarm  003_varm  004_link135  005_link135angled  006_horarm__
EBAmk2_007_trialink  008_link147_new  009_trialinkfront  010_gearservo  011_gearmast  012_mainbase  013_lower_base
EBAmk2_014_claw_base  015_claw_finger_dx  016_claw_gear_drive  017_claw_finger_sx  018_claw_gear_driven  019_drive_cover
```

Các file còn lại trong bộ STL (`006_horarm_plate`, `010_gearservo_22DENTI`, `011_gearmast_full`) là biến thể thay thế, không dùng.

**Cách A — nguồn gốc (khuyến nghị):** tải toàn bộ file của *EEZYbotARM Mk2* tại Thingiverse thing:1454048
(https://www.thingiverse.com/thing:1454048), rồi giải nén:

```bash
mkdir -p ~/mk2_stl
unzip ~/Downloads/<ten-file-zip-tai-ve>.zip -d ~/mk2_stl
# Nếu Thingiverse chia thành nhiều file zip, giải nén file chứa thư mục files/ với các file EBAmk2_*.STL
```

**Cách B — bản sao trên GitHub** (dùng khi không tải được từ Thingiverse). Đây là bản sao do bên thứ ba đăng lại, không phải nguồn gốc;
bộ file này đã cho ra **đúng** các số liệu mong đợi bên dưới:

```bash
git clone --depth 1 https://github.com/mchora2/EEZYbotARMMK2 ~/mk2_stl/EEZYbotARMMK2
```

Sau đó **tìm thư mục chứa STL** (không đoán đường dẫn) và lưu vào biến `STL_DIR`:

```bash
find ~/Documents/dktd/mk2_stl -iname "EBAmk2_001_base.stl"
# Ví dụ in ra /home/<tên-bạn>/mk2_stl/files/EBAmk2_001_base.STL  -> thư mục là ~/Documents/dktd/mk2_stl/files
STL_DIR=/home/thienan/Documents/dktd/mk2_stl/files
echo "$STL_DIR"
```

✅ **Kiểm tra — đủ 19 file:**

```bash
miss=0; for f in 001_base 002_mainarm 003_varm 004_link135 005_link135angled 006_horarm__ \
  007_trialink 008_link147_new 009_trialinkfront 010_gearservo 011_gearmast 012_mainbase 013_lower_base \
  014_claw_base 015_claw_finger_dx 016_claw_gear_drive 017_claw_finger_sx 018_claw_gear_driven 019_drive_cover; do
  if [ -f "$STL_DIR/EBAmk2_$f.STL" ]; then echo "OK    EBAmk2_$f.STL"; else echo "THIEU EBAmk2_$f.STL"; miss=1; fi
done; [ $miss -eq 0 ] && echo PASS || echo FAIL
```

PASS khi có 19 dòng `OK` và dòng cuối `PASS`.
Nếu `find` chỉ thấy tên đuôi `.stl` (chữ thường), script sẽ báo `THIEU`; khi đó đổi tên file sang `.STL` hoặc dùng cách B.
Lưu ý: biến `STL_DIR` chỉ tồn tại trong terminal hiện tại.

## 5. Dựng mô hình từ STL

```bash
cd ~/eezybot_mk2_sim
python build_model.py --stl-dir "$STL_DIR"
```

Kết quả mong đợi (tham số mặc định trong `params.py`):

```
  kiem tra lo C cua EBAmk2_007_trialink: lech 0.01 mm
  kiem tra lo T cua EBAmk2_006_horarm__: lech 0.00 mm
  kiem tra lo O cua 001_base: lech 0.00 mm
  kiem tra lo F cua 001_base: lech 0.00 mm
  kep: goc ngon kep but D10 = 173.21 do (mo 6.79 do tu luc dong), goc truc SG90 = 10.87 do tu luc dong
  diem kep - W = (95.98, 0.475, -8.00) mm, kep nghieng 0.27 do
  ngoi but o home: STL [243.41   0.47 131.25]  FK [243.4    0.48 131.25]  lech 0.004 mm
chi tiet | khoi luong (g) | the tich (cm3) | ghi chu
EBAmk2_001_base | 178.2 | 143.7 | luoi da va lo
...
TONG khoi luong chuyen dong: 577.3 g  (FILL_FACTOR=1.0)

Servo: tau_stall=0.922 N m, w0=5.51 rad/s, kp=10.563 N m/rad, b=0.1673 N m s/rad
Da ghi .../eezybot_mk2_sim/model/mk2.xml
```

Ý nghĩa các dòng kiểm tra:
- Các dòng `lech` là phép kiểm tra hình học: lỗ thứ 3 của mỗi chi tiết phải rơi đúng khớp tính từ `params.py`. Nếu lệch quá 1 mm, script sẽ dừng.
- Dòng `kep`: góc ngón để hai má kẹp vừa chạm bút Ø10, tính từ hình dạng má kẹp trong STL. 2 ngón chạm nhau khi ngón phải quay 180°; kẹp bút Ø10 cần mở mỗi ngón 6,79°, tức trục SG90 quay 10,87° (tỉ số bánh răng 016 → 018 là 16/10).
- Dòng `diem kep - W` được so với `params.GRIP_POINT_W`. Nếu không khớp, script dừng và in giá trị đúng để chép vào `params.py`.
- Dòng `ngoi but o home`: vị trí ngòi tính từ STL phải trùng FK giải tích của `mk2_kinematics.py`, sai lệch < 0,05 mm.

✅ **Kiểm tra — MuJoCo nạp được mô hình:**

```bash
python -c "import mujoco; m=mujoco.MjModel.from_xml_path('model/mk2.xml'); print('nbody',m.nbody,'njnt',m.njnt,'neq',m.neq,'nu',m.nu,'nmesh',m.nmesh)"
ls model/meshes | wc -l
```

Kết quả mong đợi:

```
nbody 14 njnt 13 neq 6 nu 4 nmesh 19
19
```

PASS khi: mọi dòng `lech` ≤ 0,05 mm, dòng `ngoi but o home` lệch < 0,05 mm, tổng khối lượng `577.3 g` (nếu chưa sửa `params.py`), và dòng nạp mô hình in đúng như trên.

Ý nghĩa các con số:
- 14 body: world, 9 khâu tay, 2 ngón kẹp, bánh răng 016, gearservo 010.
- 13 khớp: 9 khớp tay, 2 khớp ngón, khớp trục SG90, khớp gearservo.
- 6 ràng buộc: 3 vòng hình bình hành và 3 cặp bánh răng (ngón–ngón, 016–018, 010–011).
- 4 servo: đế, vai, khuỷu, kẹp.
- 19 lưới STL.

Nếu đặt `BACKLASH_DEG > 0`, mỗi khớp dẫn động có thêm một khớp "play", nên `njnt` sẽ là 16.

**Quan trọng:** sau mỗi lần sửa `params.py`, phải chạy lại `build_model.py`.

### 5b. Kiểm chứng lắp ráp STL (khuyến nghị, chạy một lần)

```bash
python tools/check_assembly.py --stl-dir "$STL_DIR"
```

Script dựng trường khoảng cách có dấu (SDF) cho từng chi tiết, rồi kiểm tra hai việc:
- **[A]** Tại tư thế home, liệt kê các cặp chi tiết nằm sát nhau.
- **[B]** Tại 33 tư thế phủ hết giới hạn khớp, kiểm tra không cặp chi tiết nào lồng vào nhau. Tư thế đầu tiên được thử thêm với khớp đế ±45°.

Script chạy khoảng 5–8 phút. Kết quả mong đợi (rút gọn):

```
[A] Tu the home — cac cap cham nhau (khe < 0.5 mm; am nho la sai so luoi):
    015_claw_finger_dx           - 017_claw_finger_sx           khe -0.28 mm
    ...
    005_link135angled            - 001_base                     khe +0.50 mm
[B] 33 tu the trong gioi han (theta2 (42.5, 125.0), phi (-65.0, 20.0), rel (-145.0, -60.0)):
    khe nho nhat = -0.28 mm (015_claw_finger_dx - 017_claw_finger_sx tai (42.5, -65.0, -45.0))
PASS: khong chi tiet nao lan vao nhau trong gioi han khop
```

✅ **PASS khi** dòng cuối là `PASS`. Khe âm cỡ −0,3 mm là sai số của lưới 0,3 mm tại các mặt đang tì vào nhau (ví dụ hai má kẹp, hoặc má kẹp với thân bút), không phải va chạm thật. Ngưỡng báo lỗi là −0,35 mm.

Thêm `--limits` để in bảng va chạm theo (θ2, φ) rộng ra ngoài giới hạn. Phần này chạy thêm khoảng 10 phút; đây là cách đã dùng để đặt `THETA2_LIM`, `PHI_LIM`, `REL_LIM` (mục 10).

## 6. Xem mô hình và điều khiển bằng tay

Điều kiện: bước 2.2 PASS (không phải `llvmpipe`).

```bash
python -m mujoco.viewer --mjcf=model/mk2.xml
```

✅ **Kiểm tra bằng mắt — PASS khi:**
1. Cửa sổ mở ra, thấy tay máy, tờ giấy A4 màu trắng trước đế và kẹp đang giữ bút màu xanh thẳng đứng. Terminal không báo lỗi.
2. Ở tư thế ban đầu (các thanh điều khiển = 0): tay chính dựng đứng, tay ngang nằm ngang, kẹp và bút ở phía trước.
3. Mở panel bên phải → mục **Control** (phím tắt `Alt`+`C`), kéo 3 thanh `servo_base`, `servo_shoulder`, `servo_elbow`; tay máy bám theo và **kẹp luôn nằm ngang, bút luôn thẳng đứng** nhờ hình bình hành.
4. Kéo thanh `servo_gripper` về phía âm: hai ngón mở ra đối xứng, bánh răng quay theo. Bút được gắn cứng vào kẹp nên không rơi.

Đơn vị thanh trượt là rad.
- 3 servo tay có phạm vi ±1.571 (±90°). Riêng `servo_base` là góc **trục servo**; qua hộp số 2:1, khớp đế chỉ quay một nửa góc này.
- `servo_gripper`: 0 là kẹp bút; −1.117 rad là mở hết (ngón mở 40°).

Thao tác chuột/phím (theo tài liệu MuJoCo "simulate"):
- `Space`: chạy/tạm dừng.
- Nhấp đúp vào một chi tiết: chọn chi tiết đó.
- Giữ `Ctrl` + kéo **chuột trái**: xoay chi tiết đã chọn (tác dụng mô-men).
- Giữ `Ctrl` + kéo **chuột phải**: đẩy chi tiết trong mặt phẳng đứng (tác dụng lực); thêm `Shift` để đẩy trong mặt phẳng ngang.

Đóng cửa sổ để thoát. Trong viewer này **không có vệt mực**, vì vệt mực do `run_sim.py` vẽ (bước 8).

## 7. Bước kiểm chứng động học (mục "Mô phỏng" trong đề cương)

```bash
python check_kinematics.py
```

Script chạy khoảng 45 giây trên máy thử (phần [4] quét vùng làm việc nên lâu nhất). Kết quả mong đợi với tham số mặc định:

```
CANH BAO: PEN_LENGTH, TOOL_MASS_G va gioi han khop trong params.py la gia tri tam [GIA-DINH]. Hay do tren tay may that roi sua lai.
[1] FK(IK(p)) tren 5000 diem: max |FK(IK(p))-p| = 1.30e-13 mm, max |IK(FK(q))-q| = 9.95e-13 mrad
[2] MuJoCo (vong kin, g=0) vs FK giai tich, 40 tu the: max sai lech = 0.015 mm, max vi pham rang buoc vong = 0.0000 mm
[3] Do vong do trong luc (servo P huu han, params.SERVO_ERR_SAT_DEG):
    q=(0, 90, 0): lech goc (deg) = [ 0.   -0.02 -0.55], lech dau but = 1.68 mm, ...
    q=(0, 60, -30): lech goc (deg) = [ 0.   -0.68 -0.49], lech dau but = 2.13 mm, ...
    q=(0, 45, -60): lech goc (deg) = [ 0.   -0.95 -0.3 ], lech dau but = 2.27 mm, ...
    q=(0, 110, 0): lech goc (deg) = [-0.    0.44 -0.55], lech dau but = 2.56 mm, ...
[4] Hinh ve lon nhat (sao + tron ban kinh R) theo do cao mat giay z (mm), cach gioi han >= 3 deg:
    z_giay =   1.0 mm: tam x = 245 mm, R_max =  56.4 mm   <- mat giay (params.PAPER_THICKNESS)
    z_giay =  10.0 mm: tam x = 250 mm, R_max =  62.4 mm
    ...
    z_giay = 100.0 mm: tam x = 240 mm, R_max =  68.8 mm
Da luu .../results/workspace_rz.png
```

Dòng `CANH BAO` là nhắc nhở bình thường, không phải lỗi. Phần [2] và [3] tắt va chạm ngòi–giấy để so sánh động học thuần.

✅ **Kiểm tra — PASS khi:**

| Dòng | Tiêu chí | Ý nghĩa nếu FAIL |
|---|---|---|
| [1] | cả hai sai số < 1e-9 | Công thức IK không còn là nghịch đảo của FK (đã sửa nhầm `mk2_kinematics.py`?) |
| [2] | sai lệch < 0.05 mm **và** vi phạm vòng ≈ 0 | Lớp ánh xạ hình bình hành hoặc `model/mk2.xml` không khớp `params.py` → chạy lại bước 5 |
| [3] | độ võng đầu bút cỡ 1,5–2,6 mm | Lớn hơn nhiều: `SERVO_ERR_SAT_DEG` hoặc khối lượng bị đổi |
| [4] | dòng `<- mat giay` có `R_max` > 40 mm, có file `results/workspace_rz.png` | `R=40` mặc định ở bước 8 sẽ vượt vùng làm việc |

Ý nghĩa từng dòng:
1. Công thức IK là nghịch đảo đúng của FK. FK/IK đã tính cả độ lệch ngang 0,475 mm của ngòi so với mặt phẳng cánh tay; độ lệch này do kẹp lắp lệch tâm, tính từ STL.
2. Mô hình vật lý có vòng kín cho cùng vị trí đầu bút với công thức giải tích. Điều này chứng minh **lớp ánh xạ hình bình hành** là đúng.
3. Servo là bộ điều khiển P hữu hạn, nên dưới trọng lực đầu bút võng xuống khoảng 1,5–2,6 mm. Đây là hiệu ứng thật mà mô hình động học thuần không thấy được.
4. Bảng để **chọn R và tâm hình vẽ**. Dòng đánh dấu là mặt tờ giấy thật (dày 1 mm, nằm trên bàn). Chọn R nhỏ hơn hẳn R_max để còn khoảng dự phòng.

> Vì sao ngòi thò 90 mm: với ngòi cách điểm kẹp 60 mm, chạy lại bảng [4] (sửa `PEN_GRIP_TO_TIP` rồi build lại) cho mặt giấy z = 1 mm chỉ được **R_max ≈ 10,5 mm**. Nguyên nhân là cổ tay phải hạ thấp hơn trục vai, trong khi kiểm tra va chạm STL cho thấy tay ngang 006 đâm vào cổ tay 009 khi chúc xuống quá 67,5°. Ngòi thò dài thêm k mm có tác dụng tương đương kê giấy cao thêm k mm.

## 8. Chạy thí nghiệm M0 vs M2

```bash
python run_sim.py                    # mặc định: R = 40 mm, tâm (245, 0) = tâm tờ A4, ngòi vẽ ở z = 1 mm (mặt giấy)
```

Trình tự mỗi lần vẽ, mỗi phương pháp coi như vẽ trên một tờ giấy mới:
1. Đưa bút tới vị trí cách điểm đầu 15 mm theo phương đứng (`PEN_LIFT`).
2. Hạ bút thẳng xuống trong 1 s, dừng 0,3 s.
3. Vẽ một nét liền (M0 hoặc M2).
4. Dừng 0,3 s, nhấc bút lên trong 0,5 s.

**Mực chỉ được ghi khi MuJoCo báo ngòi chạm giấy**, tức khe ngòi–giấy ≤ `INK_GAP` = 0,05 mm. Nét rộng 1 mm. Các chỉ số đo trên **vết mực**, giống cách đo trên giấy thật.

Kết quả mong đợi với tham số mặc định (chạy khoảng 15 giây):

```
Duong di L = 541.9 mm (ly thuyet 13.55R = 542.0); T(M2) = 5.44 s; v(M0) = L/T = 99.6 mm/s
[M0] toc do servo lon nhat = [0.97 0.51 0.39] rad/s (MG996R khong tai 5.51 rad/s @4.8V) -> OK
[M2] toc do servo lon nhat = [1.46 0.76 0.56] rad/s (MG996R khong tai 5.51 rad/s @4.8V) -> OK
[M0] max vi pham rang buoc vong = 0.069 mm, max |mo-men servo| = [0.082 0.17  0.095] Nm
[M0] ngoi cham giay 100.0% thoi gian ve; so net muc = 1; luc ep ngoi TB/max = 2.48/3.69 N
[M2] max vi pham rang buoc vong = 0.069 mm, max |mo-men servo| = [0.117 0.171 0.095] Nm
[M2] ngoi cham giay 100.0% thoi gian ve; so net muc = 1; luc ep ngoi TB/max = 2.46/4.52 N

Chi so du doan tu mo phong (cung T), do tren VET MUC: 5 mui sao / duong tron / 10 canh:
PP    vot lo TB  vot lo max  sai lech dinh TB  do tron  lech thang max  co muc %   (mm)
M0        -1.03       -0.52              1.08     0.76            1.30     100.0
M2        -0.38        0.13              0.44     0.40            1.19     100.0
```

✅ **Kiểm tra:**

```bash
ls results/cmd_derivatives.png results/pen_trace.png results/log_M0.csv results/log_M2.csv results/ink_M0.png results/ink_M2.png
wc -l results/log_M0.csv results/log_M2.csv          # -> 403 dòng mỗi file (1 dòng tiêu đề + 402 mẫu)
md5sum results/cmd_derivatives.png assets/simulation_results/cmd_derivatives.png
md5sum results/pen_trace.png       assets/simulation_results/pen_trace.png
```

PASS khi:
- cả hai dòng tốc độ servo kết thúc bằng `-> OK`;
- vi phạm ràng buộc vòng < 0.1 mm;
- **ngòi chạm giấy 100% thời gian vẽ, mỗi lần chỉ 1 nét mực** (nét liền, không đứt);
- đủ 6 file, mỗi CSV 403 dòng;
- bảng chỉ số trùng bảng trên (với tham số mặc định).

Hai lệnh `md5sum` là phép so khắt khe nhất: nếu mỗi cặp cho cùng một mã, ảnh của bạn trùng từng byte với ảnh mẫu.
Nếu mã khác nhưng bảng số liệu vẫn trùng, mở hai ảnh ra so bằng mắt; khác biệt nhỏ ở mức điểm ảnh có thể do khác CPU hoặc phông chữ, chưa phải lỗi.
Ảnh `ink_*.png` dựng bằng OpenGL nên điểm ảnh phụ thuộc GPU; chỉ so bằng mắt với `assets/simulation_results/ink_M2.png`.

Các tùy chọn khác:

```bash
python run_sim.py --view             # xem robot vẽ, vệt mực hiện dần trên giấy (cần bước 6 PASS)
python run_sim.py --R 30 --vmax 150 --amax 1500 --jmax 30000
```

Với `--view`:
- Camera tự hướng vào tờ giấy. Mỗi phương pháp bắt đầu trên tờ giấy trắng: M0 vẽ màu đỏ, M2 màu xanh.
- Viewer chạy ở chế độ "passive", nên phím `Space` (tạm dừng) không có tác dụng (theo tài liệu MuJoCo).
- Script tự đóng cửa sổ khi chạy xong.

Với `--R 30`, bảng chỉ số mong đợi là `M0 -0.99 -0.43 1.04 0.92 1.30 100.0` và `M2 -0.40 0.07 0.43 0.39 1.15 100.0`.

Các file trong `results/`:

| File | Nội dung |
|---|---|
| `cmd_derivatives.png` | **Bằng chứng 1**: q̇, q̈, jerk của lệnh M0 và M2 (phần vẽ) chồng lên nhau |
| `pen_trace.png` | **Vết mực** M0, M2 (chỉ chỗ ngòi chạm giấy) so với hình lý tưởng, kèm ảnh phóng to mũi sao |
| `ink_M0.png`, `ink_M2.png` | Ảnh 3D tay máy và tờ giấy có vết mực sau khi vẽ xong |
| `log_M0.csv`, `log_M2.csv` | Lệnh 20 ms, pha (0 hạ bút, 1 vẽ, 2 nhấc bút), góc servo, vị trí ngòi, góc thực, mô-men servo, có mực (0/1), lực ép ngòi (N) |

Cách đọc các chỉ số (đo giống cách đo trên giấy trong đề cương):
- **Vọt lố**: đo so với **giao điểm hai cạnh vẽ thật kéo dài**. Giá trị **âm** nghĩa là nét vẽ **cắt góc**, không chạm tới giao điểm.
- **Sai lệch đỉnh**: khoảng cách ngắn nhất từ nét vẽ tới giao điểm. Chỉ số này bắt được cả trường hợp vọt lố lẫn cắt góc.
- **Độ tròn**: D_max − D_min theo 4 hướng 0°, 45°, 90°, 135°.
- **Lệch thẳng**: khe hở lớn nhất trên đoạn 15–85% của mỗi cạnh.
- **Có mực %**: phần thời gian vẽ mà ngòi chạm giấy. Nhỏ hơn 100% nghĩa là nét bị đứt.

> Phát hiện từ mô phỏng (cần kiểm chứng trên phần cứng):
> - M0 **cắt góc** khoảng 1 mm chứ không vọt lố, nên chỉ số "vọt lố" trong đề cương có thể ra âm với M0. Nên đo thêm "sai lệch đỉnh".
> - Bút được kẹp cứng, lệnh ngòi đặt đúng mặt giấy; độ võng do trọng lực (bước 7, dòng [3]) ép ngòi xuống giấy với lực trung bình khoảng 2,5 N.
> - Kết quả nhạy với ma sát ngòi–giấy (`PEN_TIP_FRICTION`, đang giả định 0,2). Thử với μ ≈ 0, lực ép giảm còn ~1,3 N và vọt lố TB của M2 đổi từ −0,38 thành 0,00 mm. Cần đo hệ số này trên bút thật.
> - "Lệch thẳng max" cỡ 1,2–1,3 mm ở cả M0 và M2. Thử tắt ma sát hoặc tắt trọng lực, chỉ số này vẫn còn, nên nó không do hai yếu tố đó gây ra. Nguyên nhân chưa được phân tích.

Lưu ý: M2 trong gói này dùng **Ruckig** làm chuẩn tham chiếu, như đề cương mô tả. Bộ double-S tự viết của bạn (để chạy trên ESP32) có thể đưa vào thay hàm `m2_path` trong `trajectories.py`, rồi so sánh với Ruckig.

## 9. Chụp ảnh tư thế cho báo cáo

Điều kiện: bước 2.2 PASS.

```bash
python snapshot.py --xyz 245 0 20 --out results/pose.png     # đặt ngòi bút tại (245, 0, 20) mm bằng IK
python snapshot.py --servo 90 90 90 --out results/home.png   # đặt góc 3 servo trực tiếp (độ)
```

Kết quả mong đợi của lệnh thứ nhất:

```
servo (deg): [89.78 70.78 45.12] | FK dau but (mm): [245.   0.  20.]
MuJoCo dau but (mm): [245.07  -0.    18.7 ]
Da luu results/pose.png
```

✅ **Kiểm tra — PASS khi:** có file `results/pose.png` và ảnh giống `assets/simulation_results/pose_xyz_245_0_20.png`.
- Chênh lệch khoảng 1,3 mm giữa FK và MuJoCo là độ võng do trọng lực, cùng bản chất với dòng [3] ở bước 7, không phải lỗi.
- Góc servo đế là 89,78° chứ không phải 90° vì IK bù độ lệch ngang 0,475 mm của ngòi.

## 10. Hiệu chỉnh cho giống thực tế (quan trọng nhất)

Các tham số đánh dấu `[GIA-DINH]` trong `params.py` là giá trị tạm. Các tham số `[DO-STL]` đã tính từ STL nhưng vẫn nên đo lại trên tay máy thật. Hãy đo theo thứ tự sau, sửa `params.py`, rồi chạy lại `build_model.py`:

| # | Tham số | Cách đo |
|---|---|---|
| 1 | `PEN_GRIP_TO_TIP`, `PEN_LENGTH`, `TOOL_MASS_G` | Đo từ giữa má kẹp tới ngòi (đang đặt 90 mm), chiều dài bút, cân bút. `TOOL_OFFSET` tự tính lại từ `GRIP_POINT_W` (đo từ STL) và `PEN_GRIP_TO_TIP` |
| 2 | `H_SHOULDER` (qua `LOWER_BASE_Z`) | Thước kẹp: từ mặt bàn tới tâm trục vai. Giá trị 94,25 mm suy từ STL, với giả định gearmast tì lên mặt tâm của 013 (bi Ø6 còn khe ~0,6 mm) |
| 3 | `SERVO_OFFSET`, `SERVO_SIGN` | Ra lệnh 90/90/90, kiểm tra tay chính thẳng đứng, tay ngang nằm ngang, khớp đế nhìn thẳng; ghi độ lệch và chiều quay |
| 4 | `THETA2_LIM`, `PHI_LIM`, `REL_LIM`, `THETA1_LIM` | Đang lấy từ kiểm tra va chạm STL (`tools/check_assembly.py --limits`), cách biên va chạm ≥ 2,5°. Trên tay thật: quay từ từ từng servo tới khi chạm cơ khí, lùi lại 3–5° |
| 5 | `MEASURED_MASS_G` | Cân từng chi tiết sau khi in, điền vào dict. Mặc định đang giả định in đặc 100% (cận trên) |
| 6 | `SERVO_ERR_SAT_DEG` | Ra lệnh bước 30° và quay video 240 fps (hoặc đọc chiết áp bên trong servo, theo tài liệu Adafruit [12] trong đề cương); chỉnh tham số tới khi đáp ứng mô phỏng khớp với đo |
| 7 | `BACKLASH_DEG` | Giữ cố định servo, lắc nhẹ đầu tay và đo độ dơ ở đầu bút; chia cho cánh tay đòn để ra độ dơ góc |
| 8 | `BASE_GEAR_RATIO` | Đếm răng: gearmast 50 / gearservo 25 = 2,0 (bản `22DENTI` thì 50/22) |
| 9 | `PEN_TIP_FRICTION`, `INK_GAP` | Ma sát ngòi–giấy và khe tối đa vẫn ra mực; ảnh hưởng tới lệch thẳng và chỗ nét đứt |

✅ **Kiểm tra sau mỗi lần sửa `params.py`** (theo đúng thứ tự):

```bash
python build_model.py --stl-dir "$STL_DIR"   # các dòng "lech" vẫn phải <= 0.05 mm, ngoi but STL ~ FK
python check_kinematics.py                    # [1] và [2] vẫn phải PASS như bước 7
python run_sim.py                             # đọc lại bảng R_max ở bước 7 nếu gặp IKError
```

Sau khi hiệu chỉnh, các số "Kết quả mong đợi" ở bước 5–9 **sẽ thay đổi**; đó là điều bình thường.
Riêng tiêu chí [1] và [2] ở bước 7 không phụ thuộc tham số và phải luôn PASS.

Cách kiểm tra mô hình đã sát thực tế: chạy cùng một quỹ đạo trên tay máy thật, rồi so ảnh tờ vẽ với `pen_trace.png` và so 3 chỉ số đo.

## 11. Giới hạn của mô hình (ghi rõ trong báo cáo)

- Các chi tiết in là **vật rắn tuyệt đối**: không mô hình hóa độ đàn hồi của PLA. Rung do uốn khâu sẽ không xuất hiện.
- Servo được mô hình là bộ P bão hòa mô-men cộng đường đặc tính mô-men–tốc độ của động cơ DC (đặt bằng `damping`). Bộ điều khiển thật bên trong MG996R/SG90 không được công bố. Không mô hình vùng chết và độ phân giải xung.
- **Va chạm**: trong lúc mô phỏng chỉ có va chạm ngòi–giấy. Va chạm giữa các khâu được kiểm tra ngoài (bằng `tools/check_assembly.py`) và được loại trừ bằng giới hạn khớp.
- **Bút gắn cứng vào kẹp.** Ngón kẹp được đặt đúng góc kẹp Ø10 nhưng không mô phỏng lực kẹp, ma sát hay trượt bút trong kẹp. Không có lò xo ở bút. Tờ giấy là vật rắn, không mô phỏng độ lún của giấy.
- Servo (MG996R ×3, SG90) không có trong STL. Chúng được vẽ bằng hộp kích thước datasheet, đặt đúng trục quay; vị trí dọc trục chỉ gần đúng.
- Khối lượng ốc vít, thanh ren, bi và bạc đạn 606zz không được tính.
- Kẹp nghiêng 0,27° (ngòi lệch ~0,4 mm trên 90 mm). Đây là kết quả của tọa độ lỗ trong STL, được giữ nguyên.

## 12. Port sang ESP32

`mk2_kinematics.py` chỉ dùng `math`, nên có thể chép nguyên công thức sang C/C++ (dùng `float`). Các hàm cần port:

- `ik()`: atan2, acos, chọn nghiệm khuỷu trên, **bù độ lệch ngang `DY` của ngòi** (`th1 = atan2(y, x) − atan2(DY, r)`).
- `to_servo()`: lớp ánh xạ với offset, chiều quay và hộp số 2:1.
- `within_limits()`: kiểm tra giới hạn θ1, θ2, φ, φ − θ2 và tầm servo trước khi gửi lệnh.

Để đối chiếu float (ESP32) với double (PC) như đề cương yêu cầu, hãy so cột `servo*_deg` trong `results/log_M2.csv` với log UART của ESP32 cho cùng một quỹ đạo.

## 13. Xử lý lỗi thường gặp

| Lỗi / hiện tượng | Nguyên nhân | Cách xử lý |
|---|---|---|
| `ModuleNotFoundError: No module named 'mujoco'` | Chưa kích hoạt venv | `source ~/venvs/mk2/bin/activate`, kiểm tra `which python` (bước 3.3) |
| `No matching distribution found for mujoco==3.2.3` | Python không phải 3.8, hoặc không phải x86_64 | Quay lại bước 2.1 |
| `Khong tim thay .../EBAmk2_xxx.STL` | `--stl-dir` sai thư mục, thiếu file kẹp, hoặc đuôi file là `.stl` chữ thường | Làm lại phần tìm `STL_DIR` và lệnh kiểm tra 19 file ở bước 4 |
| `RuntimeError: Hinh hoc khong khop` | `params.py` bị sửa sai phần hình học, hoặc STL khác bản gốc | Khôi phục `params.py` (`git diff params.py`), dùng STL từ bước 4 |
| `params.GRIP_POINT_W/CLAW_TILT_DEG khong khop hinh hoc STL` | Đã sửa hình học kẹp hoặc `PEN_DIAMETER` | Chép 2 dòng script in ra vào `params.py`, chạy lại bước 5 |
| `Chua co model/mk2.xml` | Chưa chạy bước 5 | Chạy `build_model.py` |
| `Hinh ve (...) nam ngoai to giay` | `--cx`/`--cy`/`--R` làm hình vẽ tràn khỏi tờ A4 | Giảm `--R`, hoặc sửa `PAPER_CENTER` rồi build lại |
| `IKError: th2=... ngoai`, `phi=... ngoai`, `phi-th2=... ngoai`, `diem ngoai tam voi` | Hình vẽ vượt vùng làm việc hoặc giới hạn khớp | Giảm `--R` hoặc đổi `--cx` theo bảng `R_max` ở bước 7 |
| `ngoi cham giay < 100%` hoặc `so net muc > 1` | Ngòi nhấc khỏi giấy giữa chừng (nét đứt) | Kiểm tra `--z` (phải bằng mặt giấy 1 mm), giảm `--vmax/--amax` |
| `-> VUOT gioi han, can gian thoi gian` | Quỹ đạo đòi tốc độ servo lớn hơn MG996R làm được. Đây là **cảnh báo**, mô phỏng vẫn chạy nhưng kết quả không thực tế | Giảm `--vmax`/`--amax`/`--jmax` |
| `WARNING: Nan, Inf or huge value in QACC` | Mô phỏng mất ổn định (thường do đặt `GRIPPER_ARMATURE` = 0) | Khôi phục `GRIPPER_ARMATURE = 1e-6`, build lại |
| Viewer không mở / lỗi GLFW / rất chậm | Driver đồ họa đang dùng `llvmpipe` hoặc không hoạt động | Làm lại bước 2.2. Không cần cài `libglfw3` qua apt vì gói `glfw` của pip đã kèm sẵn thư viện |
| `khong chup duoc anh ink_M0.png` hoặc lỗi `EGL`/`OSMesa` | Không có OpenGL, hoặc biến `MUJOCO_GL` đang được đặt | `unset MUJOCO_GL` rồi thử lại. Các file khác vẫn được ghi bình thường |
| Viewer chạy giật | Lưới STL đầy đủ khá nặng | Tắt hiển thị bóng trong panel Rendering, hoặc thử chạy trên NVIDIA (bước 2.2, tùy chọn) |

## 14. Nguồn

- STL và BOM: daGHIZmo, *EEZYbotARM Mk2*, Thingiverse thing:1454048 (CC BY-NC), https://www.thingiverse.com/thing:1454048. Bản vẽ khối gá kẹp `EBAmk2_014_claw_std.pdf` nằm trong cùng bộ file. Kích thước đo bằng `tools/holes.py`.
  Bản sao STL bên thứ ba dùng để kiểm chứng: https://github.com/mchora2/EEZYbotARMMK2
- MG996R: TowerPro, https://towerpro.com.tw/product/mg996r/ (9,4 kgf·cm và 0,19 s/60° ở 4,8 V; 11 kgf·cm và 0,15 s/60° ở 6 V; 55 g; 40,7 × 19,7 × 42,9 mm). Trang này ghi 0,17 s ở một bảng khác; gói này dùng 0,19 s như đề cương.
- SG90: TowerPro SG90 datasheet (bản lưu tại https://tams.informatik.uni-hamburg.de/lehre/2023ss/vorlesung/es/doc/sg90.pdf): 1,8 kgf·cm, 0,1 s/60°, 9 g, ~22,2 × 11,8 × 31 mm.
- Khổ giấy A4 210 × 297 mm: ISO 216.
- Mật độ PLA 1,24 g/cm³: NatureWorks, *Ingeo 4043D Technical Data Sheet* (ASTM D792).
- MuJoCo: XML Reference (`equality/connect`, `equality/joint`, `actuator/position`, `armature`, `margin`/`gap` của geom, `contype`/`conaffinity`) và mục Backlash trong Modeling, https://mujoco.readthedocs.io
- MuJoCo Python bindings (lệnh `python -m mujoco.viewer --mjcf=...`, `launch_passive`, `user_scn`): https://mujoco.readthedocs.io/en/stable/python.html
- Phím tắt viewer: MuJoCo "Code samples → simulate", https://mujoco.readthedocs.io/en/stable/programming/samples.html
- Phiên bản Python hỗ trợ của từng gói: trang PyPI của gói (ví dụ https://pypi.org/project/mujoco/3.2.3/, https://pypi.org/project/ruckig/0.12.2/), mục "Download files" và "Requires: Python".
- NVIDIA PRIME Render Offload: NVIDIA Linux driver README, https://download.nvidia.com/XFree86/Linux-x86_64/470.82.00/README/primerenderoffload.html
- Ruckig: Berscheid & Kröger, RSS 2021 (tài liệu [2] trong đề cương).
