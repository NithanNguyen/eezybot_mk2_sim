# Mô phỏng EEZYbotARM Mk2 — Python + MuJoCo

### FAST RUN
```bash
source ~/venvs/mk2/bin/activate
cd ~/eezybot_mk2_sim

python -m mujoco.viewer --mjcf=model/mk2.xml       # tay máy + gripper thật (KHÔNG bút); kéo servo_gripper để đóng/mở
python -m mujoco.viewer --mjcf=model/mk2_pen.xml   # có bút + giấy (panel Simulation -> Load key để 2 ngón kẹp bút)
python -m mujoco.viewer --mjcf=model/mk2_sort.xml  # cảnh phân loại: kệ + GY-33, hộp 3 ngăn, vùng vứt bỏ, nắp chai
python run_sort.py --view        # xem chu trình phân loại 9 nắp (mục 9b)
python run_sort.py               # chạy không cửa sổ, ghi kết quả vào results/sort/ (khoảng 2 phút)
python run_sort.py --foreign 2   # thêm 2 vật lạ để thử nhánh "vứt bỏ"
python run_sim.py --pen --view   # xem tay máy cầm bút vẽ, vệt mực hiện dần trên giấy
python run_sim.py --pen          # chạy không cửa sổ, ghi kết quả vào results/
python check_kinematics.py       # kiểm chứng động học cho TCP gripper (khoảng 6 giây)
python check_kinematics.py --pen # kiểm chứng cho ngòi bút + bảng kích thước hình vẽ khả thi (khoảng 21 giây)
```

Bộ mã mô phỏng cho đồ án CE212 "Tay máy robot 3-DOF phân loại sản phẩm". Gói này gồm:

- Mô hình MuJoCo của Mk2: cánh tay dựng từ **13 file STL gốc**, có **3 vòng hình bình hành kín**. Mọi chi tiết đặt theo tâm lỗ trục và mặt tiếp giáp đo trên STL.
- **Gripper thật** trên robot (thiết kế riêng, không có file CAD). Mô hình được dựng lại từ số đo thước kẹp và biên dạng trích từ ảnh chụp (`gripper.py`, `gripper_shape.py`).
  - Cơ cấu 2 hình bình hành, ngón **tịnh tiến**, 2 bánh răng 1:1, trục SG90 trùng trục bánh răng trái.
  - Gripper thay cả khối cổ tay `009` và kẹp gốc `014–018`.
- **Mặc định không có bút** (bài gắp vật). Cờ `--pen` gắn **bút Ø10 thẳng đứng giữa 2 ngón** (trục bút lùi 27 mm so với mút ngón, ngòi cách điểm kẹp 90 mm) và **tờ giấy A4 dày 1 mm** để thử vẽ.
  - **Vệt mực rộng 1 mm** chỉ được ghi khi MuJoCo báo ngòi chạm giấy, và hiện trong viewer.
- Kiểm tra lắp ráp bằng `tools/check_assembly.py`: không chi tiết nào lồng vào nhau trong giới hạn khớp, và gripper không tự va chạm khi đóng/mở 0–90°.
- Mô hình servo MG996R (đế, vai, khuỷu) và SG90 (gripper). Hộp servo vẽ theo kích thước datasheet.
- Thư viện FK/IK giải tích kèm lớp ánh xạ góc servo. Đầu công tác chọn được: TCP gripper (mặc định) hoặc ngòi bút.
- Thí nghiệm M0/M2 vẽ ngôi sao + đường tròn, đúng như mô tả trong đề cương; chỉ số đo trên vết mực.
- **Chu trình phân loại màu nắp chai** (mục 9b): kệ trạm cảm biến có lỗ Ø23, module GY-33, hộp đích 3 ngăn, vùng vứt bỏ, 9 nắp chai Ø30 × 13 mm.
  - Gripper kẹp nắp bằng **tiếp xúc và ma sát thật**.
  - Màu được phân loại bằng **đúng thuật toán và kết quả hiệu chuẩn của firmware**: `data/color/model.json`, chép từ repo `eezybot_mk2_robot`.

> **Trạng thái kiểm chứng (2026-10-16, bản gripper thật + cờ `--pen`).** Toàn bộ quy trình bước 5 → 9 đã được chạy lại từ đầu
> trong venv **Python 3.8.20** với đúng các phiên bản ghim trong `requirements.txt`, bộ STL `EBAmk2_*.STL` (xem bước 4).
> Mọi số liệu "Kết quả mong đợi" bên dưới là output thật của lần chạy đó.
> 2 đồ thị `cmd_derivatives.png` và `pen_trace.png` trong `assets/simulation_results/` trùng **từng byte** (md5) với ảnh vừa sinh ra.
> `run_sim.py --pen` chạy 2 lần liên tiếp cho cùng md5 và cùng `log_M2.csv` (kết quả tất định).
>
> Lần chạy này **không** làm trên Ubuntu 20.04 thật mà trên Linux x86_64 khác. Khả năng tương thích với Ubuntu 20.04 được suy ra
> từ thẻ wheel (mọi gói là `cp38 manylinux` cần glibc ≤ 2.28, Ubuntu 20.04 có glibc 2.31). Bước 2 giúp bạn tự xác nhận điều này trên máy mình.
> Ảnh OpenGL được dựng trên màn hình ảo (Xvfb); cửa sổ viewer chưa được thử với driver NVIDIA thật.
>
> **Bổ sung 2026-10-23 (bài phân loại; `CTRL_ELBOW_DEG` đổi thành −62,5°).** Các bước 5, 7, 8, 9 (2 lệnh đầu) và 9b được chạy lại
> trong cùng môi trường (Python 3.8.20, đúng các phiên bản ghim, bộ STL `EBAmk2_*.STL`). Các số "Kết quả mong đợi" đã cập nhật theo lần chạy này.
> - `mk2.xml` và `mk2_pen.xml` chỉ khác bản trước ở `ctrlrange` của `servo_elbow`.
> - `run_sim.py --pen` vẫn cho 2 đồ thị trùng **từng byte** với ảnh mẫu.
> - Chỉ bảng R_max của `check_kinematics.py --pen` (bước 7) thay đổi.
> - Viewer `--view` của `run_sort.py` mới được thử trên màn hình ảo (Xvfb).

---

## Cách đọc hướng dẫn này

Mỗi bước có một khối **✅ Kiểm tra** gồm lệnh, kết quả mong đợi và tiêu chí **PASS**.
**Chỉ chuyển sang bước tiếp theo khi bước hiện tại PASS.** Nếu FAIL, xem dòng "Nếu FAIL" ngay dưới đó hoặc bảng lỗi ở mục 13.

Quy ước:
- `~` là thư mục home của bạn.
- Mọi lệnh `python` (từ bước 3.3 trở đi) phải chạy **trong venv** (dấu nhắc có `(mk2)`).

---

## 0. Phương án đã chốt

| Lớp | Công cụ | Dùng cho |
|---|---|---|
| Động học + quỹ đạo | Python 3.8, NumPy, Ruckig 0.12.2 | FK/IK, vùng làm việc, q̇/q̈/jerk của lệnh, thời gian quỹ đạo, chọn kích thước hình vẽ |
| Động lực học | MuJoCo 3.2.3 | Hình bình hành kín của tay (`equality/connect`), bánh răng và hình bình hành gripper (`equality/joint`), quán tính từ STL, servo bão hòa mô-men, độ rơ, lệnh giữ 20 ms, tiếp xúc ngòi–giấy |

Vì sao ghim đúng các phiên bản này:
- Ubuntu 20.04 dùng Python 3.8.
- Theo PyPI, **mujoco 3.2.3** là bản cuối có wheel `cp38` (3.2.4 yêu cầu Python ≥ 3.9).
- **ruckig 0.12.2** là bản cuối có wheel `cp38`. Với Python 3.8, bản 0.14.0 và 0.15.3 chỉ có mã nguồn, pip sẽ phải tự biên dịch.
- NumPy 1.24.4, SciPy 1.10.1, Matplotlib 3.7.5, NetworkX 3.1 là các bản cuối hỗ trợ Python 3.8 (bản kế tiếp của mỗi gói yêu cầu Python ≥ 3.9).

**Không nâng cấp lẻ từng gói.**

Mô hình gripper không cần thêm thư viện nào. Nó chỉ dùng các gói trên và `contourpy`, gói mà Matplotlib 3.7.5 tự cài kèm.

## 1. Cấu trúc thư mục

```
eezybot_mk2_sim/
├── params.py            # MỌI tham số (kích thước tay, gripper, bút, servo...) — có ghi nguồn
├── mk2_kinematics.py    # FK, IK, giới hạn khớp + tầm lệnh servo, ánh xạ góc <-> servo; set_tool("gripper"/"pen")
├── gripper.py           # gripper thật: hình học, động học ngón, lưới lăng trụ, khối lượng
├── gripper_shape.py     # biên dạng ngón/đế/khối chốt trích từ ảnh (sinh bởi tools/gripper_from_photos.py)
├── build_model.py       # STL gốc + gripper.py -> model/mk2.xml (không bút), mk2_pen.xml (bút + giấy), mk2_sort.xml
├── sort_scene.py        # đồ vật bài phân loại (kệ, GY-33, hộp 3 ngăn, vùng vứt bỏ, nắp) + tấm kẹp có va chạm của ngón
├── color_sim.py         # bộ phân loại màu (bản sao ColorMath.h của firmware) + cảm biến mô phỏng (phát lại test.csv)
├── sim_common.py        # nạp mô hình (pen / sort), giữ lệnh 20 ms, đọc góc thực
├── trajectories.py      # hình sao + tròn, M0 (tuyến tính), M2 (double-S/Ruckig)
├── check_kinematics.py  # kiểm chứng FK/IK, so MuJoCo vs giải tích, vùng làm việc  [--pen]
├── run_sim.py           # thí nghiệm vẽ M0 vs M2, đồ thị + CSV + chỉ số  (bắt buộc --pen)
├── run_sort.py          # chu trình phân loại màu nắp chai, CSV + ma trận + đồ thị  [--view] [--foreign N] [--seed]
├── snapshot.py          # chụp ảnh tư thế cho báo cáo  [--pen] [--grip]
├── data/color/          # model.json (kết quả hiệu chuẩn) + test.csv (tập kiểm tra) — chép từ repo eezybot_mk2_robot
├── tools/holes.py       # đo tâm lỗ trục trên STL (nguồn số liệu hình học)
├── tools/check_assembly.py      # kiểm chứng lắp ráp + gripper tự va chạm; suy ra giới hạn khớp
├── tools/check_sort_layout.py   # tính lại các số [TINH] của bài phân loại (tầm với, vị trí nắp trong gripper)
├── tools/gripper_from_photos.py # ảnh chụp gripper -> gripper_shape.py (ảnh KHÔNG có trong repo)
├── assets/simulation_results/   # ảnh kết quả mẫu để đối chiếu
├── requirements.txt
├── model/               # mk2.xml, mk2_pen.xml, mk2_sort.xml, meshes/ — repo có kèm bản đã dựng, bước 5 ghi đè
└── results/             # kết quả bước 7–9b — repo có kèm bản mẫu, các bước đó ghi đè (bài phân loại: results/sort/)
```

## 2. Kiểm tra máy trước khi cài

### 2.1 Hệ điều hành, glibc, Python

```bash
lsb_release -ds            # hệ điều hành
uname -m                   # kiến trúc CPU
ldd --version | head -1    # phiên bản glibc
python3 --version          # Python hệ thống
```

✅ **Kiểm tra — PASS khi:**
- Dòng 1 là `Ubuntu 20.04.x LTS`.
- Dòng 2 là `x86_64`.
- glibc **≥ 2.28** (Ubuntu 20.04 là `2.31`).
- Python là `Python 3.8.x` (x bất kỳ).

Nếu FAIL:
- Python sai (ví dụ 3.10, 3.12): các phiên bản ghim trong `requirements.txt` sẽ **không cài được**. Dừng lại, không sửa `requirements.txt` theo cảm tính.
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

Nếu FAIL:
- `llvmpipe` nghĩa là đang vẽ bằng CPU (phần mềm).
- Bước 3, 5, 7, 8 (không có `--view`) **vẫn chạy được** vì không cần OpenGL; khi đó chỉ thiếu ảnh `ink_*.png`.
- Viewer sẽ rất chậm hoặc không mở. Hãy sửa driver trước khi tới bước 6.

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

Nếu đã clone từ trước, cập nhật bằng `cd ~/eezybot_mk2_sim && git pull`.

✅ **Kiểm tra:**

```bash
ls ~/eezybot_mk2_sim/requirements.txt ~/eezybot_mk2_sim/build_model.py ~/eezybot_mk2_sim/gripper.py
```

PASS khi cả ba đường dẫn được in ra, không có `No such file or directory`.

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
import mujoco, ruckig, trimesh, scipy, matplotlib, PIL, glfw, contourpy
print("PASS" if not bad else f"FAIL: sai phien ban {bad}")
EOF
```

Kết quả mong đợi: 7 dòng bắt đầu bằng `OK`, dòng cuối `PASS`.

Nếu FAIL với `No matching distribution found for mujoco==3.2.3`: Python không phải 3.8 hoặc máy không phải x86_64 → quay lại bước 2.1.

## 4. Lấy bộ STL gốc

`build_model.py` cần **13 file** sau, tên viết **đúng chữ hoa/thường** (Linux phân biệt `.STL` và `.stl`):

```
EBAmk2_001_base  002_mainarm  003_varm  004_link135  005_link135angled  006_horarm__
EBAmk2_007_trialink  008_link147_new  010_gearservo  011_gearmast  012_mainbase  013_lower_base  019_drive_cover
```

Các file **không còn dùng**:
- `009_trialinkfront` và kẹp gốc `014`–`018`: gripper thật thay cả khối này. Gripper được dựng bằng `gripper.py`, không cần STL.
- `006_horarm_plate`, `010_gearservo_22DENTI`, `011_gearmast_full`: biến thể thay thế.

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

✅ **Kiểm tra — đủ 13 file:**

```bash
miss=0; for f in 001_base 002_mainarm 003_varm 004_link135 005_link135angled 006_horarm__ \
  007_trialink 008_link147_new 010_gearservo 011_gearmast 012_mainbase 013_lower_base 019_drive_cover; do
  if [ -f "$STL_DIR/EBAmk2_$f.STL" ]; then echo "OK    EBAmk2_$f.STL"; else echo "THIEU EBAmk2_$f.STL"; miss=1; fi
done; [ $miss -eq 0 ] && echo PASS || echo FAIL
```

PASS khi có 13 dòng `OK` và dòng cuối `PASS`.

Lưu ý:
- Nếu `find` chỉ thấy tên đuôi `.stl` (chữ thường), script sẽ báo `THIEU`; khi đó đổi tên file sang `.STL` hoặc dùng cách B.
- Biến `STL_DIR` chỉ tồn tại trong terminal hiện tại.

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
  gripper: Q - W = (32.60, 28.04) mm (|Q-W| = 43.00); khe 2 mui ngon: dong 0.78 mm, mo het 70.8 mm
  but D10 lui 27 mm tu mut ngon: goc servo kep = 4.29 do; khe 2 mui ngon khi kep but = 6.01 mm
  TCP - W = (132.69, 0.480, 0.50) mm; diem kep but - W = (110.59, 0.480, 0.50) mm
  nap D30 (sort): tam nap - W = (110.00, 0.480, 0.50) mm; goc servo cham nap = 21.27 do; khe nho nhat toi chi tiet khong va cham = 4.99 mm (khau tren trai); mo 40 do: ngon cach nap 8.90 mm
  TCP o home: hinh hoc [279.69   0.48 229.75]  FK [279.69   0.48 229.75]  lech 0.000 mm
  ngoi but o home (--pen): hinh hoc [257.59   0.48 139.75]  FK [257.59   0.48 139.75]  lech 0.002 mm
chi tiet | khoi luong (g) | the tich (cm3) | ghi chu
EBAmk2_001_base | 178.2 | 143.7 | luoi da va lo
...
GR_body | 26.3 | 21.2 | gripper that, hop lang tru (luoi diem 0.25 mm)
sg90 (hop datasheet) | 9.0 | - | vi tri gan dung
...
GR_gear_l | 6.2 | 5.0 | gripper that, hop lang tru (luoi diem 0.25 mm)
GR_finger_l | 8.6 | 6.9 | gripper that, hop lang tru (luoi diem 0.25 mm)
GR_uplink_l | 2.1 | 1.7 | gripper that, hop lang tru (luoi diem 0.25 mm)
...
TONG khoi luong chuyen dong: 588.7 g  (FILL_FACTOR=1.0)
  + but (chi --pen): 15.0 g [GIA-DINH]

Servo: tau_stall=0.922 N m, w0=5.51 rad/s, kp=10.563 N m/rad, b=0.1673 N m s/rad
Da ghi .../eezybot_mk2_sim/model/mk2.xml (mac dinh, khong but), .../eezybot_mk2_sim/model/mk2_pen.xml (--pen) va .../eezybot_mk2_sim/model/mk2_sort.xml (bai phan loai)
```

Ý nghĩa các dòng kiểm tra:
- **Các dòng `lech`** là phép kiểm tra hình học: lỗ thứ 3 của mỗi chi tiết phải rơi đúng khớp tính từ `params.py`. Nếu lệch quá 1 mm, script sẽ dừng.
- **Dòng `gripper`:**
  - `Q - W` là khoảng cách 2 lỗ gắn gripper (W: đầu tay ngang 006, Q: đầu thanh 008), tính từ hình học cánh tay trong STL.
  - Giá trị này trùng số đo thước kẹp của bạn (ngang 32,6 mm, cao 28 mm, thẳng 43 mm). Vì vậy gripper nằm ngang, không nghiêng.
  - Khe 2 mũi ngón: 0,78 mm khi đóng và 70,8 mm khi mở hết (= 0,78 + 2 × 35 mm chiều dài khâu).
- **Dòng `but D10`:** góc servo kẹp để mặt trong 2 ngón vừa chạm thân bút Ø10. Góc này tính từ biên dạng ngón và vị trí bút (lùi 27 mm so với mút ngón).
- **Dòng `TCP - W` / `diem kep but - W`** được so với `params.TCP_W`, `PEN_GRIP_W`, `PEN_GRIP_DEG`. Nếu lệch quá 0,02 mm hoặc 0,01°, script dừng và in giá trị đúng để chép vào `params.py`.
- **Dòng `nap D30 (sort)`** (bài phân loại, mục 9b), so với `params.CAP_GRIP_W` và `CAP_GRIP_DEG` theo cùng cách:
  - Tâm nắp Ø30 khi kẹp nằm cách trục 2 bánh răng `GRIP_CAP_GY` = 65 mm, tức 110 mm trước W.
  - Góc servo để 2 ngón vừa chạm nắp là 21,27°.
  - Khe tới các chi tiết **không** có va chạm trong mô phỏng (khâu trên, khâu dưới, bánh răng, đế) phải > 0. Nếu không, script dừng.
- **Dòng `TCP o home` và `ngoi but o home`:** vị trí tính từ hình học phải trùng FK giải tích của `mk2_kinematics.py`, sai lệch < 0,05 mm.
- **Khối lượng `GR_*`** tính trên khối lăng trụ của gripper (lưới điểm 0,25 mm, PLA 1,24 g/cm³, đặc 100%). Khối lượng bút chỉ cộng vào mô hình `--pen`.

✅ **Kiểm tra — góc hiệu dụng (tầm lệnh) của các servo:**

```bash
python -c "import mujoco,numpy as np; m=mujoco.MjModel.from_xml_path('model/mk2.xml'); [print(f'{m.actuator(i).name:15s}', np.round(m.actuator_ctrlrange[i],4), 'rad =', np.round(np.degrees(m.actuator_ctrlrange[i]),1), 'deg') for i in range(m.nu)]"
```

Kết quả mong đợi:

```
servo_base      [-1.5708  1.5708] rad = [-90.  90.] deg
servo_shoulder  [-0.9948  0.6981] rad = [-57.  40.] deg
servo_elbow     [-1.0908  0.3054] rad = [-62.5  17.5] deg
servo_gripper   [0.     1.5708] rad = [ 0. 90.] deg
```

3 dòng đầu lấy từ `CTRL_BASE_DEG`, `CTRL_SHOULDER_DEG`, `CTRL_ELBOW_DEG` trong `params.py`. Dòng cuối lấy từ `GRIP_TRAVEL_DEG`: 0 là đóng, 90° là mở hết.
Cận dưới của khuỷu là −62,5°, thay cho −57,3° trước ngày 2026-10-23. Lý do và việc cần kiểm tra trên tay thật: mục 9b.

✅ **Kiểm tra — MuJoCo nạp được cả 3 mô hình:**

```bash
for f in model/mk2.xml model/mk2_pen.xml model/mk2_sort.xml; do python -c "import mujoco,sys; m=mujoco.MjModel.from_xml_path(sys.argv[1]); print(sys.argv[1], 'nbody',m.nbody,'njnt',m.njnt,'neq',m.neq,'nu',m.nu,'nmesh',m.nmesh,'nkey',m.nkey)" $f; done
ls model/meshes | wc -l
```

Kết quả mong đợi:

```
model/mk2.xml nbody 17 njnt 16 neq 9 nu 4 nmesh 20 nkey 0
model/mk2_pen.xml nbody 17 njnt 16 neq 9 nu 4 nmesh 20 nkey 1
model/mk2_sort.xml nbody 31 njnt 27 neq 9 nu 4 nmesh 37 nkey 0
20
```

PASS khi tất cả các điều sau đúng:
- Mọi dòng `lech` ≤ 0,05 mm.
- 2 dòng `o home` lệch < 0,05 mm.
- Tổng khối lượng là `588.7 g` (nếu chưa sửa `params.py`).
- 3 dòng nạp mô hình in đúng như trên.

Ý nghĩa các con số:
- **17 body:** world, 9 khâu tay (khâu `wrist` mang thân gripper), 6 chi tiết gripper, gearservo 010.
  - 6 chi tiết gripper là: 2 "bánh răng + khâu dưới", 2 khâu trên, 2 ngón.
- **16 khớp:** 9 khớp tay, 6 khớp gripper, khớp gearservo.
  - Khớp `gripper` là trục SG90, các khớp còn lại của gripper bị ràng buộc theo nó.
- **9 ràng buộc:** 3 vòng hình bình hành của tay, cặp bánh răng gripper, 4 ràng buộc hình bình hành gripper (2 ngón, 2 khâu trên), cặp bánh răng đế 010–011.
- **4 servo:** đế, vai, khuỷu, gripper.
- `mk2_sort.xml` có thêm:
  - 14 body: kệ, hộp, vùng vứt bỏ, 9 nắp và 2 vật lạ.
  - 11 khớp tự do (`freejoint`), mỗi khớp cho 1 nắp.
  - 17 lưới lồi khai báo thẳng trong XML (16 mảnh mica, 1 chân lục giác), không ghi file vào `meshes/`.
- **20 lưới:** 13 STL gốc và 7 lưới gripper `GR_*`.
- **`nkey 1`:** `mk2_pen.xml` có tư thế mẫu (keyframe) `pen`, trong đó gripper mở 4,29° vừa kẹp bút. Các script tự nạp tư thế này.

Nếu đặt `BACKLASH_DEG > 0`, mỗi khớp dẫn động của tay (đế, vai, khuỷu) có thêm một khớp "play", nên `njnt` sẽ là 19 (đã thử với 0,5°).

**Quan trọng:** sau mỗi lần sửa `params.py`, `gripper.py` hoặc `gripper_shape.py`, phải chạy lại `build_model.py`.

### 5b. Kiểm chứng lắp ráp (khuyến nghị, chạy một lần)

```bash
python tools/check_assembly.py --stl-dir "$STL_DIR"
```

Script dựng trường khoảng cách có dấu (SDF) cho 19 chi tiết (12 STL của tay và đế, 7 chi tiết gripper); thân bút được kiểm tra bằng các điểm mẫu trên bề mặt. Sau đó script kiểm tra:
- **[A]** Tại tư thế home, liệt kê các cặp chi tiết nằm sát nhau.
- **[B]** Tại 33 tư thế phủ hết giới hạn khớp, kiểm tra không cặp chi tiết nào lồng vào nhau. Tư thế đầu tiên được thử thêm với khớp đế ±45°.
- **[D]** Gripper ở tư thế home, góc servo kẹp 0 → 90° (bước 10°): các chi tiết của gripper không lồng vào nhau.

Script chạy vài phút. Kết quả mong đợi (rút gọn):

```
[A] Tu the home — cac cap cham nhau (khe < 0.5 mm; am nho la sai so luoi):
    004_link135                  - 006_horarm__                 khe -0.20 mm
    006_horarm__                 - GR_body                      khe -0.20 mm
    ...
    pen                          - GR_finger_l                  khe -0.08 mm
    pen                          - GR_finger_r                  khe -0.07 mm
    ...
    005_link135angled            - 001_base                     khe +0.50 mm
[B] 33 tu the trong gioi han (theta2 (42.5, 125.0), phi (-65.0, 20.0), rel (-145.0, -60.0)):
    khe nho nhat = -0.20 mm (004_link135 - 006_horarm__ tai (42.5, -65.0, -45.0))
PASS: khong chi tiet nao lan vao nhau trong gioi han khop
[D] Gripper, goc servo kep 0..90 do (buoc 10): khe nho nhat = -0.24 mm (GR_gear_l - GR_gear_r tai 10 do)
PASS: gripper khong tu va cham
```

✅ **PASS khi** có cả 2 dòng `PASS`.
- Khe âm cỡ −0,2 mm là sai số của lưới 0,3 mm tại các mặt đang tì vào nhau, không phải va chạm thật. Ví dụ: 2 bánh răng ăn khớp, ngón tì vào thân bút.
- Ngưỡng báo lỗi là −0,35 mm.

Thêm `--limits` để in bảng va chạm **[C]** theo (θ2, φ) rộng ra ngoài giới hạn. Phần này chạy thêm khoảng 10 phút; đây là cách đã dùng để đặt `THETA2_LIM`, `PHI_LIM`, `REL_LIM` (mục 10).
Với gripper thật, tay ngang `006` chạm thân gripper `GR_body` khi φ ≤ −70°, nên `PHI_LIM = (-65, 20)` vẫn giữ nguyên.

## 6. Xem mô hình và điều khiển bằng tay

Điều kiện: bước 2.2 PASS (không phải `llvmpipe`).

### 6.1 Mô hình mặc định (gắp vật, không bút)

```bash
python -m mujoco.viewer --mjcf=model/mk2.xml
```

✅ **Kiểm tra bằng mắt — PASS khi:**
1. Cửa sổ mở ra, thấy tay máy và gripper ở đầu tay. Không có bút, không có giấy. Terminal không báo lỗi.
   - Màu gripper: thân xám đậm, "bánh răng + khâu dưới" xanh ngọc, khâu trên xanh dương, ngón xám nhạt, SG90 là hộp xanh dương.
2. Ở tư thế ban đầu (các thanh điều khiển = 0): tay chính dựng đứng, tay ngang nằm ngang, gripper đóng và hướng về phía trước.
3. Mở panel bên phải → mục **Control** (phím tắt `Alt`+`C`), kéo 3 thanh `servo_base`, `servo_shoulder`, `servo_elbow`. Tay máy bám theo và **gripper luôn nằm ngang** nhờ hình bình hành.
4. Kéo thanh `servo_gripper` từ 0 lên 1.571:
   - 2 bánh răng quay ngược chiều nhau.
   - 2 ngón tách ra đối xứng và **tịnh tiến**: ngón luôn song song với nhau, không xoay.
   - Khi mở hết, khâu dưới vuông góc với cánh tay.

Đơn vị thanh trượt là rad.
- 3 servo tay có phạm vi theo `CTRL_*_DEG` (bước 5). Riêng `servo_base` là góc **trục servo**; qua hộp số 2:1, khớp đế chỉ quay một nửa góc này.
- `servo_gripper`: 0 là đóng (khâu dưới duỗi thẳng), 1.571 rad (90°) là mở hết.

### 6.2 Mô hình có bút (thử vẽ)

```bash
python -m mujoco.viewer --mjcf=model/mk2_pen.xml
```

✅ **PASS khi:** thấy thêm tờ giấy A4 màu trắng trước đế và bút màu xanh thẳng đứng giữa 2 ngón.

Lưu ý về tư thế ban đầu:
- Viewer khởi động với mọi thanh điều khiển = 0, nên 2 ngón có thể đang khép xuyên qua thân bút.
- Muốn đặt đúng tư thế kẹp bút: panel bên trái → mục **Simulation** → bấm **Load key** (key 0 = `pen`). Khi đó `servo_gripper` = 0.0748 rad (4,29°).
- Bút gắn cứng vào cổ tay, nên mở gripper bút cũng không rơi. Lực kẹp không được mô phỏng (mục 11).

### 6.3 Thao tác chuột/phím

Theo tài liệu MuJoCo "simulate":
- `Space`: chạy/tạm dừng.
- Nhấp đúp vào một chi tiết: chọn chi tiết đó.
- Giữ `Ctrl` + kéo **chuột trái**: xoay chi tiết đã chọn (tác dụng mô-men).
- Giữ `Ctrl` + kéo **chuột phải**: đẩy chi tiết trong mặt phẳng đứng (tác dụng lực); thêm `Shift` để đẩy trong mặt phẳng ngang.

Đóng cửa sổ để thoát. Trong viewer này **không có vệt mực**, vì vệt mực do `run_sim.py --pen` vẽ (bước 8).

## 7. Bước kiểm chứng động học (mục "Mô phỏng" trong đề cương)

Chạy 2 lần: cho TCP gripper (mặc định) và cho ngòi bút.

```bash
python check_kinematics.py          # khoảng 6 giây trên máy thử
python check_kinematics.py --pen    # khoảng 21 giây (phần [4] quét vùng làm việc)
```

TCP gripper là điểm giữa 2 đệm mũi ngón khi đóng, lùi 5 mm từ mút ngón (`GRIP_TCP_BACK`).

Kết quả mong đợi của lệnh thứ nhất:

```
Dau cong tac: gripper (132.69, 0.48, 0.5) mm so voi W. CANH BAO: PEN_LENGTH, TOOL_MASS_G, GRIP_CENTER_Y va gioi han khop trong params.py la gia tri tam [GIA-DINH]. Hay do tren tay may that roi sua lai.
[1] FK(IK(p)) tren 5000 diem: max |FK(IK(p))-p| = 1.30e-13 mm, max |IK(FK(q))-q| = 1.14e-12 mrad
[2] MuJoCo (vong kin, g=0) vs FK giai tich (TCP gripper), 40 tu the: max sai lech = 0.011 mm, max vi pham rang buoc vong = 0.0000 mm
[3] Do vong do trong luc (servo P huu han, params.SERVO_ERR_SAT_DEG):
    q=(0, 90, 0): lech goc (deg) = [ 0.   -0.02 -0.63], lech TCP gripper = 1.72 mm, ...
    q=(0, 60, -30): lech goc (deg) = [ 0.   -0.72 -0.56], lech TCP gripper = 2.28 mm, ...
    q=(0, 45, -55): lech goc (deg) = [ 0.   -1.01 -0.39], lech TCP gripper = 2.46 mm, ...
    q=(0, 110, 0): lech goc (deg) = [ 0.    0.47 -0.63], lech TCP gripper = 2.39 mm, ...
[4] Bang hinh ve kha thi chi danh cho but: chay  python check_kinematics.py --pen
Da luu .../results/workspace_rz_gripper.png
```

Kết quả mong đợi của lệnh `--pen`:

```
Dau cong tac: pen (110.59, 0.48, -89.5) mm so voi W. CANH BAO: ...
[1] FK(IK(p)) tren 5000 diem: max |FK(IK(p))-p| = 1.39e-13 mm, max |IK(FK(q))-q| = 1.20e-12 mrad
[2] MuJoCo (vong kin, g=0) vs FK giai tich (ngoi but), 40 tu the: max sai lech = 0.013 mm, max vi pham rang buoc vong = 0.0000 mm
[3] Do vong do trong luc (servo P huu han, params.SERVO_ERR_SAT_DEG):
    q=(0, 90, 0): lech goc (deg) = [ 0.   -0.02 -0.76], lech ngoi but = 2.17 mm, ...
    q=(0, 60, -30): lech goc (deg) = [ 0.   -0.78 -0.67], lech ngoi but = 2.59 mm, ...
    q=(0, 45, -55): lech goc (deg) = [ 0.   -1.09 -0.46], lech ngoi but = 2.71 mm, ...
    q=(0, 110, 0): lech goc (deg) = [ 0.    0.51 -0.76], lech ngoi but = 2.95 mm, ...
[4] Hinh ve lon nhat (sao + tron ban kinh R) theo do cao mat giay z (mm), cach gioi han >= 3 deg:
    z_giay =   1.0 mm: tam x = 275 mm, R_max =  34.1 mm   <- mat giay (params.PAPER_THICKNESS)
    z_giay =  10.0 mm: tam x = 265 mm, R_max =  51.3 mm
    z_giay =  20.0 mm: tam x = 265 mm, R_max =  62.9 mm
    ...
    z_giay =  70.0 mm: tam x = 260 mm, R_max =  82.1 mm
    ...
    z_giay = 100.0 mm: tam x = 260 mm, R_max =  71.4 mm
Da luu .../results/workspace_rz_pen.png
```

Dòng `CANH BAO` là nhắc nhở bình thường, không phải lỗi. Phần [2] và [3] tắt va chạm ngòi–giấy để so sánh động học thuần.

✅ **Kiểm tra — PASS khi:**

| Dòng | Tiêu chí | Ý nghĩa nếu FAIL |
|---|---|---|
| [1] | cả hai sai số < 1e-9 | Công thức IK không còn là nghịch đảo của FK (đã sửa nhầm `mk2_kinematics.py`?) |
| [2] | sai lệch < 0.05 mm **và** vi phạm vòng ≈ 0 | Lớp ánh xạ hình bình hành hoặc `model/*.xml` không khớp `params.py` → chạy lại bước 5 |
| [3] | độ võng cỡ 1,5–2,5 mm (TCP) và 2,0–3,0 mm (ngòi bút) | Lớn hơn nhiều: `SERVO_ERR_SAT_DEG` hoặc khối lượng bị đổi |
| [4] (`--pen`) | dòng `<- mat giay` có `R_max` > 20 mm, có file `results/workspace_rz_pen.png` | `R = 20` mặc định ở bước 8 sẽ vượt vùng làm việc |

Ý nghĩa từng dòng:
1. Công thức IK là nghịch đảo đúng của FK. FK/IK đã tính cả độ lệch ngang 0,48 mm của đầu công tác so với mặt phẳng cánh tay (`GRIP_CENTER_Y`, mục 10).
2. Mô hình vật lý có vòng kín cho cùng vị trí đầu công tác với công thức giải tích. Điều này chứng minh **lớp ánh xạ hình bình hành** là đúng.
3. Servo là bộ điều khiển P hữu hạn, nên dưới trọng lực đầu công tác võng xuống vài mm. Đây là hiệu ứng thật mà mô hình động học thuần không thấy được.
   - Tư thế thử thứ 3 là (0, 45, −55) chứ không phải −60. Lý do: khi viết bước này, φ = −60° nằm ngoài tầm lệnh `CTRL_ELBOW_DEG` cũ (−57,3°). Tư thế được giữ nguyên để số liệu so sánh được.
4. Bảng để **chọn R và tâm hình vẽ**. Dòng đánh dấu là mặt tờ giấy thật (dày 1 mm, nằm trên bàn). Chọn R nhỏ hơn hẳn R_max để còn khoảng dự phòng.

**Vì sao R_max trên mặt giấy là 34,1 mm (bản kẹp cũ: 56,4 mm).** Có 2 nguyên nhân. Bảng [4] đã được chạy lại khi thay từng yếu tố:

| Trường hợp | R_max tại z = 1 mm |
|---|---|
| Kẹp cũ, chưa có tầm lệnh servo (README cũ) | 56,4 mm |
| Kẹp cũ + `CTRL_ELBOW_DEG` | 34,9 mm |
| Gripper thật, chỉ dùng `PHI_LIM` | 42,2 mm |
| Gripper thật + `CTRL_ELBOW_DEG` cận dưới −57,3° (2026-10-16) | 22,9 mm |
| Gripper thật + `CTRL_ELBOW_DEG` cận dưới −62,5° (hiện tại, từ 2026-10-23, mục 9b) | **34,1 mm** (tâm x = 275 mm) |

- **Gripper thật kẹp bút cao hơn.** Điểm kẹp bút nằm cao hơn tâm W 0,5 mm; kẹp cũ thấp hơn W 8 mm. Ngòi vì vậy cao hơn 8,5 mm so với cổ tay, nên muốn chạm giấy thì tay ngang phải chúc xuống sâu hơn.
- **Tầm lệnh servo khuỷu chặn ở φ ≥ −62,5°** (`CTRL_ELBOW_DEG`; trước 2026-10-23 là −57,3°, commit "update - Operating Angle"). Giới hạn này vẫn chặt hơn giới hạn va chạm `PHI_LIM` = −65°. IK kiểm tra cả tầm lệnh này, nên điểm vượt tầm bị báo `IKError` thay vì bị cắt lệnh âm thầm.
- Nếu cần hình to hơn:
  - Kê giấy cao hơn: z = 10 mm cho R_max 51,3 mm; z = 20 mm cho 62,9 mm.
  - Hoặc cho bút thò dài hơn: thò thêm k mm tương đương kê giấy cao thêm k mm.
  - Hoặc nới thêm `CTRL_ELBOW_DEG` nếu servo thật quay được tới đó (không vượt `PHI_LIM` = −65°).

## 8. Chạy thí nghiệm M0 vs M2

Bài vẽ **cần bút**, nên phải có cờ `--pen`:

```bash
python run_sim.py --pen     # mặc định: R = 20 mm, tâm (290, 0) = tâm tờ A4, ngòi vẽ ở z = 1 mm (mặt giấy)
```

Thiếu `--pen`, script dừng ngay với thông báo `run_sim.py la bai VE hinh, can but. ... python run_sim.py --pen`.

Trình tự mỗi lần vẽ, mỗi phương pháp coi như vẽ trên một tờ giấy mới:
1. Đưa bút tới vị trí cách điểm đầu 15 mm theo phương đứng (`PEN_LIFT`).
2. Hạ bút thẳng xuống trong 1 s, dừng 0,3 s.
3. Vẽ một nét liền (M0 hoặc M2).
4. Dừng 0,3 s, nhấc bút lên trong 0,5 s.

**Mực chỉ được ghi khi MuJoCo báo ngòi chạm giấy**, tức khe ngòi–giấy ≤ `INK_GAP` = 0,05 mm. Nét rộng 1 mm. Các chỉ số đo trên **vết mực**, giống cách đo trên giấy thật.

Kết quả mong đợi với tham số mặc định (chạy khoảng 7 giây trên máy thử):

```
Dau cong tac: pen (110.59, 0.48, -89.5) mm so voi W. CANH BAO: ...
Duong di L = 271.0 mm (ly thuyet 13.55R = 271.0); T(M2) = 3.60 s; v(M0) = L/T = 75.3 mm/s
[M0] toc do servo lon nhat = [0.56 0.38 0.33] rad/s (MG996R khong tai 5.51 rad/s @4.8V) -> OK
[M2] toc do servo lon nhat = [1.11 0.74 0.64] rad/s (MG996R khong tai 5.51 rad/s @4.8V) -> OK
[M0] max vi pham rang buoc vong = 0.055 mm, max |mo-men servo| = [0.083 0.219 0.125] Nm
[M0] ngoi cham giay 100.0% thoi gian ve; so net muc = 1; luc ep ngoi TB/max = 3.09/4.05 N
[M2] max vi pham rang buoc vong = 0.054 mm, max |mo-men servo| = [0.107 0.219 0.125] Nm
[M2] ngoi cham giay 100.0% thoi gian ve; so net muc = 1; luc ep ngoi TB/max = 3.07/5.86 N

Chi so du doan tu mo phong (cung T), do tren VET MUC: 5 mui sao / duong tron / 10 canh:
PP    vot lo TB  vot lo max  sai lech dinh TB  do tron  lech thang max  co muc %   (mm)
M0        -0.81       -0.10              0.85     1.02            1.19     100.0
M2        -0.44        0.13              0.50     0.55            1.17     100.0
  vot lo < 0: net ve cat goc, khong cham giao diem 2 canh keo dai
```

✅ **Kiểm tra:**

```bash
ls results/cmd_derivatives.png results/pen_trace.png results/log_M0.csv results/log_M2.csv results/ink_M0.png results/ink_M2.png
wc -l results/log_M0.csv results/log_M2.csv          # -> 311 dòng mỗi file (1 dòng tiêu đề + 310 mẫu)
md5sum results/cmd_derivatives.png assets/simulation_results/cmd_derivatives.png
md5sum results/pen_trace.png       assets/simulation_results/pen_trace.png
```

PASS khi:
- Cả hai dòng tốc độ servo kết thúc bằng `-> OK`.
- Vi phạm ràng buộc vòng < 0,1 mm.
- **Ngòi chạm giấy 100% thời gian vẽ, mỗi lần chỉ 1 nét mực** (nét liền, không đứt).
- Đủ 6 file, mỗi CSV 311 dòng.
- Bảng chỉ số trùng bảng trên (với tham số mặc định).

Hai lệnh `md5sum` là phép so khắt khe nhất: nếu mỗi cặp cho cùng một mã, ảnh của bạn trùng từng byte với ảnh mẫu.
- Nếu mã khác nhưng bảng số liệu vẫn trùng, mở hai ảnh ra so bằng mắt. Khác biệt nhỏ ở mức điểm ảnh có thể do khác CPU hoặc phông chữ, chưa phải lỗi.
- Ảnh `ink_*.png` dựng bằng OpenGL nên điểm ảnh phụ thuộc GPU; chỉ so bằng mắt với `assets/simulation_results/ink_M2.png`.

Các tùy chọn khác:

```bash
python run_sim.py --pen --view       # xem robot vẽ, vệt mực hiện dần trên giấy (cần bước 6 PASS)
python run_sim.py --pen --R 15 --vmax 150 --amax 1500 --jmax 30000
```

Với `--view`:
- Camera tự hướng vào tờ giấy. Mỗi phương pháp bắt đầu trên tờ giấy trắng: M0 vẽ màu đỏ, M2 màu xanh.
- Viewer chạy ở chế độ "passive", nên phím `Space` (tạm dừng) không có tác dụng (theo tài liệu MuJoCo).
- Script tự đóng cửa sổ khi chạy xong.

Với `--R 15`, bảng chỉ số mong đợi là `M0 -0.77 -0.38 0.80 0.84 1.18 100.0` và `M2 -0.39 0.03 0.42 0.56 1.16 100.0`.

Về các giá trị R khác:
- `--R 30` vẫn chạy được, nhưng hình vẽ đã đi vào vùng cách giới hạn khớp dưới 3° (bảng [4] bước 7).
- `--R 35` báo `IKError: th2=42.5 ngoai (42.5, 125.0)`.

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
> - M0 **cắt góc** khoảng 0,8 mm chứ không vọt lố, nên chỉ số "vọt lố" trong đề cương có thể ra âm với M0. Nên đo thêm "sai lệch đỉnh".
> - Bút được kẹp cứng, lệnh ngòi đặt đúng mặt giấy. Độ võng do trọng lực (bước 7, dòng [3]) ép ngòi xuống giấy với lực trung bình khoảng 3,1 N.
> - "Lệch thẳng max" cỡ 1,2 mm ở cả M0 và M2. Nguyên nhân chưa được phân tích.
> - *Số liệu của bản kẹp cũ, chưa chạy lại với gripper thật:* thử với μ ≈ 0, lực ép giảm còn ~1,3 N và vọt lố TB của M2 đổi từ −0,38 thành 0,00 mm; tắt ma sát hoặc tắt trọng lực, "lệch thẳng" vẫn còn. Kết quả nhạy với ma sát ngòi–giấy (`PEN_TIP_FRICTION`, đang giả định 0,2); cần đo hệ số này trên bút thật.

Lưu ý: M2 trong gói này dùng **Ruckig** làm chuẩn tham chiếu, như đề cương mô tả. Bộ double-S tự viết của bạn (để chạy trên ESP32) có thể đưa vào thay hàm `m2_path` trong `trajectories.py`, rồi so sánh với Ruckig.

## 9. Chụp ảnh tư thế cho báo cáo

Điều kiện: bước 2.2 PASS.

```bash
python snapshot.py --pen --xyz 290 0 20 --out results/pose.png    # đặt NGÒI BÚT tại (290, 0, 20) mm bằng IK
python snapshot.py --xyz 260 0 100 --out results/pose_gripper.png # đặt TCP gripper tại (260, 0, 100) mm bằng IK
python snapshot.py --servo 90 90 90 --out results/home.png        # đặt góc 3 servo trực tiếp (độ)
python snapshot.py --grip 90 --servo 90 90 90 --out results/open.png   # mở gripper 90 độ
```

Kết quả mong đợi của 3 lệnh đầu:

```
servo (deg): [89.81 58.18 47.42] | FK ngoi but (mm): [290.   0.  20.]
MuJoCo ngoi but (mm): [290.55  -0.    17.78]
Da luu results/pose.png
servo (deg): [89.79 70.43 33.94] | FK TCP gripper (mm): [260.   0. 100.]
MuJoCo TCP gripper (mm): [260.28  -0.    99.  ]
Da luu results/pose_gripper.png
servo (deg): [90. 90. 90.] | FK TCP gripper (mm): [279.69   0.48 229.75]
MuJoCo TCP gripper (mm): [279.72   0.48 228.03]
Da luu results/home.png
```

✅ **Kiểm tra — PASS khi:** có file `results/pose.png` và ảnh giống `assets/simulation_results/pose_xyz_290_0_20.png`.
- Chênh lệch 1–2,3 mm giữa FK và MuJoCo là độ võng do trọng lực, cùng bản chất với dòng [3] ở bước 7, không phải lỗi.
- Góc servo đế là 89,8° chứ không phải 90° vì IK bù độ lệch ngang 0,48 mm của đầu công tác.
- `--grip` là góc servo kẹp (độ). Không có `--grip`: gripper đóng, hoặc kẹp bút 4,29° nếu có `--pen`.

## 9b. Chu trình phân loại màu nắp chai (`run_sort.py`)

Bài gắp–thả của đề tài chạy trên mô hình `model/mk2_sort.xml`. Mô hình này được dựng ở bước 5 cùng 2 mô hình kia, bằng `build_model.py` + `sort_scene.py`.

### 9b.1 Đồ vật (nguồn kích thước: yêu cầu của chủ dự án, 2026-10-23, trừ dòng ghi khác)

| Đồ vật | Mô hình trong MuJoCo |
|---|---|
| Kệ trạm cảm biến | Mặt mica trong suốt 60 × 60 × 3 mm, lỗ tròn Ø23 ở giữa, mặt trên ở z = 43 mm. 4 chân lục giác đều (đỉnh–đỉnh 5 mm) cao 40 mm, đặt sát 4 góc, không lòi ra ngoài mặt kệ. Mặt mica ghép từ 16 mảnh lồi quanh lỗ |
| Module GY-33 | Hộp 25 × 25 × 12 mm, tâm ngay dưới lỗ, mặt trên chạm mặt dưới mica (z = 40 mm). Không mô hình giá đỡ |
| Nắp chai | 3 xanh lá + 3 đỏ + 3 trắng. Trụ rỗng Ø30 × 13 mm, thành 1 mm, 2,5 g, đặt **úp**. Thêm 2 "vật lạ" màu xanh dương, chỉ dùng khi chạy `--foreign` |
| Hộp đích | 1 khối 3 ngăn liền, thành và đáy 3 mm. Ngoài 177 × 66 × 50 mm, lòng mỗi ngăn 55 × 60 mm. Kích thước do mô phỏng chọn `[CHON]` |
| Vùng vứt bỏ | Vùng phẳng 80 × 80 mm vẽ trên mặt bàn `[CHON]` |
| Gripper | Thêm **tấm kẹp có va chạm** dọc mặt kẹp của 2 ngón, theo 3 lớp chiều dày của ngón (`sort_scene.finger_pads`). Các chi tiết khác của tay và gripper vẫn không có va chạm |

### 9b.2 Bố trí

Hệ tọa độ: gốc ở trục quay đế trên mặt bàn, x hướng ra trước, y sang trái, z lên. Kệ đặt đối diện tay máy, vùng vứt bỏ bên trái, hộp bên phải.
Các số `[TINH]` được tính lại bằng `python tools/check_sort_layout.py`.

| Đồ vật | Vị trí | Vì sao |
|---|---|---|
| Kệ | Tâm (279; 0) | Kẹp nắp cần điểm "cap" (tâm nắp) ở z = 59 mm. Ở độ cao này tay chỉ với tới bán kính 274,5…283,5 mm; 279 nằm giữa dải |
| Hộp 3 ngăn | Trục dài hướng xuyên tâm, góc −30°, tâm r = 266 mm. Ngăn 3 (trắng) r = 208, ngăn 2 (xanh lá) r = 266, ngăn 1 (đỏ, xa robot nhất) r = 324 | Thả ở z = 116 mm; dải với tới ở độ cao này là 193…338,5 mm, nên mỗi đầu còn dư khoảng 14 mm. Đặt xuyên tâm thì đế chỉ cần quay đúng 1 góc (giới hạn ±45°) |
| Vùng vứt bỏ | Tâm r = 285 mm, góc +30° | Thả ở z = 66 mm; dải với tới 267…294,5 mm |

**Vì sao điểm kẹp ở z = 59 mm và `CTRL_ELBOW_DEG` = −62,5°.**
- Nắp úp trên kệ chiếm z = 43…56 mm. Ngón dày 15 mm, điểm "cap" ở giữa chiều dày.
- Ở z = 59 mm, tấm dưới của ngón (dày 3,3 mm) nằm ở z = 51,5…54,8 mm, ôm trọn trên thân nắp; tấm trên nằm cao hơn nắp.
- Với cận cũ −57,3°, điểm "cap" thấp nhất chỉ là 62,3 mm. Khi đó tấm dưới chỉ chồng lên nắp khoảng 1,2 mm.
- Chủ dự án chọn nới cận này thành −62,5°, vẫn cách biên va chạm STL (`PHI_LIM` = −65°) 2,5°.
- **Phải kiểm tra trên tay thật:** cho tay ngang chúc xuống khoảng 62,5° so với phương ngang (tay chính nghiêng khoảng 43,5°) và xác nhận không kẹt, không va chạm.

### 9b.3 Chu trình (mỗi nắp)

1. Tay ở Home (servo 90/90/90), gripper mở 40°. Sau `CAP_PLACE_DELAY_S` = 1 s, "người" đặt 1 nắp lên trạm. Thứ tự nắp ngẫu nhiên theo `--seed`.
2. ESP32 đọc kênh C mỗi `POLL_MS` = 500 ms (`src/robot/sorter.h` của firmware).
   - Khi |C − C_empty| ≥ C_margin (có nắp), chờ `settle_ms` = 300 ms (≥ 200 ms theo yêu cầu).
   - Sau đó lấy trung bình 5 lần đọc RGBC, mỗi lần cách nhau 120 ms. Tổng thời gian đo khoảng 0,78 s.
3. Phân loại bằng `color_sim.classify()`, bản sao `colormath::classify()` (`lib/ColorClassifier/ColorMath.h`), tính bằng float32 giống ESP32.
   - Lớp được chọn theo tâm lớp gần nhất trong mặt phẳng (r, g).
   - d > d_max → vứt bỏ, in `# ERR UNKNOWN_COLOR ...` như firmware.
   - |C − C_empty| < C_margin → không có nắp, tiếp tục chờ.
4. Tới điểm cao hơn điểm kẹp 40 mm, hạ thẳng xuống, kẹp, nhấc thẳng lên.
   - Lệnh kẹp là 15°, thấp hơn góc chạm nắp 21,27° hơn `SERVO_ERR_SAT_DEG`, nên SG90 kẹp bằng mô-men bão hòa.
5. Tới điểm mà **đáy nắp cách mặt trên hộp 50 mm** (z điểm "cap" = 50 + 50 + 16 = 116 mm) rồi mở kẹp.
   - Với vùng vứt bỏ, đáy nắp cách mặt bàn 50 mm (z = 66 mm). Tay tới phía trên vùng ở z = 99 mm rồi mới hạ thẳng xuống.
6. Về Home, chờ nắp tiếp theo.

Quỹ đạo, lệnh mỗi 20 ms → IK → servo:
- **Giữa các trạm:** double-S đồng bộ 3 khớp (Ruckig, cả 3 khớp xong cùng lúc). Giới hạn 0,536 rad/s, 5,36 rad/s², 107 rad/s³, quy đổi từ 150 mm/s, 1500 mm/s², 30000 mm/s³ (`include/arm_config.h` của firmware) qua bán kính 280 mm.
- **Hạ và nhấc:** đường thẳng đứng, double-S theo độ dài cung với 150 / 1500 / 30000.

**Cảm biến trong mô phỏng.** MuJoCo không mô phỏng quang học của TCS34725, nên mỗi phép đo **phát lại 1 dòng đo thật** của `data/color/test.csv`, đúng nhãn của vật đang nằm trên trạm.
- Nắp và vật lạ: rút không hoàn lại.
- Trạm trống: rút có hoàn lại.
- `data/color/model.json` và `test.csv` được chép nguyên từ repo `eezybot_mk2_robot` (commit `a172b69`, hiệu chuẩn 2026-10-23).

✅ **Kiểm tra 1 — bộ phân loại khớp firmware:**

```bash
python color_sim.py
```

Kết quả mong đợi: ma trận trùng `reports/color/eval_test.json` của repo firmware (99/100, 1 mẫu đỏ ở dòng 24 bị "không xác định").

```
  green  [20  0  0  0  0]
  red    [ 0 19  0  1  0]
  white  [ 0  0 20  0  0]
  empty  [ 0  0  0  0 20]
  other  [ 0  0  0 20  0]
Dung 99/100 = 99.0%
```

✅ **Kiểm tra 2 — chạy chu trình:**

```bash
python run_sort.py            # 9 nắp, seed 0; khoảng 1 phút 45 giây trên máy thử 2 nhân (thời gian mô phỏng 78 s)
python run_sort.py --view     # xem trực tiếp; tối đa bằng thời gian thực (máy chậm sẽ chạy chậm hơn)
python run_sort.py --foreign 2  # thêm 2 vật lạ -> phải vào vùng vứt bỏ kèm "# ERR UNKNOWN_COLOR"
```

Kết quả mong đợi (`python run_sort.py`, rút gọn):

```
Tu the (diem 'cap' = tam nap khi kep, mm) -> goc servo (do), du tru toi gioi han gan nhat:
  tram: diem kep         p = [279.   0.  59.]  servo = [89.8  43.5  28.91]  du tru = 1.00 do
  tram: phia tren        p = [279.   0.  99.]  servo = [89.8  57.94 41.46]  du tru = 13.96 do
  ngan 3 (white): tha    p = [ 180.1 -104.   116. ]  servo = [29.74 87.97 39.35]  du tru = 6.39 do
  ngan 2 (green): tha    p = [ 230.4 -133.   116. ]  servo = [29.79 67.52 45.25]  du tru = 14.90 do
  ngan 1 (red): tha      p = [ 280.6 -162.   116. ]  servo = [29.83 48.09 57.39]  du tru = 5.59 do
  vut bo: phia tren      p = [246.8 142.5  99. ]  servo = [149.81  56.17  42.78]  du tru = 13.67 do
  vut bo: tha            p = [246.8 142.5  66. ]  servo = [149.81  44.79  32.59]  du tru = 2.29 do
Thu tu dat nap (seed 0): red, red, green, white, red, white, white, green, green
[t=   3.28 s] CLS,RED,RED,0.001499,0.461412,0.274220,281.000,167.000,161.000,621.000,0
[t=   3.28 s] # SORT RED (lan 1)  [nap 1: cap_red_1, dong test.csv 31]
           gap: z diem kep (thuc) = 53.5 mm, kep 20.30 do, luc kep 5.38 N, nhac nap 40.3 mm, truot max 0.46 mm; day nap luc tha 99.1 mm; nap nam o: red -> DUNG dich
...
[t=  77.62 s] Tram trong, het nap -> dung.

Toc do servo lon nhat = [1.07 0.97 0.86] rad/s (MG996R khong tai 5.51 rad/s @4.8V) -> OK
Khe ho nho nhat tren quy dao LENH (FK; diem thap nhat = day nap khi mang nap, mep duoi ngon khi kep rong):
  Home -> tren tram (kep rong)           mat ban 91.5 mm, ke + nap 35.5 mm
  tram -> ngan 1 (mang nap)              mat ban 83.0 mm, ke 40.0 mm, hop dich 44.7 mm
  ve Home (kep rong)                     mat ban 108.5 mm, hop dich 58.5 mm, ke 109.3 mm
  tram -> ngan 2 (mang nap)              mat ban 83.0 mm, ke 40.0 mm, hop dich 44.3 mm
  tram -> ngan 3 (mang nap)              mat ban 83.0 mm, ke 40.0 mm, hop dich 47.7 mm
So chu ky lenh co tam kep cham ke/hop/ban: 0
Thoi gian 1 chu ky (dat nap -> tay ve Home): TB 7.35 s, min 7.22, max 7.50
Thoi gian tu luc dat nap toi luc ra quyet dinh: TB 0.78 s
Ma tran: mau that (hang) x noi nap nam sau chu trinh (cot: ngan 1 do, ngan 2 xanh, ngan 3 trang, vut bo, ngoai)
  green  [0 3 0 0 0]
  red    [3 0 0 0 0]
  white  [0 0 3 0 0]
Nap toi dung dich da chon (co hoc gap-tha): 9/9; nap toi dung noi theo mau that (toan chu trinh): 9/9
```

Đã thử thêm `--seed 1`, `--seed 2`, `--seed 3`: đều cho 9/9. `--foreign 2` cho 11/11: 2 vật lạ có 2 dòng `# ERR UNKNOWN_COLOR nearest=WHITE dist=0.08..` và nằm trong vùng vứt bỏ.

PASS khi:
- Dòng cuối ghi `9/9` ở cả 2 vế.
- `So chu ky lenh co tam kep cham ke/hop/ban: 0`: ngón không chạm kệ, hộp hay bàn.
- Mọi dòng `truot max` < 1 mm: nắp không trượt trong kẹp.
- Tốc độ servo `-> OK`.

Các file trong `results/sort/`:

| File | Nội dung |
|---|---|
| `summary.txt` | Toàn bộ output trên, gồm các dòng `CLS,...`/`# SORT ...`/`# ERR ...` giống Serial của firmware |
| `caps.csv` | Mỗi nắp: màu thật, dòng `test.csv` đã dùng, R/G/B/C, r/g, d, nhãn, đích, nơi nắp nằm cuối cùng, z điểm kẹp thực, góc và lực kẹp, độ trượt, các mốc thời gian |
| `log.csv` | Lệnh 20 ms: pha, lệnh 3 servo, góc thực, lệnh và góc kẹp, vị trí điểm "cap" và vị trí nắp, cờ ngón chạm môi trường |
| `sort_matrix.png` | Ma trận màu thật × nơi nắp nằm sau chu trình |
| `sort_timeline.png` | Lệnh 3 servo và servo kẹp theo thời gian |
| `scene_end_top.png`, `scene_end_persp.png` | Ảnh cảnh cuối (cần OpenGL) |

**Kiểm tra bố trí** (không cần MuJoCo): `python tools/check_sort_layout.py` in lại mọi số `[TINH]`:
- góc chạm nắp, khe khi mở;
- lớp ngón nào ôm thân nắp;
- dải với tới theo độ cao, với cận khuỷu cũ và mới;
- góc servo và dư trữ tới giới hạn ở từng tư thế.

**Giới hạn riêng của bài phân loại** (ghi rõ trong báo cáo):
- **Độ võng khi kẹp.** Điểm kẹp lệnh ở z = 59 mm, nhưng MuJoCo cho z thực khoảng 53,5 mm (võng khoảng 5,5 mm).
  - Nguyên nhân: tư thế duỗi xa, servo P hữu hạn với `SERVO_ERR_SAT_DEG` = 5° `[GIA-DINH]`.
  - Khi đó mép dưới ngón chỉ còn cách mica khoảng 3 mm.
  - Trên tay thật hãy đo z thực ở tư thế này. Nếu ngón chạm mica, tăng `CAP_GRASP_Z` (tối đa khoảng 60 mm để tấm dưới vẫn ôm nắp).
- **Hệ số ma sát** `CONTACT_FRICTION` = 0,4 là giả định, chưa đo. Lực kẹp mô phỏng khoảng 5,4 N (tổng 2 ngón), trong khi trọng lượng nắp chỉ 0,025 N, nên kết quả gắp không nhạy với giá trị này.
- **Hình va chạm của nắp** là 12 viên nang R2 quanh chu vi + 1 tấm hộp ở mặt trên (hình vẽ vẫn là nắp rỗng).
  - Bản thử đầu dùng 1 hình trụ. Khi 3 nắp rơi chồng vào cùng 1 ngăn, va chạm trụ–trụ làm nắp nảy văng ra ngoài: 45/54 lần thả nằm đúng ngăn, có lúc mô phỏng mất ổn định.
  - Với viên nang, cùng phép thử cho 54/54.
  - Nắp vẫn được giữ đúng khối lượng và quán tính của trụ rỗng.
- Mô hình đặt `jacobian="dense"`: với 11 nắp tự do (nv = 82), chế độ `auto` của MuJoCo 3.2.3 chuyển sang Jacobian thưa và dừng với lỗi `mj_makeConstraint: constraint Jacobian mis-allocation`.
- **Người đặt nắp** được mô phỏng bằng cách đặt tức thời nắp đứng yên, đúng tâm lỗ. Mỗi dòng `test.csv` đã là trung bình 5 lần đọc, nên lần đọc kênh C để phát hiện nắp ít nhiễu hơn 1 lần đọc thật.
- **Kiểm tra va chạm của tay.** Chỉ ngón có va chạm.
  - Khe giữa nắp và khâu, bánh răng, đế của gripper được kiểm tra hình học (≥ 5 mm, dòng `nap D30` ở bước 5).
  - Khe giữa điểm thấp nhất (đáy nắp khi mang, mép dưới ngón khi kẹp rỗng) và kệ, hộp được tính trên quỹ đạo lệnh. Thân tay và servo không được kiểm tra với đồ vật.

## 10. Hiệu chỉnh cho giống thực tế (quan trọng nhất)

Ý nghĩa các nhãn trong `params.py`:
- `[GIA-DINH]`: giá trị tạm.
- `[DO-THAT]`: bạn đã đo trên robot thật bằng thước kẹp.
- `[ANH]`: lấy từ ảnh chụp.
- `[DO-STL]`: tính từ STL, vẫn nên đo lại trên tay máy thật.

Hãy đo theo thứ tự sau, sửa `params.py`, rồi chạy lại `build_model.py`:

| # | Tham số | Cách đo |
|---|---|---|
| 1 | `GRIP_CENTER_Y` | **Đang giả định 0,48 mm** (đường tâm gripper trùng tâm kẹp cũ). Ở home (servo 90/90/90), nhìn từ trên xuống, đo khoảng lệch ngang (vuông góc với cánh tay) giữa điểm giữa 2 mũi ngón và trục quay đế. Dấu dương là lệch sang trái khi nhìn dọc cánh tay ra phía trước |
| 2 | `PEN_GRIP_TO_TIP`, `PEN_FROM_TIP`, `PEN_LENGTH`, `TOOL_MASS_G` | Đo từ giữa chiều dày ngón tới ngòi (đang đặt 90 mm), vị trí trục bút so với mút ngón (27 mm), chiều dài bút; cân bút |
| 3 | `GRIP_*` (kích thước gripper) | Đã đo bằng thước kẹp. Chỉ sửa khi gripper thay đổi. Sau khi sửa, `build_model.py` sẽ dừng và in giá trị mới của `TCP_W`, `PEN_GRIP_W`, `PEN_GRIP_DEG` để chép vào `params.py` |
| 4 | `H_SHOULDER` (qua `LOWER_BASE_Z`) | Thước kẹp: từ mặt bàn tới tâm trục vai. Giá trị 94,25 mm suy từ STL, với giả định gearmast tì lên mặt tâm của 013 (bi Ø6 còn khe ~0,6 mm) |
| 5 | `SERVO_OFFSET`, `SERVO_SIGN` | Ra lệnh 90/90/90, kiểm tra tay chính thẳng đứng, tay ngang nằm ngang, khớp đế nhìn thẳng; ghi độ lệch và chiều quay |
| 6 | `THETA2_LIM`, `PHI_LIM`, `REL_LIM`, `THETA1_LIM` | Đang lấy từ kiểm tra va chạm (`tools/check_assembly.py --limits`), cách biên va chạm ≥ 2,5°. Trên tay thật: quay từ từ từng servo tới khi chạm cơ khí, lùi lại 3–5° |
| 7 | `CTRL_BASE_DEG`, `CTRL_SHOULDER_DEG`, `CTRL_ELBOW_DEG` | Tầm lệnh servo thật. IK từ chối mọi điểm cần lệnh ngoài tầm này. Cận dưới `CTRL_ELBOW_DEG` (−62,5°) quyết định điểm kẹp nắp thấp nhất (mục 9b) và R_max trên giấy (bước 7). **Kiểm tra trên tay thật trước tiên** |
| 7b | `CAP_GRASP_Z`, `CONTACT_FRICTION` | Bài phân loại (mục 9b): đo z thực của tâm ngón ở tư thế kẹp nắp; hệ số ma sát nắp–ngón chưa đo |
| 8 | `MEASURED_MASS_G` | Cân từng chi tiết sau khi in, điền vào dict. Mặc định đang giả định in đặc 100% (cận trên) |
| 9 | `SERVO_ERR_SAT_DEG` | Ra lệnh bước 30° và quay video 240 fps (hoặc đọc chiết áp bên trong servo, theo tài liệu Adafruit [12] trong đề cương); chỉnh tham số tới khi đáp ứng mô phỏng khớp với đo |
| 10 | `BACKLASH_DEG` | Giữ cố định servo, lắc nhẹ đầu tay và đo độ dơ ở đầu công tác; chia cho cánh tay đòn để ra độ dơ góc |
| 11 | `BASE_GEAR_RATIO` | Đếm răng: gearmast 50 / gearservo 25 = 2,0 (bản `22DENTI` thì 50/22) |
| 12 | `PEN_TIP_FRICTION`, `INK_GAP` | Ma sát ngòi–giấy và khe tối đa vẫn ra mực; ảnh hưởng tới lệch thẳng và chỗ nét đứt |

Biên dạng ngón, đế và khối giữ chốt nằm trong `gripper_shape.py`, sinh từ ảnh chụp bằng:

```bash
python tools/gripper_from_photos.py --closed "<anh dong, chup tu tren xuong>.png" \
    --open "<anh mo, chup tu tren xuong>.png" --bottom "<anh mo, chup tu duoi len>.png"
```

Các ảnh này **không có trong repo**. Tọa độ pixel của các tâm ốc trong script được đọc trên đúng các ảnh đã dùng; ảnh khác sẽ cần đọc lại.

✅ **Kiểm tra sau mỗi lần sửa `params.py`** (theo đúng thứ tự):

```bash
python build_model.py --stl-dir "$STL_DIR"   # các dòng "lech" vẫn phải <= 0.05 mm; 2 dòng "o home" hinh hoc ~ FK
python check_kinematics.py                    # [1] và [2] vẫn phải PASS như bước 7
python check_kinematics.py --pen              # đọc lại bảng R_max
python run_sim.py --pen                       # giảm --R nếu gặp IKError
python tools/check_assembly.py --stl-dir "$STL_DIR"   # nếu đã sửa kích thước gripper
```

Sau khi hiệu chỉnh, các số "Kết quả mong đợi" ở bước 5–9 **sẽ thay đổi**; đó là điều bình thường.
Riêng tiêu chí [1] và [2] ở bước 7 không phụ thuộc tham số và phải luôn PASS.

Cách kiểm tra mô hình đã sát thực tế: chạy cùng một quỹ đạo trên tay máy thật, rồi so ảnh tờ vẽ với `pen_trace.png` và so 3 chỉ số đo.

## 11. Giới hạn của mô hình (ghi rõ trong báo cáo)

Chung:
- Các chi tiết in là **vật rắn tuyệt đối**: không mô hình hóa độ đàn hồi của PLA. Rung do uốn khâu sẽ không xuất hiện.
- Servo được mô hình là bộ P bão hòa mô-men cộng đường đặc tính mô-men–tốc độ của động cơ DC (đặt bằng `damping`).
  - Bộ điều khiển thật bên trong MG996R/SG90 không được công bố.
  - Không mô hình vùng chết và độ phân giải xung.
- **Va chạm**:
  - Mô hình bút chỉ có va chạm ngòi–giấy.
  - Mô hình phân loại (mục 9b) có va chạm giữa tấm kẹp của ngón, nắp và đồ vật.
  - Va chạm giữa các khâu được kiểm tra ngoài (bằng `tools/check_assembly.py`) và được loại trừ bằng giới hạn khớp.
- Servo (MG996R ×3, SG90) không có trong STL. Chúng được vẽ bằng hộp kích thước datasheet, đặt đúng trục quay; vị trí dọc trục chỉ gần đúng.
- Khối lượng ốc vít, thanh ren, bi và bạc đạn 606zz không được tính.

Riêng gripper:
- **Hình dạng là khối lăng trụ** (biên dạng 2D đùn theo chiều dày) theo các lớp chiều dày bạn đã xác nhận. Bo cạnh, đầu ốc, càng servo không được vẽ.
  - Biên dạng lấy từ ảnh. Khớp đồng dạng giữa vị trí ốc trong mô hình và tâm ốc trên 3 ảnh có sai số RMS 1,16 / 1,50 / 0,52 mm.
  - Kích thước cơ cấu (khoảng cách trục, khâu, W–Q...) lấy từ số đo thước kẹp, không lấy từ ảnh.
- `GRIP_CENTER_Y` = 0,48 mm là **giả định** (mục 10).
- **Gắp vật chỉ có trong mô hình phân loại** (`mk2_sort.xml`, mục 9b).
  - Ở đó mặt kẹp của 2 ngón có các tấm hộp mỏng có va chạm, đặt theo biên dạng ảnh.
  - Trong `mk2.xml` và `mk2_pen.xml`, các chi tiết gripper vẫn không có va chạm (`contype = 0`).
- **TCP gripper cố định** tại điểm kẹp khi đóng. Khi mở, ngón chuyển động theo cung tròn bán kính 35 mm, nên mũi ngón lùi về phía cổ tay (tới 35 mm khi mở 90°). TCP không đi theo mũi ngón.
- **Bút gắn cứng vào cổ tay** (mô hình `--pen`). Ngón được đặt đúng góc kẹp 4,29° nhưng không mô phỏng lực kẹp, ma sát hay trượt bút trong kẹp. Không có lò xo ở bút. Tờ giấy là vật rắn, không mô phỏng độ lún của giấy.

## 12. Port sang ESP32

`mk2_kinematics.py` chỉ dùng `math`, nên có thể chép nguyên công thức sang C/C++ (dùng `float`). Các hàm cần port:

- `ik()`: atan2, acos, chọn nghiệm khuỷu trên, **bù độ lệch ngang `DY`** = 0,48 mm (`th1 = atan2(y, x) − atan2(DY, r)`).
- Hằng số đầu công tác `DX`, `DZ` (mm, so với tâm cổ tay W) phụ thuộc dụng cụ, tương ứng `set_tool()`:
  - Gripper: (132,69; 0,50).
  - Ngòi bút: (110,59; −89,50).
  - Tâm nắp chai khi kẹp (bài phân loại, `set_tool("cap")`): (110,00; 0,50).
- `to_servo()`: lớp ánh xạ với offset, chiều quay và hộp số 2:1.
- `within_limits()`: kiểm tra giới hạn θ1, θ2, φ, φ − θ2, tầm lệnh `CTRL_*_DEG` và tầm servo trước khi gửi lệnh.

Để đối chiếu float (ESP32) với double (PC) như đề cương yêu cầu, hãy so cột `servo*_deg` trong `results/log_M2.csv` với log UART của ESP32 cho cùng một quỹ đạo.

## 13. Xử lý lỗi thường gặp

| Lỗi / hiện tượng | Nguyên nhân | Cách xử lý |
|---|---|---|
| `ModuleNotFoundError: No module named 'mujoco'` | Chưa kích hoạt venv | `source ~/venvs/mk2/bin/activate`, kiểm tra `which python` (bước 3.3) |
| `No matching distribution found for mujoco==3.2.3` | Python không phải 3.8, hoặc không phải x86_64 | Quay lại bước 2.1 |
| `Khong tim thay .../EBAmk2_xxx.STL` | `--stl-dir` sai thư mục, hoặc đuôi file là `.stl` chữ thường | Làm lại phần tìm `STL_DIR` và lệnh kiểm tra 13 file ở bước 4 |
| `RuntimeError: Hinh hoc khong khop` | `params.py` bị sửa sai phần hình học, hoặc STL khác bản gốc | Khôi phục `params.py` (`git diff params.py`), dùng STL từ bước 4 |
| `params.TCP_W/PEN_GRIP_W/PEN_GRIP_DEG khong khop hinh hoc gripper` | Đã sửa `GRIP_*`, `PEN_*` hoặc `gripper_shape.py` | Chép các dòng script in ra vào `params.py`, chạy lại bước 5 |
| `luoi lang tru khong kin`, `ear clipping that bai`, `tam giac hoa sai` | Biên dạng trong `gripper_shape.py` tự cắt nhau, hoặc kích thước `GRIP_*` làm các lớp chồng lên nhau | Khôi phục `gripper_shape.py`/`params.py` (`git diff`) |
| `run_sim.py la bai VE hinh, can but` | Chạy `run_sim.py` không có `--pen` | `python run_sim.py --pen` |
| `Chua co model/mk2.xml` (hoặc `mk2_pen.xml`) | Chưa chạy bước 5 | Chạy `build_model.py` |
| `Hinh ve (...) nam ngoai to giay` | `--cx`/`--cy`/`--R` làm hình vẽ tràn khỏi tờ A4 | Giảm `--R`, hoặc sửa `PAPER_CENTER` rồi build lại |
| `IKError: th2=... ngoai`, `phi=... ngoai`, `phi-th2=... ngoai`, `lenh CTRL_..._DEG=... ngoai`, `diem ngoai tam voi` | Hình vẽ vượt vùng làm việc, giới hạn khớp hoặc tầm lệnh servo | Giảm `--R` hoặc đổi `--cx` theo bảng `R_max` ở bước 7 (`--pen`) |
| `ngoi cham giay < 100%` hoặc `so net muc > 1` | Ngòi nhấc khỏi giấy giữa chừng (nét đứt) | Kiểm tra `--z` (phải bằng mặt giấy 1 mm), giảm `--vmax/--amax` |
| `-> VUOT gioi han, can gian thoi gian` | Quỹ đạo đòi tốc độ servo lớn hơn MG996R làm được. Đây là **cảnh báo**, mô phỏng vẫn chạy nhưng kết quả không thực tế | Giảm `--vmax`/`--amax`/`--jmax` |
| `WARNING: Nan, Inf or huge value in QACC` | Mô phỏng mất ổn định (thường do đặt `GRIPPER_ARMATURE` = 0) | Khôi phục `GRIPPER_ARMATURE = 1e-6`, build lại |
| 2 ngón xuyên qua thân bút trong viewer `mk2_pen.xml` | Viewer khởi động với `servo_gripper` = 0 (đóng) | Panel Simulation → **Load key** (bước 6.2) |
| Viewer không mở / lỗi GLFW / rất chậm | Driver đồ họa đang dùng `llvmpipe` hoặc không hoạt động | Làm lại bước 2.2. Không cần cài `libglfw3` qua apt vì gói `glfw` của pip đã kèm sẵn thư viện |
| `khong chup duoc anh ink_M0.png`, lỗi `EGL`/`OSMesa`/`GLFWError`, hoặc `Exception ignored in: <function Renderer.__del__ ...>` | Không có OpenGL (ví dụ chạy qua SSH không có màn hình), hoặc biến `MUJOCO_GL` đang được đặt | `unset MUJOCO_GL` rồi thử lại. Script chỉ thử dựng ảnh một lần rồi bỏ qua; các file khác vẫn được ghi bình thường |
| `Khong voi toi '...'` khi chạy `run_sort.py` | Bố trí trong `params.py` mục 8 (vị trí kệ, hộp, vùng vứt bỏ, `CAP_GRASP_Z`) nằm ngoài vùng làm việc, hoặc `CTRL_ELBOW_DEG` bị đổi về −57,3° | Chạy `python tools/check_sort_layout.py`, chọn vị trí trong dải bán kính in ở mục [3] |
| `params.CAP_GRIP_W/CAP_GRIP_DEG khong khop hinh hoc gripper` | Đã sửa `GRIP_CAP_GY`, `CAP_DIAMETER`, `GRIP_*` hoặc biên dạng ngón | Chép 2 dòng script in ra vào `params.py`, chạy lại bước 5 |
| `mj_makeConstraint: constraint Jacobian mis-allocation` | Mô hình có nhiều bậc tự do bị nạp ở chế độ Jacobian thưa (MuJoCo 3.2.3) | `mk2_sort.xml` đã đặt `jacobian="dense"`; nếu tự thêm nắp/vật, giữ thuộc tính này |
| `run_sort.py` báo nắp nằm ở `ngoai` | Nắp nảy khỏi ngăn hoặc tuột khỏi kẹp | Xem cột `slip_max_mm`, `final_pos` trong `results/sort/caps.csv`; xem lại bằng `--view` |
| Viewer chạy giật | Lưới STL đầy đủ khá nặng | Tắt hiển thị bóng trong panel Rendering, hoặc thử chạy trên NVIDIA (bước 2.2, tùy chọn) |

## 14. Nguồn

- STL và BOM: daGHIZmo, *EEZYbotARM Mk2*, Thingiverse thing:1454048 (CC BY-NC), https://www.thingiverse.com/thing:1454048. Kích thước đo bằng `tools/holes.py`.
  Bản sao STL bên thứ ba dùng để kiểm chứng: https://github.com/mchora2/EEZYbotARMMK2
- Gripper thật: số đo thước kẹp và xác nhận cấu tạo của chủ dự án (2026-10-16), cùng 6 ảnh chụp gripper trong tài liệu dự án (không có trong repo).
  - Các giá trị đo được ghi nhãn `[DO-THAT]` trong `params.py`.
  - Thuật toán trích biên dạng (`tools/gripper_from_photos.py`):
    - S. Umeyama, "Least-squares estimation of transformation parameters between two point patterns", *IEEE Trans. PAMI* 13(4), 376–380, 1991 (khớp đồng dạng).
    - D. H. Douglas & T. K. Peucker, "Algorithms for the reduction of the number of points required to represent a digitized line or its caricature", *Cartographica* 10(2), 112–122, 1973 (đơn giản hóa biên dạng).
  - Tam giác hóa đa giác (`gripper.py`): D. Eberly, "Triangulation by Ear Clipping", Geometric Tools, https://www.geometrictools.com/Documentation/TriangulationByEarClipping.pdf
- MG996R: TowerPro, https://towerpro.com.tw/product/mg996r/ (9,4 kgf·cm và 0,19 s/60° ở 4,8 V; 11 kgf·cm và 0,15 s/60° ở 6 V; 55 g; 40,7 × 19,7 × 42,9 mm). Trang này ghi 0,17 s ở một bảng khác; gói này dùng 0,19 s như đề cương.
- SG90: TowerPro SG90 datasheet (bản lưu tại https://tams.informatik.uni-hamburg.de/lehre/2023ss/vorlesung/es/doc/sg90.pdf): 1,8 kgf·cm, 0,1 s/60°, 9 g, ~22,2 × 11,8 × 31 mm.
- Khổ giấy A4 210 × 297 mm: ISO 216.
- Mật độ PLA 1,24 g/cm³: NatureWorks, *Ingeo 4043D Technical Data Sheet* (ASTM D792).
- MuJoCo: XML Reference (`equality/connect`, `equality/joint`, `actuator/position`, `keyframe`, `armature`, `margin`/`gap` của geom, `contype`/`conaffinity`) và mục Backlash trong Modeling, https://mujoco.readthedocs.io
  - Bài phân loại dùng thêm: `freejoint`, `option/cone`, `option/jacobian`, `option/flag/multiccd`, `solref`/`solimp` của geom, `mesh/vertex` (lưới lồi khai báo trong XML).
- Bài phân loại màu (mục 9b), repo firmware https://github.com/NithanNguyen/eezybot_mk2_robot (commit `a172b69`):
  - `data/color/model.json`, `data/color/test.csv`, `reports/color/eval_test.json`;
  - `lib/ColorClassifier/ColorMath.h` (thuật toán), `lib/ColorClassifier/ColorClassifier.h` (`READ_INTERVAL_MS`), `src/robot/sorter.h` (`POLL_MS`);
  - `src/robot/color_console.cpp` (định dạng dòng `CLS`/`ERR`), `include/arm_config.h` (giới hạn quỹ đạo, `DROP_HEIGHT_ABOVE_BIN`).
- Kích thước kệ, module GY-33, nắp chai, độ dày thành hộp, quy tắc thả 50 mm và chu trình: yêu cầu của chủ dự án (2026-10-23).
- MuJoCo Python bindings (lệnh `python -m mujoco.viewer --mjcf=...`, `launch_passive`, `user_scn`): https://mujoco.readthedocs.io/en/stable/python.html
- Phím tắt viewer: MuJoCo "Code samples → simulate", https://mujoco.readthedocs.io/en/stable/programming/samples.html
- contourpy (dò đường biên trên ảnh raster): https://contourpy.readthedocs.io
- Phiên bản Python hỗ trợ của từng gói: trang PyPI của gói (ví dụ https://pypi.org/project/mujoco/3.2.3/, https://pypi.org/project/ruckig/0.12.2/), mục "Download files" và "Requires: Python".
- NVIDIA PRIME Render Offload: NVIDIA Linux driver README, https://download.nvidia.com/XFree86/Linux-x86_64/470.82.00/README/primerenderoffload.html
- Ruckig: Berscheid & Kröger, RSS 2021 (tài liệu [2] trong đề cương).
