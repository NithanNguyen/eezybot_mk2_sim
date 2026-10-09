# Mô phỏng EEZYbotARM Mk2 — Python + MuJoCo

### FAST RUN
```bash
source ~/venvs/mk2/bin/activate
cd ~/eezybot_mk2_sim

python run_sim.py --view      # xem tay máy vẽ theo thời gian thực
python run_sim.py             # chạy không cửa sổ, ghi kết quả vào results/
python -m mujoco.viewer --mjcf=model/mk2.xml   # điều khiển tay bằng thanh trượt
python check_kinematics.py    # kiểm chứng động học (khoảng 30 giây)
```



Bộ mã mô phỏng cho đồ án CE212 "Tay máy robot 3-DOF phân loại sản phẩm". Gói này gồm:

- Mô hình MuJoCo của Mk2 dựng từ **chính các file STL gốc**, có **3 vòng hình bình hành kín**.
- Mô hình servo MG996R.
- Thư viện FK/IK giải tích kèm lớp ánh xạ góc servo.
- Thí nghiệm M0/M2 vẽ ngôi sao + đường tròn, đúng như mô tả trong đề cương.

> **Trạng thái kiểm chứng (2026-10-09).** Toàn bộ quy trình bước 3 → 9 đã được chạy lại từ đầu trong venv sạch
> **Python 3.8.20 + `pip install -r requirements.txt`**, với bộ STL `EBAmk2_*.STL` (xem bước 4). Mọi số liệu "Kết quả mong đợi"
> bên dưới là output thật của lần chạy đó, và 3 ảnh trong `assets/simulation_results/` trùng **từng byte** (md5) với ảnh vừa sinh ra.
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
| Động lực học | MuJoCo 3.2.3 | Hình bình hành kín (`equality/connect`), quán tính từ STL, servo bão hòa mô-men, độ rơ, lệnh giữ 20 ms |

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

`build_model.py` cần **12 file** sau, tên viết **đúng chữ hoa/thường** (Linux phân biệt `.STL` và `.stl`):

```
EBAmk2_001_base  002_mainarm  003_varm  004_link135  005_link135angled  006_horarm__
EBAmk2_007_trialink  008_link147_new  009_trialinkfront  011_gearmast  012_mainbase  013_lower_base
```

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
find ~/mk2_stl -iname "EBAmk2_001_base.stl"
# Ví dụ in ra /home/<tên-bạn>/mk2_stl/files/EBAmk2_001_base.STL  -> thư mục là ~/mk2_stl/files
STL_DIR=$(dirname "$(find ~/mk2_stl -name 'EBAmk2_001_base.STL' | head -1)")
echo "$STL_DIR"
```

✅ **Kiểm tra — đủ 12 file:**

```bash
miss=0; for f in 001_base 002_mainarm 003_varm 004_link135 005_link135angled 006_horarm__ \
  007_trialink 008_link147_new 009_trialinkfront 011_gearmast 012_mainbase 013_lower_base; do
  if [ -f "$STL_DIR/EBAmk2_$f.STL" ]; then echo "OK    EBAmk2_$f.STL"; else echo "THIEU EBAmk2_$f.STL"; miss=1; fi
done; [ $miss -eq 0 ] && echo PASS || echo FAIL
```

PASS khi có 12 dòng `OK` và dòng cuối `PASS`.
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
chi tiet | khoi luong (g) | the tich (cm3) | ghi chu
EBAmk2_001_base | 178.2 | 143.7 | luoi da va lo
...
TONG khoi luong chuyen dong: 552.3 g  (FILL_FACTOR=1.0)

Servo: tau_stall=0.922 N m, w0=5.51 rad/s, kp=10.563 N m/rad, b=0.1673 N m s/rad
Da ghi .../eezybot_mk2_sim/model/mk2.xml
```

Các dòng "lech" là phép kiểm tra hình học: lỗ thứ 3 của mỗi chi tiết phải rơi đúng khớp tính từ `params.py`. Nếu lệch quá 1 mm, script sẽ dừng.

✅ **Kiểm tra — MuJoCo nạp được mô hình:**

```bash
python -c "import mujoco; m=mujoco.MjModel.from_xml_path('model/mk2.xml'); print('nbody',m.nbody,'njnt',m.njnt,'neq',m.neq,'nu',m.nu,'nmesh',m.nmesh)"
ls model/meshes | wc -l
```

Kết quả mong đợi:

```
nbody 10 njnt 9 neq 3 nu 3 nmesh 12
12
```

PASS khi: mọi dòng "lech" ≤ 0.05 mm, tổng khối lượng `552.3 g` (nếu chưa sửa `params.py`), và dòng nạp mô hình in đúng như trên.
Ý nghĩa: 10 body (world + 9 khâu), 9 khớp, 3 ràng buộc vòng kín, 3 servo, 12 lưới STL.
Nếu bạn đặt `BACKLASH_DEG > 0` trong `params.py`, mỗi khớp dẫn động có thêm một khớp "play", nên `njnt` sẽ là 12; các số khác không đổi.

**Quan trọng:** sau mỗi lần sửa `params.py`, phải chạy lại `build_model.py`.

## 6. Xem mô hình và điều khiển bằng tay

Điều kiện: bước 2.2 PASS (không phải `llvmpipe`).

```bash
python -m mujoco.viewer --mjcf=model/mk2.xml
```

✅ **Kiểm tra bằng mắt — PASS khi:**
1. Cửa sổ mở ra, thấy tay máy đứng trên nền ô caro, terminal không báo lỗi.
2. Ở tư thế ban đầu (các thanh điều khiển = 0): tay chính dựng đứng, tay ngang nằm ngang.
3. Mở panel bên phải → mục **Control** (phím tắt `Alt`+`C`), kéo 3 thanh `servo_base`, `servo_shoulder`, `servo_elbow`; tay máy bám theo và **cổ tay luôn giữ nằm ngang** nhờ hình bình hành.

Đơn vị thanh trượt là rad, phạm vi ±1.571 (±90°). Riêng `servo_base` là góc **trục servo**; qua hộp số 2:1, khớp đế chỉ quay một nửa góc này.

Thao tác chuột/phím (theo tài liệu MuJoCo "simulate"):
- `Space`: chạy/tạm dừng.
- Nhấp đúp vào một chi tiết: chọn chi tiết đó.
- Giữ `Ctrl` + kéo **chuột trái**: xoay chi tiết đã chọn (tác dụng mô-men).
- Giữ `Ctrl` + kéo **chuột phải**: đẩy chi tiết trong mặt phẳng đứng (tác dụng lực); thêm `Shift` để đẩy trong mặt phẳng ngang.

Đóng cửa sổ để thoát.

## 7. Bước kiểm chứng động học (mục "Mô phỏng" trong đề cương)

```bash
python check_kinematics.py
```

Script chạy khoảng 30 giây trên máy thử (phần [4] quét vùng làm việc nên lâu nhất). Kết quả mong đợi với tham số mặc định:

```
CANH BAO: TOOL_OFFSET va TOOL_MASS_G trong params.py la gia tri tam [GIA-DINH]. Hay do tren tay may that roi sua lai.
[1] FK(IK(p)) tren 5000 diem: max |FK(IK(p))-p| = 1.04e-13 mm, max |IK(FK(q))-q| = 1.29e-12 mrad
[2] MuJoCo (vong kin, g=0) vs FK giai tich, 40 tu the: max sai lech = 0.005 mm, max vi pham rang buoc vong = 0.0000 mm
[3] Do vong do trong luc (servo P huu han, params.SERVO_ERR_SAT_DEG):
    q=(0, 90, 0): lech goc (deg) = [-0.    0.02 -0.35], lech dau but = 0.93 mm, ...
    q=(0, 60, -30): lech goc (deg) = [ 0.   -0.56 -0.29], lech dau but = 1.51 mm, ...
    q=(0, 45, -60): lech goc (deg) = [ 0.   -0.79 -0.15], lech dau but = 1.81 mm, ...
    q=(0, 110, 0): lech goc (deg) = [-0.    0.41 -0.36], lech dau but = 1.56 mm, ...
[4] Hinh ve lon nhat (sao + tron ban kinh R) theo do cao mat giay z (mm), cach gioi han >= 3 deg:
    ...
    z_giay =   60 mm: tam x = 180 mm, R_max =  84.3 mm
    ...
Da luu .../results/workspace_rz.png
```

Dòng `CANH BAO` là nhắc nhở bình thường, không phải lỗi.

✅ **Kiểm tra — PASS khi:**

| Dòng | Tiêu chí | Ý nghĩa nếu FAIL |
|---|---|---|
| [1] | cả hai sai số < 1e-9 | Công thức IK không còn là nghịch đảo của FK (đã sửa nhầm `mk2_kinematics.py`?) |
| [2] | sai lệch < 0.05 mm **và** vi phạm vòng ≈ 0 | Lớp ánh xạ hình bình hành hoặc `model/mk2.xml` không khớp `params.py` → chạy lại bước 5 |
| [3] | độ võng đầu bút cỡ 1–2 mm | Lớn hơn nhiều: `SERVO_ERR_SAT_DEG` hoặc khối lượng bị đổi |
| [4] | có bảng `R_max`, có file `results/workspace_rz.png` | — |

Ý nghĩa từng dòng:
1. Công thức IK là nghịch đảo đúng của FK.
2. Mô hình vật lý có vòng kín cho cùng vị trí đầu bút với công thức giải tích. Điều này chứng minh **lớp ánh xạ hình bình hành** là đúng.
3. Servo là bộ điều khiển P hữu hạn, nên dưới trọng lực đầu bút võng xuống khoảng 1–2 mm. Đây là hiệu ứng thật mà mô hình động học thuần không thấy được.
4. Bảng để **chọn R và độ cao mặt giấy**: chọn R nhỏ hơn hẳn R_max để còn khoảng dự phòng. Ảnh vùng làm việc được lưu ở `results/workspace_rz.png`.

## 8. Chạy thí nghiệm M0 vs M2

```bash
python run_sim.py                    # mặc định: R=40, tâm (180,0), giấy z=60 mm
```

Kết quả mong đợi với tham số mặc định (chạy khoảng 3 giây):

```
Duong di L = 541.9 mm (ly thuyet 13.55R = 542.0); T(M2) = 5.44 s; v(M0) = L/T = 99.6 mm/s
[M0] toc do servo lon nhat = [1.42 0.54 0.35] rad/s (MG996R khong tai 5.51 rad/s @4.8V) -> OK
[M2] toc do servo lon nhat = [2.14 0.82 0.5 ] rad/s (MG996R khong tai 5.51 rad/s @4.8V) -> OK
[M0] max vi pham rang buoc vong = 0.013 mm, max |mo-men servo| = [0.109 0.17  0.064] Nm
[M2] max vi pham rang buoc vong = 0.015 mm, max |mo-men servo| = [0.164 0.165 0.071] Nm

Chi so du doan tu mo phong (cung T), do tren 5 mui sao / duong tron / 10 canh:
PP    vot lo TB  vot lo max  sai lech dinh TB  do tron  lech thang max   (mm)
M0        -0.98       -0.61              1.16     0.68            0.65
M2         0.05        0.23              0.09     0.75            0.64
```

✅ **Kiểm tra:**

```bash
ls results/cmd_derivatives.png results/pen_trace.png results/log_M0.csv results/log_M2.csv
wc -l results/log_M0.csv results/log_M2.csv          # -> 298 dòng mỗi file (1 dòng tiêu đề + 297 mẫu)
md5sum results/cmd_derivatives.png assets/simulation_results/cmd_derivatives.png
md5sum results/pen_trace.png       assets/simulation_results/pen_trace.png
```

PASS khi:
- cả hai dòng tốc độ servo kết thúc bằng `-> OK`;
- vi phạm ràng buộc vòng < 0.1 mm (vòng kín được giữ trong lúc chuyển động);
- đủ 4 file, mỗi CSV 298 dòng;
- bảng chỉ số trùng bảng trên (với tham số mặc định).

Hai lệnh `md5sum` là phép so khắt khe nhất: nếu mỗi cặp cho cùng một mã, ảnh của bạn trùng từng byte với ảnh mẫu.
Nếu mã khác nhưng bảng số liệu vẫn trùng, mở hai ảnh ra so bằng mắt; khác biệt nhỏ ở mức điểm ảnh có thể do khác CPU hoặc phông chữ, chưa phải lỗi.

Các tùy chọn khác:

```bash
python run_sim.py --view             # xem robot vẽ theo thời gian thực (cần bước 6 PASS)
python run_sim.py --R 30 --z 60 --vmax 150 --amax 1500 --jmax 30000
```

Với `--view`, viewer chạy ở chế độ "passive" nên phím `Space` (tạm dừng) không có tác dụng (theo tài liệu MuJoCo); script tự đóng cửa sổ khi chạy xong.
Với `--R 30`, bảng chỉ số mong đợi là `M0 -1.02 -0.65 1.09 0.57 0.83` và `M2 0.04 0.34 0.14 0.67 0.61`.

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

## 9. Chụp ảnh tư thế cho báo cáo

Điều kiện: bước 2.2 PASS.

```bash
python snapshot.py --xyz 180 0 60 --out results/pose.png     # đặt đầu bút tại (180, 0, 60) mm bằng IK
python snapshot.py --servo 90 90 90 --out results/home.png   # đặt góc 3 servo trực tiếp (độ)
```

Kết quả mong đợi của lệnh thứ nhất:

```
servo (deg): [90.   69.24 38.82] | FK dau but (mm): [180.   0.  60.]
MuJoCo dau but (mm): [180.46  -0.    59.35]
Da luu results/pose.png
```

✅ **Kiểm tra — PASS khi:** có file `results/pose.png` và ảnh giống `assets/simulation_results/pose_xyz_180_0_60.png`.
Chênh lệch khoảng 0.8 mm giữa FK và MuJoCo là độ võng do trọng lực, cùng bản chất với dòng [3] ở bước 7, không phải lỗi.

## 10. Hiệu chỉnh cho giống thực tế (quan trọng nhất)

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

✅ **Kiểm tra sau mỗi lần sửa `params.py`** (theo đúng thứ tự):

```bash
python build_model.py --stl-dir "$STL_DIR"   # các dòng "lech" vẫn phải <= 0.05 mm
python check_kinematics.py                    # [1] và [2] vẫn phải PASS như bước 7
python run_sim.py                             # đọc lại bảng R_max ở bước 7 nếu gặp IKError
```

Sau khi hiệu chỉnh, các số "Kết quả mong đợi" ở bước 5–9 **sẽ thay đổi**; đó là điều bình thường.
Riêng tiêu chí [1] và [2] ở bước 7 không phụ thuộc tham số và phải luôn PASS.

Cách kiểm tra mô hình đã sát thực tế: chạy cùng một quỹ đạo trên tay máy thật, rồi so ảnh tờ vẽ với `pen_trace.png` và so 3 chỉ số đo.

## 11. Giới hạn của mô hình (ghi rõ trong báo cáo)

- Các chi tiết in là **vật rắn tuyệt đối**: không mô hình hóa độ đàn hồi của PLA. Rung do uốn khâu sẽ không xuất hiện.
- Servo được mô hình là bộ P bão hòa mô-men cộng đường đặc tính mô-men–tốc độ của động cơ DC (đặt bằng `damping`). Bộ điều khiển thật bên trong MG996R không được công bố. Không mô hình vùng chết và độ phân giải xung.
- Không có va chạm (giữa các khâu, bút–giấy). Lực lò xo của giá bút không được mô hình hóa.
- Vị trí các chi tiết theo phương ngang (dọc trục khớp) trong `build_model.py` chỉ là gần đúng. Sai lệch này chỉ ảnh hưởng hình hiển thị và rất ít đến quán tính.
- Khối lượng ốc vít, thanh ren và bi không được tính. Hai servo vai được đặt gần đúng hai bên trục vai.

## 12. Port sang ESP32

`mk2_kinematics.py` chỉ dùng `math`, nên có thể chép nguyên công thức sang C/C++ (dùng `float`). Các hàm cần port:

- `ik()`: atan2, acos, chọn nghiệm khuỷu trên.
- `to_servo()`: lớp ánh xạ với offset, chiều quay và hộp số 2:1.
- `within_limits()`: kiểm tra giới hạn trước khi gửi lệnh.

Để đối chiếu float (ESP32) với double (PC) như đề cương yêu cầu, hãy so cột `servo*_deg` trong `results/log_M2.csv` với log UART của ESP32 cho cùng một quỹ đạo.

## 13. Xử lý lỗi thường gặp

| Lỗi / hiện tượng | Nguyên nhân | Cách xử lý |
|---|---|---|
| `ModuleNotFoundError: No module named 'mujoco'` | Chưa kích hoạt venv | `source ~/venvs/mk2/bin/activate`, kiểm tra `which python` (bước 3.3) |
| `No matching distribution found for mujoco==3.2.3` | Python không phải 3.8, hoặc không phải x86_64 | Quay lại bước 2.1 |
| `Khong tim thay .../EBAmk2_xxx.STL` | `--stl-dir` sai thư mục, hoặc đuôi file là `.stl` chữ thường | Làm lại phần tìm `STL_DIR` và lệnh kiểm tra 12 file ở bước 4 |
| `RuntimeError: Hinh hoc khong khop` | `params.py` bị sửa sai phần hình học, hoặc STL khác bản gốc | Khôi phục `params.py` (`git diff params.py`), dùng STL từ bước 4 |
| `Chua co model/mk2.xml` | Chưa chạy bước 5 | Chạy `build_model.py` |
| `IKError: th2=... ngoai (...)`, `IKError: phi-th2=... ngoai (...)`, `IKError: diem ngoai tam voi` | Hình vẽ vượt vùng làm việc hoặc giới hạn khớp | Giảm `--R` hoặc đổi `--z`, `--cx` theo bảng `R_max` ở bước 7 |
| `-> VUOT gioi han, can gian thoi gian` | Quỹ đạo đòi tốc độ servo lớn hơn MG996R làm được. Đây là **cảnh báo**, mô phỏng vẫn chạy nhưng kết quả không thực tế | Giảm `--vmax`/`--amax`/`--jmax` |
| Viewer không mở / lỗi GLFW / rất chậm | Driver đồ họa đang dùng `llvmpipe` hoặc không hoạt động | Làm lại bước 2.2. Không cần cài `libglfw3` qua apt vì gói `glfw` của pip đã kèm sẵn thư viện |
| Lỗi `EGL` hoặc `OSMesa` khi chạy `snapshot.py` | Biến môi trường `MUJOCO_GL` đang được đặt | Chạy `unset MUJOCO_GL` rồi thử lại (mặc định MuJoCo dùng GLFW) |
| Viewer chạy giật | Lưới STL đầy đủ khá nặng | Tắt hiển thị bóng trong panel Rendering, hoặc thử chạy trên NVIDIA (bước 2.2, tùy chọn) |

## 14. Nguồn

- STL và BOM: daGHIZmo, *EEZYbotARM Mk2*, Thingiverse thing:1454048 (CC BY-NC), https://www.thingiverse.com/thing:1454048. Kích thước đo bằng `tools/holes.py`.
  Bản sao STL bên thứ ba dùng để kiểm chứng: https://github.com/mchora2/EEZYbotARMMK2
- MG996R: TowerPro, https://towerpro.com.tw/product/mg996r/ (9,4 kgf·cm và 0,19 s/60° ở 4,8 V; 11 kgf·cm và 0,15 s/60° ở 6 V; 55 g). Trang này ghi 0,17 s ở một bảng khác; gói này dùng 0,19 s như đề cương.
- Mật độ PLA 1,24 g/cm³: NatureWorks, *Ingeo 4043D Technical Data Sheet* (ASTM D792).
- MuJoCo: XML Reference (`equality/connect`, `actuator/position`, `armature`, `frictionloss`) và mục Backlash trong Modeling, https://mujoco.readthedocs.io
- MuJoCo Python bindings (lệnh `python -m mujoco.viewer --mjcf=...`, `launch_passive`): https://mujoco.readthedocs.io/en/stable/python.html
- Phím tắt viewer: MuJoCo "Code samples → simulate", https://mujoco.readthedocs.io/en/stable/programming/samples.html
- Phiên bản Python hỗ trợ của từng gói: trang PyPI của gói (ví dụ https://pypi.org/project/mujoco/3.2.3/, https://pypi.org/project/ruckig/0.12.2/), mục "Download files" và "Requires: Python".
- NVIDIA PRIME Render Offload: NVIDIA Linux driver README, https://download.nvidia.com/XFree86/Linux-x86_64/470.82.00/README/primerenderoffload.html
- Ruckig: Berscheid & Kröger, RSS 2021 (tài liệu [2] trong đề cương).
- Giới hạn khớp mặc định (độ rộng 90°): URDF trong https://github.com/HotBlackRobotics/ntbd
