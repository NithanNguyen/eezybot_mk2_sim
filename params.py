"""
params.py — Toan bo tham so cua mo hinh EEZYbotARM Mk2.

Quy uoc don vi trong file nay: mm, gram, do (de doc/sua tay).
build_model.py va mk2_kinematics.py tu doi sang SI (m, kg, rad).

Moi tham so co ghi NGUON:
  [DO-STL]  : do tu file STL goc (Thingiverse thing:1454048) bang tools/holes.py
  [DATASHEET]: lay tu datasheet (ghi ro trang)
  [DO-THAT]  : do bang thuoc kep tren gripper that (nguoi dung, 2026-10-16)
  [ANH]      : do tren anh chup gripper that bang tools/gripper_from_photos.py
  [GIA-DINH] : gia tri tam, CHUA DO tren tay may that -> phai do va sua lai
"""

# ---------------------------------------------------------------------------
# 1. Hinh hoc co cau (mm)  — [DO-STL], khoang cach tam lo truc
# ---------------------------------------------------------------------------
L_UPPER = 135.0      # vai O -> khuyu E, 002_mainarm (tam lo 135.00)          [DO-STL]
L_FORE = 147.0       # khuyu E -> co tay W, 006_horarm (147.01)               [DO-STL]
L_TAIL = 57.0        # khuyu E -> duoi T, 006_horarm (57.00)                  [DO-STL]
L_CRANK = 57.0       # tay quay 003_varm: O -> V (57.00)                      [DO-STL]
L_LINK135 = 135.0    # 004_link135 va 005_link135angled (135.00 / 135.01)     [DO-STL]
L_LINK147 = 147.0    # 008_link147_new (147.00)                               [DO-STL]
TRI_ARM = 43.0       # canh tam giac 007_trialink, 009_trialinkfront (43.0)   [DO-STL]

# Diem co dinh F tren de quay (001_base), so voi truc vai O, trong mat phang
# (x: ve phia truoc, z: len tren). 001_base: O=(y46.43,z43.00), F=(y8.46,z63.19)
F_REL_O = (-37.97, 20.19)                                                    # [DO-STL]

# Toa do lo cua 007_trialink (he toa do rieng cua chi tiet, mm): A, B(tam), C
TRI_A = (6.42, 33.35)
TRI_B = (41.91, 9.09)
TRI_C = (77.42, 33.35)                                                        # [DO-STL]

# Chieu cao truc vai O so voi mat ban (mm). Chuoi chi tiet tu duoi len (tools/check_assembly.py):
#   012 mainbase: mat do 013 o z = 33.95
#   013 lower_base: mat tam (r < 6 mm) cao 6.30 -> 40.25; gearmast 011 (11 mm) ty len mat nay
#   (bi D6 trong ranh R3.17 con khe ~0.6 mm, nen tam la diem tua) -> mat tren gearmast 51.25
#   001_base: lo vai cao 43.00 tren mat day                               -> 94.25
LOWER_BASE_Z = 33.95                                  # [DO-STL]
H_SHOULDER = LOWER_BASE_Z + 6.30 + 11.0 + 43.00       # = 94.25 mm   [DO-STL] -> nen do lai tren tay that

# Truc vai cat truc dung (O nam ngay tren truc quay de): 001_base O.y = truc.y
SHOULDER_OFFSET_X = 0.0                                                       # [DO-STL]

# Ti so truyen banh rang de: gearmast(_full) 50 rang / gearservo 25 rang = 2.0
# (Neu ban in gearservo_22DENTI 22 rang thi doi thanh 50/22 = 2.2727)
BASE_GEAR_RATIO = 2.0                                                         # [DO-STL]

# ---------------------------------------------------------------------------
# 2. Kep (gripper) — gripper THAT tren robot (thiet ke rieng, KHONG phai kep goc 014-018)
#    Co cau 2 hinh binh hanh: moi ben 1 chi tiet "banh rang + khau duoi" quay quanh truc
#    banh rang G, 1 "khau tren" quay quanh chot C tren than, ngon noi 2 khau -> ngon TINH TIEN,
#    khong xoay. 2 banh rang bang nhau an khop 1:1; truc SG90 = truc banh rang trai.
#    Gripper thay ca 009 (trialinkfront): thap ga co lo W (dau tay ngang 006) va Q (dau 008).
#    Hinh hoc chi tiet + luoi: gripper.py; bien dang ngon/de: gripper_shape.py (tu anh).
#    He G (mm): goc = trung diem 2 truc banh rang, gx = ngang (phai khi nhin tu tren, mui ngon
#    huong len), gy = huong mui ngon, z = len, z = 0 tai mat day gripper.
# ---------------------------------------------------------------------------
GRIP_GEAR_DIST = 27.0           # tam-tam 2 truc banh rang                    [DO-THAT]
GRIP_PIVOT_DIST = 11.0          # tam-tam 2 chot khau tren (2 oc den giua)    [DO-THAT]
GRIP_FINGER_HOLES = 20.0        # tam-tam 2 lo oc tren 1 ngon                 [DO-THAT]
GRIP_LINK = 35.0                # tam-tam khau duoi (truc banh rang -> oc duoi ngon) = khau tren [DO-THAT]
GRIP_TRAVEL_DEG = 90.0          # dong (khau duoi thang, 0 do) -> mo (vuong goc canh tay, 90 do) [DO-THAT]
GRIP_THICKNESS = 15.0           # be day tong                                  [DO-THAT]
GRIP_LINK_WIDTH = 6.0           # be rong 2 khau                               [ANH]
GRIP_W_TO_GEAR = 45.0           # tam W -> duong truc banh rang, doc canh tay  [DO-THAT]
GRIP_W_HEIGHT = 7.0             # tam W (va mat giua lop banh rang/khau) cao hon mat day gripper [DO-THAT]
# Q - W (doc tay, dung) = (32.6, 28.0), thang 43 mm: nguoi dung do lai, trung hinh hoc 007/009
# (mk2_kinematics.linkage_points) -> gripper nam ngang, khong nghieng.         [DO-THAT]
GRIP_CENTER_Y = 0.48            # y the gioi cua mat phang doi xung gripper = tam kep cu 014 [GIA-DINH]
GRIP_TCP_BACK = 5.0             # TCP gap vat: giua 2 dem mui ngon luc dong, lui 5 mm tu mut ngon [QUY-UOC]
GRIPPER_GEAR_RATIO = 1.0        # goc truc SG90 / goc banh rang (truc SG90 = truc banh rang trai) [DO-THAT]
# Quan tinh rotor quy doi cua cac khop kep (kg m^2). Can > 0: banh rang/ngon nhua rat nhe,
# khong co gia tri nay mo phong mat on dinh (dt = 0.5 ms).                    [GIA-DINH]
GRIPPER_ARMATURE = 1e-6

# But ve — CHI gan khi chay voi co --pen (model/mk2_pen.xml). But dung thang, kep giua 2 ngon.
PEN_DIAMETER = 10.0             # mm                                           [YEU-CAU]
PEN_FROM_TIP = 27.0             # truc but lui vao 27 mm so voi mut ngon       [DO-THAT]
PEN_GRIP_TO_TIP = 90.0          # mm, tu diem kep (giua chieu day ngon) toi ngoi [DO-THAT]
PEN_LENGTH = 140.0              # mm, tong chieu dai but                       [GIA-DINH]
TOOL_MASS_G = 15.0              # khoi luong but                               [GIA-DINH]
PEN_TIP_FRICTION = 0.2          # he so ma sat ngoi-giay                       [GIA-DINH]

# Diem lam viec so voi tam truc co tay W, truc the gioi o tu the home (mm). Tinh tu hinh hoc
# boi gripper.py; build_model.py kiem tra lai va dung neu lech > 0.02 mm.
TCP_W = (132.69, 0.480, 0.50)   # TCP gap vat (khong but), gripper dong
PEN_GRIP_W = (110.59, 0.480, 0.50)  # diem kep but (tam but, giua chieu day ngon)
PEN_GRIP_DEG = 4.29             # goc servo kep de 2 mat kep vua cham but D10 (tu luc dong)

# Dau cong tac so voi W (mat phang canh tay: dx, dz) va do lech ngang dy (mm) — dung cho FK/IK.
# Mac dinh "gripper"; "pen" chi dung khi chay voi --pen (mk2_kinematics.set_tool).
TOOL_OFFSETS = {
    "gripper": (TCP_W[0], TCP_W[2]),
    "pen": (PEN_GRIP_W[0], PEN_GRIP_W[2] - PEN_GRIP_TO_TIP),
}
TOOL_OFFSET_Y = TCP_W[1]

# ---------------------------------------------------------------------------
# 2b. Giay ve — A4 (ISO 216: 210 x 297 mm), dat ngay tren mat ban
# ---------------------------------------------------------------------------
PAPER_SIZE = (210.0, 297.0)     # (theo x, theo y) mm                          [ISO 216]
PAPER_THICKNESS = 1.0           # mm, mat tren giay o z = 1 mm                 [YEU-CAU]
PAPER_CENTER = (290.0, 0.0)     # (x, y) tam to giay = tam hinh ve mac dinh; R_max = 22.9 mm tai day (check_kinematics.py --pen [4])
INK_WIDTH = 1.0                 # be rong net but (mm)                         [YEU-CAU]
# Ngoi coi la "cham giay" (ra muc) khi khe ngoi-giay <= INK_GAP. Tiep xuc cung trong MuJoCo
# rung vi mo +-0.015 mm; nguong nay chi anh huong viec ghi muc, khong anh huong luc tiep xuc.
INK_GAP = 0.05                  # mm                                           [GIA-DINH]
PEN_LIFT = 15.0                 # do cao nhac but khi di chuyen toi/roi diem dau (mm)

# ---------------------------------------------------------------------------
# 3. Gioi han khop (do). theta2 = goc canh tay chinh so voi phuong ngang,
#    phi = goc tay ngang (forearm) so voi phuong ngang, rel = phi - theta2.
#    Lay tu kiem tra va cham giua cac chi tiet STL (tools/check_assembly.py --limits):
#      phi <= -70 (bang [C], buoc 5 do): tay ngang 006 cham than gripper GR_body (ban kep cu: 009 o -67.5)
#      | phi > 22.5: tay quay 003 cham de 001
#      rel < -147.5: 006 cham canh tay chinh 002 | theta2 < 40: 002 cham tam giac 007
#    Gioi han duoi day cach bien va cham >= 2.5 do. Vi tri cu the van nen do tren tay that.
# ---------------------------------------------------------------------------
THETA1_LIM = (-45.0, 45.0)      # khop de (servo 0..180 qua hop so 2:1)       [ntbd URDF]
THETA2_LIM = (42.5, 125.0)                                                    # [DO-STL va cham]
PHI_LIM = (-65.0, 20.0)                                                       # [DO-STL va cham]
REL_LIM = (-145.0, -60.0)       # phi - theta2                                [DO-STL va cham]

# ---------------------------------------------------------------------------
# 4. Lop anh xa goc dong hoc -> goc servo (do). Phai hieu chuan tren phan cung.
#    servo = OFFSET + SIGN * (goc - goc_home)
#    home: theta1 = 0, theta2 = 90 (tay chinh dung), phi = 0 (tay ngang nam ngang)
# ---------------------------------------------------------------------------
SERVO_OFFSET = (90.0, 90.0, 90.0)                                             # [GIA-DINH]
SERVO_SIGN = (+1, +1, +1)                                                     # [GIA-DINH]
SERVO_RANGE = (0.0, 180.0)

CTRL_BASE_DEG     = (-90.0, 90.0)
CTRL_SHOULDER_DEG = (-57.0, 40.0) 
CTRL_ELBOW_DEG    = (-62.5, 17.5)  # gioi han tren cua phi = 20 do, de tranh cham 003 quay vao 001 base

# ---------------------------------------------------------------------------
# 5. Servo MG996R — TowerPro (https://towerpro.com.tw/product/mg996r/)
# ---------------------------------------------------------------------------
SUPPLY = "4.8V"                 # "4.8V" hoac "6V"
MG996R = {
    "4.8V": {"stall_kgfcm": 9.4, "s_per_60deg": 0.19},                        # [DATASHEET]
    "6V":   {"stall_kgfcm": 11.0, "s_per_60deg": 0.15},                       # [DATASHEET]
}
SERVO_MASS_G = 55.0                                                           # [DATASHEET]
MG996R_SIZE = (40.7, 19.7, 42.9)  # dai x rong x cao, mm (TowerPro)          [DATASHEET]
# Truc ra cua MG996R lech khoi tam than servo 9.75 mm theo chieu dai: suy tu vi tri lo truc
# vai trong cua so lap servo cua 001 va tu tam an khop gearservo/gearmast trong 012. [DO-STL]
MG996R_SHAFT_OFFSET = 9.75

# Servo kep SG90 — TowerPro SG90 datasheet: 1.8 kgf.cm, 0.1 s/60 do (4.8 V), 9 g,
# kich thuoc ~22.2 x 11.8 x 31 mm                                             [DATASHEET]
SG90 = {"stall_kgfcm": 1.8, "s_per_60deg": 0.10, "mass_g": 9.0, "size": (22.2, 11.8, 31.0)}
# Sai so goc tai do mo-men bat dau bao hoa (do cung cua vong P ben trong servo).
# Nha san xuat KHONG cong bo -> hieu chinh bang dap ung buoc do tren servo that.
SERVO_ERR_SAT_DEG = 5.0                                                       # [GIA-DINH]
SERVO_ARMATURE = 0.0            # quan tinh rotor quy doi (kg m^2)            [GIA-DINH]
SERVO_FRICTIONLOSS = 0.0        # ma sat kho (N m)                            [GIA-DINH]
BACKLASH_DEG = 0.0              # do ro moi khop dan dong; 0 = tat            [GIA-DINH]

# ---------------------------------------------------------------------------
# 6. Vat lieu in 3D
# ---------------------------------------------------------------------------
PLA_DENSITY = 1240.0            # kg/m^3, NatureWorks Ingeo 4043D TDS          [DATASHEET]
FILL_FACTOR = 1.0               # 1.0 = dac hoan toan (can tren); in thuc te nhe hon
# Neu da can chi tiet sau khi in, dien khoi luong (g) vao day -> ghi de FILL_FACTOR
MEASURED_MASS_G = {
    # "EBAmk2_002_mainarm": 30.0,
}

# ---------------------------------------------------------------------------
# 7. Mo phong
# ---------------------------------------------------------------------------
TIMESTEP = 0.0005               # s
CMD_PERIOD = 0.020              # s, chu ky xung servo / cap nhat lenh (de cuong)
