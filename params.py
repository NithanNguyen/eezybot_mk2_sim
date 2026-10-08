"""
params.py — Toan bo tham so cua mo hinh EEZYbotARM Mk2.

Quy uoc don vi trong file nay: mm, gram, do (de doc/sua tay).
build_model.py va mk2_kinematics.py tu doi sang SI (m, kg, rad).

Moi tham so co ghi NGUON:
  [DO-STL]  : do tu file STL goc (Thingiverse thing:1454048) bang tools/holes.py
  [DATASHEET]: lay tu datasheet (ghi ro trang)
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

# Chieu cao truc vai O so voi mat ban (mm).
# Tinh tu chuoi chi tiet: be vit mainbase z=33.5 + lower_base->mat tren gearmast
# 16.78 (gom khe bi 1.78 uoc tu ranh bi R~3.18 va bi D6) + 43.00 trong 001_base.
H_SHOULDER = 93.3                                     # [DO-STL + suy luan] -> nen do lai

# Truc vai cat truc dung (O nam ngay tren truc quay de): 001_base O.y = truc.y
SHOULDER_OFFSET_X = 0.0                                                       # [DO-STL]

# Ti so truyen banh rang de: gearmast(_full) 50 rang / gearservo 25 rang = 2.0
# (Neu ban in gearservo_22DENTI 22 rang thi doi thanh 50/22 = 2.2727)
BASE_GEAR_RATIO = 2.0                                                         # [DO-STL]

# ---------------------------------------------------------------------------
# 2. Dau cong tac (but ve / tam kep) so voi co tay W, he truc co dinh (x truoc, z len)
# ---------------------------------------------------------------------------
TOOL_OFFSET = (40.0, -45.0)     # (dx, dz) mm                                 [GIA-DINH] !!!
TOOL_MASS_G = 15.0              # gia but + but (+ kep)                        [GIA-DINH]

# ---------------------------------------------------------------------------
# 3. Gioi han khop (do). theta2 = goc canh tay chinh so voi phuong ngang,
#    phi = goc tay ngang (forearm) so voi phuong ngang, rel = phi - theta2.
#    Mac dinh lay be rong 90 do theo URDF cua repo HotBlackRobotics/ntbd
#    (joint_2 [-35,55], joint_3 [-70,20]); vi tri cu the phai do lai.
# ---------------------------------------------------------------------------
THETA1_LIM = (-45.0, 45.0)      # khop de (servo 0..180 qua hop so 2:1)       [ntbd URDF]
THETA2_LIM = (35.0, 125.0)                                                    # [GIA-DINH]
REL_LIM = (-160.0, -70.0)       # phi - theta2                                [GIA-DINH]

# ---------------------------------------------------------------------------
# 4. Lop anh xa goc dong hoc -> goc servo (do). Phai hieu chuan tren phan cung.
#    servo = OFFSET + SIGN * (goc - goc_home)
#    home: theta1 = 0, theta2 = 90 (tay chinh dung), phi = 0 (tay ngang nam ngang)
# ---------------------------------------------------------------------------
SERVO_OFFSET = (90.0, 90.0, 90.0)                                             # [GIA-DINH]
SERVO_SIGN = (+1, +1, +1)                                                     # [GIA-DINH]
SERVO_RANGE = (0.0, 180.0)

# ---------------------------------------------------------------------------
# 5. Servo MG996R — TowerPro (https://towerpro.com.tw/product/mg996r/)
# ---------------------------------------------------------------------------
SUPPLY = "4.8V"                 # "4.8V" hoac "6V"
MG996R = {
    "4.8V": {"stall_kgfcm": 9.4, "s_per_60deg": 0.19},                        # [DATASHEET]
    "6V":   {"stall_kgfcm": 11.0, "s_per_60deg": 0.15},                       # [DATASHEET]
}
SERVO_MASS_G = 55.0                                                           # [DATASHEET]
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
