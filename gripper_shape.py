"""gripper_shape.py — SINH TU DONG boi tools/gripper_from_photos.py, KHONG sua tay.

Bien dang lay tu anh chup gripper that (he G va he ngon dinh nghia trong params.py, muc 2):
  FINGER: ngon TRAI, he ngon (goc = tam oc duoi Lw, x = gx, y = gy); mat kep huong +x.
          Dat theo 2 lo oc cua ngon (cach 20 mm, thuoc kep). Ngon phai = guong qua truc y.
  FINGER_INNER: lop long ngon, thut 1.5 mm so voi FINGER.
  BASE: vien de (mat duoi), he G, tu anh 'mo - chup tu duoi len'.
  BLOCK: khoi giu 2 chot khau tren, he G, tu anh 'mo - chup tu tren xuong'.
  SERVO_GX: tam SG90 theo gx (he G), tu anh 'mo - chup tu tren xuong'.
Don vi mm. [ANH]
"""

# ngon trai, he ngon
FINGER = [
    (0.41, -3.98),
    (3.23, -3.01),
    (5.66, -1.13),
    (5.36, 8.22),
    (12.26, 14.02),
    (12.16, 15.72),
    (13.11, 21.62),
    (11.21, 28.57),
    (11.11, 30.62),
    (11.21, 33.52),
    (12.66, 43.87),
    (12.21, 56.62),
    (11.76, 56.87),
    (11.43, 57.69),
    (7.93, 57.59),
    (7.31, 57.12),
    (2.01, 48.97),
    (0.81, 46.32),
    (1.06, 28.27),
    (0.41, 26.02),
    (-2.04, 20.77),
    (-4.14, 14.42),
    (-3.69, -0.78),
    (-1.92, -3.26),
]

# long ngon (thut 1.5 mm), he ngon
FINGER_INNER = [
    (0.46, -2.43),
    (2.53, -1.66),
    (4.16, -0.38),
    (3.81, 7.62),
    (4.06, 8.97),
    (10.66, 14.87),
    (11.61, 21.57),
    (9.76, 28.22),
    (9.61, 30.62),
    (9.71, 33.52),
    (11.16, 43.87),
    (10.76, 55.52),
    (10.48, 56.14),
    (8.41, 56.02),
    (3.41, 48.37),
    (2.26, 45.97),
    (2.51, 27.92),
    (1.76, 25.37),
    (-0.69, 20.12),
    (-2.64, 14.42),
    (-2.19, -0.38),
    (-1.02, -1.96),
]

# vien de, he G
BASE = [
    (-18.59, 26.65),
    (-17.74, -16.01),
    (-9.41, -24.38),
    (6.70, -24.94),
    (16.78, -16.62),
    (21.13, 25.27),
    (13.15, 28.41),
    (-4.08, 29.01),
]

# khoi giu chot khau tren, he G
BLOCK = [
    (-11.20, 21.89),
    (-8.13, 24.31),
    (8.68, 24.38),
    (12.14, 21.31),
    (12.14, 8.00),
    (-11.20, 8.00),
]

SERVO_GX = -11.49   # tam than SG90 theo gx, he G
