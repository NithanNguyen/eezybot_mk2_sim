"""color_sim.py — Bo phan loai mau nap chai (ban sao cua firmware) + mo phong doc RGBC cua GY-33.

Nguon du lieu (chep nguyen tu repo firmware NithanNguyen/eezybot_mk2_robot, commit a172b69):
  data/color/model.json : tam 3 lop (r, g), d_max, C_empty, C_margin, muc LED, navg, settle_ms
  data/color/test.csv   : tap KIEM TRA doc lap do tren tram that (20 mau moi nhan green/red/white/empty/other)

Thuat toan = colormath::classify() (lib/ColorClassifier/ColorMath.h), tinh bang float32 giong ESP32
(giong classify_one() trong tools/colorcal/colorcal.py):
  1. |C - C_empty| < C_margin       -> NO_CAP   (tram trong, tiep tuc cho)
  2. bao hoa hoac R+G+B < 1         -> UNKNOWN
  3. r = R/(R+G+B), g = G/(R+G+B)
  4. tam lop gan nhat (Euclid) trong mat phang (r, g)
  5. khoang cach > d_max            -> UNKNOWN  (khu vuc vut bo, bao loi qua Serial)

MuJoCo khong mo phong quang hoc cua cam bien. Moi phep do trong mo phong PHAT LAI 1 dong do that cua
test.csv co nhan dung voi vat dang nam o tram (nap xanh/do/trang, vat la "other", hoac tram trong "empty").
Moi dong test.csv da la trung binh navg = 5 lan doc (dung cau hinh trong model.json).
"""
import csv, json, math, os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_JSON = os.path.join(HERE, "data", "color", "model.json")
TEST_CSV = os.path.join(HERE, "data", "color", "test.csv")
F32 = np.float32
CLASSES = ("green", "red", "white")                         # thu tu tam lop trong firmware
FW_NAME = {"green": "GREEN", "red": "RED", "white": "WHITE", "unknown": "UNKNOWN", "no_cap": "NO_CAP"}
EXPECTED = {"green": "green", "red": "red", "white": "white", "empty": "no_cap", "other": "unknown"}
RAW_MAX = 65535


def load_model(path=MODEL_JSON):
    with open(path, encoding="utf-8") as f:
        m = json.load(f)
    if tuple(m["classes"]) != CLASSES:
        raise ValueError(f"{path}: thu tu lop {m['classes']} khac {CLASSES}")
    return m


def read_rows(path=TEST_CSV):
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        for i, r in enumerate(csv.DictReader(f)):
            for k in ("R", "G", "B", "C"):
                r[k] = float(r[k])
            r["sat"] = int(r["sat"]); r["row"] = i + 2          # so dong trong file (dong 1 = tieu de)
            rows.append(r)
    return rows


def classify(model, R, G, B, C, sat=0):
    """Ban Python cua colormath::classify() (float32). Tra ve dict label/nearest/dist/r/g."""
    R, G, B, C = F32(R), F32(G), F32(B), F32(C)
    s = R + G + B
    rn = R / s if s > 0 else F32(0)
    gn = G / s if s > 0 else F32(0)
    best, dbest = 0, F32(1e30)
    for k, (cx, cy) in enumerate(model["centroids"]):
        dr, dg = rn - F32(cx), gn - F32(cy)
        d = F32(math.sqrt(F32(dr * dr + dg * dg)))
        if d < dbest:
            best, dbest = k, d
    if abs(C - F32(model["c_empty"])) < F32(model["c_margin"]):
        label = "no_cap"
    elif sat or s < 1:
        label = "unknown"
    elif dbest > F32(model["dmax"]):
        label = "unknown"
    else:
        label = CLASSES[best]
    return {"label": label, "nearest": CLASSES[best], "dist": float(dbest), "r": float(rn), "g": float(gn)}


def cls_line(res, row):
    """Dong Serial giong firmware (color_console.cpp: printResult)."""
    return (f"CLS,{FW_NAME[res['label']]},{FW_NAME[res['nearest']]},{res['dist']:.6f},{res['r']:.6f},{res['g']:.6f},"
            f"{row['R']:.3f},{row['G']:.3f},{row['B']:.3f},{row['C']:.3f},{row['sat']}")


def err_line(res, model, row):
    """Dong bao loi khi khong xac dinh mau (color_console.cpp: classifyAndPrint)."""
    return (f"# ERR UNKNOWN_COLOR nearest={FW_NAME[res['nearest']]} dist={res['dist']:.4f} "
            f"dmax={model['dmax']:.4f} sat={row['sat']}")


class SensorSim:
    """Cam bien GY-33 mo phong: moi lan do tra ve 1 dong do that cua test.csv dung nhan vat o tram.
    Nap/vat la: rut KHONG hoan lai (moi dong dung toi da 1 lan cho toi khi het, roi xao lai);
    tram trong: rut co hoan lai."""

    def __init__(self, seed=0, path=TEST_CSV):
        self.rng = np.random.default_rng(seed)
        self.rows = read_rows(path)
        self.by = {}
        for r in self.rows:
            self.by.setdefault(r["label"], []).append(r)
        self.pool = {k: [] for k in self.by}

    def measure(self, label, consume=True):
        """label: 'green'|'red'|'white'|'other'|'empty' (vat thuc dang nam o tram).
        consume=False: rut co hoan lai (dung cho lan doc kenh C de phat hien co nap)."""
        if label == "empty" or not consume:
            return self.by[label][int(self.rng.integers(len(self.by[label])))]
        if not self.pool[label]:
            self.pool[label] = list(self.rng.permutation(len(self.by[label])))
        return self.by[label][self.pool[label].pop()]


def parity_check(model=None, path=TEST_CSV):
    """Phan loai lai toan bo test.csv; tra ve ma tran nham lan (hang: nhan that; cot: du doan) de so voi
    reports/color/eval_test.json cua repo firmware (99/100 dung, 1 nap do bi 'khong xac dinh')."""
    model = model or load_model()
    rows = read_rows(path)
    labs = ("green", "red", "white", "empty", "other")
    preds = ("green", "red", "white", "unknown", "no_cap")
    M = np.zeros((len(labs), len(preds)), int)
    for r in rows:
        p = classify(model, r["R"], r["G"], r["B"], r["C"], r["sat"])["label"]
        M[labs.index(r["label"]), preds.index(p)] += 1
    ok = sum(M[i, preds.index(EXPECTED[l])] for i, l in enumerate(labs))
    return labs, preds, M, ok, len(rows)


if __name__ == "__main__":
    labs, preds, M, ok, n = parity_check()
    print("Ma tran nham lan tren data/color/test.csv (hang: that, cot: du doan " + ", ".join(preds) + ")")
    for l, row in zip(labs, M):
        print(f"  {l:6s} {row}")
    print(f"Dung {ok}/{n} = {100.0 * ok / n:.1f}%")
