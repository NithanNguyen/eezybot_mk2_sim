"""Find cylindrical holes (circular loops in cross-sections) in an STL.
For each principal axis, slice at many levels, fit circles to closed loops,
keep near-perfect circles with radius in [rmin, rmax], and cluster them.
Output: axis, center (in part's own frame, mm), radius, z-extent along axis."""
import sys, numpy as np, trimesh

def fit_circle(P):
    A = np.c_[2 * P, np.ones(len(P))]
    b = (P ** 2).sum(1)
    c, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = c[0], c[1]
    r = np.sqrt(c[2] + cx * cx + cy * cy)
    res = np.abs(np.hypot(P[:, 0] - cx, P[:, 1] - cy) - r).max()
    return cx, cy, r, res

def holes(path, rmin=1.3, rmax=9.0, nlev=60):
    m = trimesh.load(path, force="mesh")
    out = []
    for ax in range(3):
        lo, hi = m.bounds[0][ax], m.bounds[1][ax]
        n = np.zeros(3); n[ax] = 1
        others = [i for i in range(3) if i != ax]
        for lev in np.linspace(lo + 0.2, hi - 0.2, nlev):
            o = np.zeros(3); o[ax] = lev
            s = m.section(plane_origin=o, plane_normal=n)
            if s is None: continue
            for d in s.discrete:
                if len(d) < 12: continue
                if np.linalg.norm(d[0] - d[-1]) > 1e-3: continue
                P = d[:, others]
                cx, cy, r, res = fit_circle(P)
                if rmin <= r <= rmax and res < 0.08 * r + 0.05:
                    out.append((ax, cx, cy, r, lev))
    # cluster
    cl = []
    for ax, cx, cy, r, lev in out:
        for c in cl:
            if c["ax"] == ax and abs(c["cx"] - cx) < 0.4 and abs(c["cy"] - cy) < 0.4 and abs(c["r"] - r) < 0.4:
                c["levs"].append(lev); break
        else:
            cl.append(dict(ax=ax, cx=cx, cy=cy, r=r, levs=[lev]))
    res = []
    for c in cl:
        if len(c["levs"]) < 2: continue
        res.append((c["ax"], c["cx"], c["cy"], c["r"], min(c["levs"]), max(c["levs"])))
    return m, res

if __name__ == "__main__":
    names = "xyz"
    for p in sys.argv[1:]:
        m, hs = holes(p)
        print(f"== {p.split('/')[-1]}  bounds {m.bounds.round(1).tolist()}")
        for ax, cx, cy, r, a, b in sorted(hs, key=lambda h: (h[0], h[1], h[2])):
            oth = [names[i] for i in range(3) if i != ax]
            print(f"  axis {names[ax]}  {oth[0]}={cx:7.2f} {oth[1]}={cy:7.2f}  r={r:5.2f}  {names[ax]}:[{a:6.1f},{b:6.1f}]")
