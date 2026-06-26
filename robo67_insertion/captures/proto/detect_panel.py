#!/usr/bin/env python3
"""Prototype: detect the I/O box by its BRIGHT, low-saturation metallic port
panel (vs. dark carpet) -- not ORB. Verify the candidate is the real box by its
internal dark-hole structure (port openings + fan grille). Saves overlays +
intermediate masks so we can eyeball reliability across frames."""
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CAP = os.path.dirname(HERE)


def detect(bgr, *, bright=150, sat_max=45, close_ks=25, open_ks=5,
           min_area=4000, max_area=120000, min_aspect=1.15, max_aspect=2.2,
           min_extent=0.55, min_dark_frac=0.06, dark_thr=100):
    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    sat = hsv[:, :, 1]

    bright_mask = ((g > bright) & (sat < sat_max)).astype(np.uint8) * 255
    closed = cv2.morphologyEx(
        bright_mask, cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_ks, close_ks)))
    opened = cv2.morphologyEx(
        closed, cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_ks, open_ks)))

    contours, _ = cv2.findContours(opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cands = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area or area > max_area:
            continue
        rect = cv2.minAreaRect(c)
        (cx, cy), (rw, rh), ang = rect
        if rw < 5 or rh < 5:
            continue
        aspect = max(rw, rh) / min(rw, rh)
        extent = area / (rw * rh)
        if not (min_aspect <= aspect <= max_aspect):
            continue
        if extent < min_extent:
            continue
        # internal dark-hole structure: real panel has dark ports + grille holes
        blob = np.zeros(g.shape, np.uint8)
        cv2.drawContours(blob, [c], -1, 255, -1)
        inside = g[blob > 0]
        dark_frac = float((inside < dark_thr).mean())
        if dark_frac < min_dark_frac:
            continue
        score = area * dark_frac
        cands.append(dict(rect=rect, area=area, aspect=aspect, extent=extent,
                          dark_frac=dark_frac, score=score,
                          box=cv2.boxPoints(rect), c=c, cx=cx, cy=cy))
    cands.sort(key=lambda d: d["score"], reverse=True)
    return cands, dict(bright=bright_mask, closed=closed, opened=opened)


def main():
    names = sys.argv[1:] or [
        "overhead_template_src.jpg",
        "overhead_cable_box.jpg",
        "overhead_now.jpg",
        "overhead_live_cable.jpg",
    ]
    for n in names:
        path = n if os.path.isabs(n) else os.path.join(CAP, n)
        img = cv2.imread(path)
        if img is None:
            print(f"!! cannot read {path}")
            continue
        cands, masks = detect(img)
        ov = img.copy()
        for i, d in enumerate(cands[:4]):
            col = (0, 255, 0) if i == 0 else (0, 180, 255)
            cv2.drawContours(ov, [d["box"].astype(int)], -1, col, 2)
            cv2.circle(ov, (int(d["cx"]), int(d["cy"])), 4, col, -1)
            cv2.putText(ov, f"{i}:a{int(d['area'])} ar{d['aspect']:.2f} "
                        f"dk{d['dark_frac']:.2f}",
                        (int(d["cx"]) - 60, int(d["cy"]) - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
        stem = os.path.splitext(os.path.basename(n))[0]
        cv2.imwrite(os.path.join(HERE, f"panel_{stem}.png"), ov)
        cv2.imwrite(os.path.join(HERE, f"panel_{stem}_mask.png"), masks["opened"])
        best = cands[0] if cands else None
        print(f"{stem:28s} -> {len(cands)} cand"
              + (f"  best area={int(best['area'])} ar={best['aspect']:.2f} "
                 f"ext={best['extent']:.2f} dark={best['dark_frac']:.2f} "
                 f"@({int(best['cx'])},{int(best['cy'])})" if best else "  (none)"))


if __name__ == "__main__":
    main()
