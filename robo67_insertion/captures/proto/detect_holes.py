#!/usr/bin/env python3
"""Prototype 2c: locate the I/O box by the unique 'dark holes on bright metal'
signature of its port face (port openings + fan-grille dots + screw holes).

Pipeline:
  1. BLACK-HAT (close(gray) - gray): fires on small dark features on a brighter
     surround (the ports/grille); ~0 on carpet / smooth paddle / white cube.
  2. threshold -> hole-pixel mask -> hole blob centroids.
  3. blur the mask into a DENSITY map; the box face is the densest hole cluster
     -> region-grow (density > frac*peak) the connected blob holding the peak.
  4. collect the hole blobs inside that region -> minAreaRect = oriented box
     (center, size, angle) from object-intrinsic features only (immune to the
     adjacent paddle/scanner/cube)."""
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CAP = os.path.dirname(HERE)


def detect(bgr, *, bh_ks=13, bh_thr=35, density_sigma=22, region_frac=0.4,
           min_blob=8, max_blob=2500, min_cluster_pts=12, inflate=1.18):
    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (bh_ks, bh_ks))
    bh = cv2.morphologyEx(g, cv2.MORPH_BLACKHAT, k)
    mask = (bh > bh_thr).astype(np.uint8)

    density = cv2.GaussianBlur(mask.astype(np.float32), (0, 0), density_sigma)
    _, peakval, _, peak = cv2.minMaxLoc(density)
    peak = np.array(peak, float)

    # region-grow: connected high-density blob that holds the global peak
    region = (density > region_frac * peakval).astype(np.uint8)
    n, lbl = cv2.connectedComponents(region, 8)
    peak_lbl = lbl[int(peak[1]), int(peak[0])]

    cnts, _ = cv2.findContours(mask * 255, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    pts = []
    for c in cnts:
        a = cv2.contourArea(c)
        if a < min_blob or a > max_blob:
            continue
        m = cv2.moments(c)
        if m["m00"] == 0:
            continue
        cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
        pts.append((cx, cy))
    pts = np.array(pts, float)
    dbg = dict(bh=bh, density=density, pts=pts, peak=peak, region=region)
    if len(pts) == 0:
        return None, dbg

    inside = np.array([lbl[int(np.clip(y, 0, g.shape[0] - 1)),
                           int(np.clip(x, 0, g.shape[1] - 1))] == peak_lbl
                       for x, y in pts])
    cluster = pts[inside]
    if len(cluster) < min_cluster_pts:
        return None, dbg

    rect = cv2.minAreaRect(cluster.astype(np.float32))
    (cx, cy), (rw, rh), ang = rect
    rect = ((cx, cy), (rw * inflate, rh * inflate), ang)
    box = cv2.boxPoints(rect)
    return dict(center=np.array([cx, cy]), rect=rect, box=box, n=len(cluster),
                cluster=cluster, size=(rw, rh), angle=float(ang),
                peakval=float(peakval)), dbg


def main():
    names = sys.argv[1:] or [
        "overhead_template_src.jpg", "overhead_cable_box.jpg",
        "overhead_now.jpg", "overhead_live_cable.jpg",
    ]
    for n in names:
        path = n if os.path.isabs(n) else os.path.join(CAP, n)
        img = cv2.imread(path)
        if img is None:
            print(f"!! cannot read {path}")
            continue
        res, dbg = detect(img)
        ov = img.copy()
        for (x, y) in dbg["pts"]:
            cv2.circle(ov, (int(x), int(y)), 2, (0, 140, 255), -1)
        pk = dbg["peak"]
        cv2.drawMarker(ov, (int(pk[0]), int(pk[1])), (255, 255, 0), cv2.MARKER_CROSS, 22, 2)
        stem = os.path.splitext(os.path.basename(n))[0]
        if res is not None:
            c = res["center"]
            for p in res["cluster"]:
                cv2.circle(ov, (int(p[0]), int(p[1])), 3, (0, 255, 0), -1)
            cv2.drawContours(ov, [res["box"].astype(int)], -1, (255, 0, 0), 2)
            cv2.circle(ov, (int(c[0]), int(c[1])), 6, (0, 0, 255), -1)
            cv2.putText(ov, f"n={res['n']} ang={res['angle']:.0f} "
                        f"{int(res['size'][0])}x{int(res['size'][1])}",
                        (int(c[0]) - 70, int(c[1]) - 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
            print(f"{stem:28s} -> center=({c[0]:.0f},{c[1]:.0f}) ang={res['angle']:.1f} "
                  f"size={int(res['size'][0])}x{int(res['size'][1])} n={res['n']} "
                  f"peakval={res['peakval']:.3f}")
        else:
            print(f"{stem:28s} -> NO cluster ({len(dbg['pts'])} raw holes)")
        cv2.imwrite(os.path.join(HERE, f"holes_{stem}.png"), ov)


if __name__ == "__main__":
    main()
