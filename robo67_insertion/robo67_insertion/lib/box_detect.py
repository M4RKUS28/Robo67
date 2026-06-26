"""Industrial I/O-box detection for the cable-insertion overhead vision.

An overhead Logitech C920 looks straight down at the workspace. The cable task
targets a **dark, roughly rectangular industrial I/O box** whose top face is
covered in high-contrast features -- metallic ports (LAN/USB/CAN BUS), white
text labels, audio jacks, and a fan grille. The box BODY is near-black, but the
gray grooved-carpet background is a *similar* mid-gray intensity, so a plain
brightness threshold cannot separate them (verified on a live C920 frame
2026-06-26: carpet median ~116, box body similar).

The robust discriminator is **local texture energy**: the port face is a dense
island of high local variance (black body + bright ports + white text = large
local swings), whereas the carpet is a uniform low-variance field. We compute a
local standard-deviation map, threshold it, merge the port features into one
blob, and return the qualifying blob(s) as :class:`Box` (centroid + oriented
size + corner quad), sorted by score.

This module is pure-Python (numpy + OpenCV + stdlib only). Like
:mod:`hole_detect` it deliberately avoids importing rclpy/ROS/scipy so it can be
unit-tested on any host. The detected centroid maps to robot-base XY through the
same overhead homography the peg-in-hole socket uses
(:class:`~robo67_insertion.lib.pixel_mapping.HomographyMappingAdapter`).

Pipeline
--------
1. Convert to grayscale (float).
2. Compute a local standard-deviation map over a ``texture_win`` window
   (``std = sqrt(E[x^2] - E[x]^2)`` via box filters -- fast, separable).
3. Threshold ``std > min_texture_std`` into a binary texture mask.
4. Morphologically CLOSE (merge ports/labels into one box-face blob) then OPEN
   (drop speckle).
5. Find external contours; reject by area, ROI border, oriented aspect ratio,
   and fill (extent).
6. Return survivors as :class:`Box`, sorted by score (texture density * size)
   descending.
"""
from dataclasses import dataclass
import math
from typing import Optional

import cv2
import numpy as np

__all__ = [
    "Box",
    "BoxParams",
    "local_texture_std",
    "detect_gray_box",
    "BoxOrbParams",
    "OrbBoxMatcher",
    "detect_box_orb",
]


@dataclass
class Box:
    """A detected box, in image (pixel) coordinates.

    Attributes
    ----------
    u, v:
        Centroid of the box blob, in pixels.
    width_px, height_px:
        Side lengths of the oriented (min-area) bounding rectangle, in pixels
        (``width_px >= height_px`` is NOT guaranteed; they follow OpenCV's
        ``minAreaRect`` order).
    angle_deg:
        Orientation of the min-area rectangle, in degrees (OpenCV convention).
    score:
        Detection score; higher is a stronger box candidate (texture density
        times square-root of area, normalized). Used to sort candidates.
    corners:
        ``(4, 2)`` float array of the oriented rectangle's corner pixels
        (``cv2.boxPoints`` order), handy for drawing the quad overlay. For the
        ORB matcher this is the **tight** box (the min-area rect of the matched
        inlier keypoints, see :class:`OrbBoxMatcher`) -- it hugs the actual box
        face rather than the padded template outline.
    template_corners:
        ``(4, 2)`` float array of the projected reference-template outline,
        ``[TL, TR, BR, BL]`` -- **identity-preserving** across box rotations
        (the same template corner always maps to the same physical box corner).
        Set ONLY by the ORB matcher; ``None`` for the texture detector. The
        box-frame port offset relies on this ordered quad; ``corners`` (which
        comes from ``minAreaRect`` and is NOT identity-preserving) must not be
        used for that.
    """

    u: float
    v: float
    width_px: float
    height_px: float
    angle_deg: float
    score: float
    corners: np.ndarray
    template_corners: Optional[np.ndarray] = None


@dataclass
class BoxParams:
    """Tunable parameters for :func:`detect_gray_box`.

    Attributes
    ----------
    texture_win:
        Odd window size (px) for the local standard-deviation map. Larger =
        smoother/blobbier texture estimate.
    min_texture_std:
        Pixels whose local std exceeds this are 'busy' (box-face) candidates.
        Carpet sits well below this; the port face well above (live C920:
        carpet std median ~15, box face ~50-90). ~p90 of a typical frame.
    close_ks, open_ks:
        CLOSE kernel (merge ports/labels into one blob) and OPEN kernel (drop
        speckle), in pixels.
    min_area_px, max_area_px:
        Accepted blob contour area (px^2) -- rejects tiny texture flecks and
        full-frame floods.
    min_aspect, max_aspect:
        Oriented aspect ratio ``max(w,h)/min(w,h)`` bounds; the I/O box is a
        chunky rectangle, not a thin sliver.
    min_extent:
        Minimum fill ``contour_area / min_area_rect_area``; a solid blob fills
        most of its oriented box (rejects ragged carpet/cable wisps).
    roi_margin:
        Ignore blobs whose centroid is within this fractional image border
        (kills the keyboard/foot/bag at the frame edges).
    """

    texture_win: int = 21
    min_texture_std: float = 35.0
    close_ks: int = 35
    open_ks: int = 9
    min_area_px: float = 6000.0
    max_area_px: float = 300000.0
    min_aspect: float = 1.0
    max_aspect: float = 3.2
    min_extent: float = 0.55
    roi_margin: float = 0.04


def local_texture_std(gray: np.ndarray, win: int) -> np.ndarray:
    """Return the per-pixel local standard deviation over a ``win`` x ``win`` box.

    Uses the identity ``Var = E[x^2] - E[x]^2`` with normalized box filters, so
    it is O(1) per pixel regardless of window size. Input may be any dtype; the
    result is float32.
    """
    g = gray.astype(np.float32)
    win = int(win) | 1  # force odd
    mean = cv2.boxFilter(g, -1, (win, win), normalize=True)
    mean_sq = cv2.boxFilter(g * g, -1, (win, win), normalize=True)
    var = np.clip(mean_sq - mean * mean, 0.0, None)
    return np.sqrt(var)


def detect_gray_box(bgr: np.ndarray, params: BoxParams = BoxParams()) -> list[Box]:
    """Detect dark, port-covered I/O box(es) in an overhead BGR image.

    Returns a list of :class:`Box` sorted by ``score`` (texture density * size)
    descending; the best box is first. Empty if none qualifies. See the module
    docstring for the texture-energy rationale and :class:`BoxParams` for tuning.
    """
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    std = local_texture_std(gray, params.texture_win)

    mask = (std > params.min_texture_std).astype(np.uint8) * 255
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (params.close_ks, params.close_ks)))
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (params.open_ks, params.open_ks)))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    mx0, my0 = int(params.roi_margin * w), int(params.roi_margin * h)

    candidates: list[tuple[Box, float]] = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < params.min_area_px or area > params.max_area_px:
            continue

        rect = cv2.minAreaRect(c)  # ((cx, cy), (rw, rh), angle)
        (rcx, rcy), (rw, rh), angle = rect
        if rw <= 1.0 or rh <= 1.0:
            continue
        rect_area = rw * rh
        aspect = max(rw, rh) / max(1.0, min(rw, rh))
        extent = area / max(1.0, rect_area)
        if not (params.min_aspect <= aspect <= params.max_aspect):
            continue
        if extent < params.min_extent:
            continue

        m = cv2.moments(c)
        if m["m00"] == 0:
            continue
        cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
        if not (mx0 < cx < w - mx0 and my0 < cy < h - my0):
            continue

        # texture density inside the blob (mean local std over its pixels)
        blob = np.zeros(gray.shape, np.uint8)
        cv2.drawContours(blob, [c], -1, 255, -1)
        density = float(std[blob > 0].mean()) if (blob > 0).any() else 0.0
        score = density * math.sqrt(area)

        corners = cv2.boxPoints(rect).astype(float)
        box = Box(u=float(cx), v=float(cy), width_px=float(rw), height_px=float(rh),
                  angle_deg=float(angle), score=float(score), corners=corners)
        candidates.append((box, score))

    candidates.sort(key=lambda bs: bs[1], reverse=True)
    return [b for b, _ in candidates]


# ---------------------------------------------------------------------------
# ORB template matching -- robust, object-SPECIFIC detection.
#
# The texture detector above keys on "busiest blob", which a cluttered scene
# (white retail boxes, a teal package, a knob box) can hijack. To localize ONE
# specific richly-textured object (this industrial I/O box) regardless of
# position/rotation and reject every distractor, match ORB features against a
# stored reference TEMPLATE crop of the box and fit a RANSAC homography. The
# projected template outline + centroid give the box pose; the RANSAC inlier
# count is the confidence (and the absent-box reject test).
# ---------------------------------------------------------------------------


@dataclass
class BoxOrbParams:
    """Tunable parameters for :class:`OrbBoxMatcher` / :func:`detect_box_orb`.

    Attributes
    ----------
    n_features:
        ORB feature budget per image.
    ratio:
        Lowe ratio-test threshold (keep a match if its best distance is below
        ``ratio`` x the second-best).
    ransac_reproj_px:
        RANSAC reprojection tolerance (px) for the homography fit.
    min_inliers:
        Minimum RANSAC inliers to accept a detection. Below this the box is
        considered absent (rejects empty/cluttered scenes that lack the box).
    box_inflate:
        Scale applied to the tight inlier extent (the matched keypoints sit
        slightly inside the physical box face, so a small >1 inflation makes the
        quad hug the box edges). ``1.0`` = exactly the keypoint extent.
    min_box_side_px:
        If the inlier extent collapses below this on either side (degenerate /
        near-collinear inliers), fall back to the projected template outline
        instead of emitting a sliver.
    """

    n_features: int = 2000
    ratio: float = 0.75
    ransac_reproj_px: float = 5.0
    min_inliers: int = 20
    box_inflate: float = 1.15
    min_box_side_px: float = 10.0


def _box_from_quad(quad: np.ndarray, score: float,
                   template_corners: Optional[np.ndarray] = None) -> Box:
    """Build a :class:`Box` from an oriented (4, 2) corner quad.

    ``quad`` is the box used for size/centre/overlay (for the ORB matcher this
    is the TIGHT inlier rect). ``template_corners`` is the optional
    identity-preserving template outline (the ORB matcher passes its projected
    quad here; the texture detector leaves it ``None``).
    """
    rect = cv2.minAreaRect(quad.astype(np.float32))  # ((cx,cy),(w,h),angle)
    (cx, cy), (rw, rh), angle = rect
    tc = None if template_corners is None else np.asarray(template_corners, float)
    return Box(u=float(cx), v=float(cy), width_px=float(rw), height_px=float(rh),
               angle_deg=float(angle), score=float(score), corners=quad.astype(float),
               template_corners=tc)


def _tight_quad_from_inliers(inlier_pts: np.ndarray, template_quad: np.ndarray,
                             inflate: float, min_side_px: float) -> np.ndarray:
    """Tight oriented quad from the matched inlier scene keypoints.

    The RANSAC inliers are real feature points ON the box face, so their extent
    hugs the actual object far more tightly than the projected (padded) template
    outline. The box ORIENTATION is taken from the projected template top edge
    (``template_quad`` TL->TR), which is stable because it comes from the RANSAC
    homography -- using ``cv2.minAreaRect`` on the keypoints instead lets the box
    axis flip/skew with the (uneven) keypoint spread. The EXTENT along those two
    axes comes from the inliers; ``inflate`` (>1) grows it a touch so it reaches
    the physical edges (keypoints sit just inside). Falls back to
    ``template_quad`` if the inlier set is too small or the box is degenerate.
    """
    pts = np.asarray(inlier_pts, np.float32).reshape(-1, 2)
    tquad = np.asarray(template_quad, float)
    if len(pts) < 3:
        return tquad
    ex = tquad[1] - tquad[0]  # box "x" (top edge) direction in the scene
    nx = float(np.linalg.norm(ex))
    if nx < 1e-6:
        return tquad
    ex = ex / nx
    ey = np.array([-ex[1], ex[0]])  # perpendicular -> a proper rectangle
    c = pts.mean(axis=0)
    s = (pts - c) @ ex
    t = (pts - c) @ ey
    sc, tc = (s.min() + s.max()) / 2.0, (t.min() + t.max()) / 2.0
    sh = (s.max() - s.min()) / 2.0 * float(inflate)
    th = (t.max() - t.min()) / 2.0 * float(inflate)
    if 2.0 * min(sh, th) < float(min_side_px):
        return tquad
    return np.array([
        c + (sc - sh) * ex + (tc - th) * ey,
        c + (sc + sh) * ex + (tc - th) * ey,
        c + (sc + sh) * ex + (tc + th) * ey,
        c + (sc - sh) * ex + (tc + th) * ey,
    ], float)


class OrbBoxMatcher:
    """Match a stored reference template of the box into a scene via ORB+RANSAC.

    The template's ORB features are computed ONCE at construction (so the node
    can call :meth:`detect` per frame cheaply). :meth:`detect` returns a list
    with a single :class:`Box` (the located box; ``score`` = inlier count,
    ``corners`` = the TIGHT box hugging the matched inlier keypoints, and
    ``template_corners`` = the projected template outline) or an empty list when
    the box is absent / too few inliers.

    Why two quads: the projected template outline is identity-preserving (the
    box-frame port offset needs that) but loose -- it carries the template's
    padding around the box. The displayed/targeted box (``corners``) is instead
    the min-area rect of the RANSAC inlier keypoints, which sit ON the box face,
    so it hugs the real object. See :func:`_tight_quad_from_inliers`.
    """

    def __init__(self, template_bgr: np.ndarray, params: BoxOrbParams = BoxOrbParams()):
        self.params = params
        self._orb = cv2.ORB_create(nfeatures=params.n_features)
        self._bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        tgray = cv2.cvtColor(template_bgr, cv2.COLOR_BGR2GRAY)
        self._th, self._tw = tgray.shape[:2]
        self._kp_t, self._des_t = self._orb.detectAndCompute(tgray, None)
        if self._des_t is None or len(self._kp_t) < params.min_inliers:
            raise ValueError(
                f"template has too few ORB features ({0 if self._des_t is None else len(self._kp_t)}); "
                "use a larger/sharper reference crop")

    def detect(self, bgr: np.ndarray) -> list[Box]:
        p = self.params
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        kp_s, des_s = self._orb.detectAndCompute(gray, None)
        if des_s is None or len(des_s) < 2:
            return []
        good = []
        for pair in self._bf.knnMatch(self._des_t, des_s, k=2):
            if len(pair) < 2:
                continue
            m, n = pair
            if m.distance < p.ratio * n.distance:
                good.append(m)
        if len(good) < p.min_inliers:
            return []
        src = np.float32([self._kp_t[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst = np.float32([kp_s[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
        H, mask = cv2.findHomography(src, dst, cv2.RANSAC, p.ransac_reproj_px)
        if H is None or mask is None:
            return []
        inliers = int(mask.sum())
        if inliers < p.min_inliers:
            return []
        # identity-preserving projected template outline ([TL, TR, BR, BL])
        corners = np.float32([[0, 0], [self._tw, 0], [self._tw, self._th],
                              [0, self._th]]).reshape(-1, 1, 2)
        template_quad = cv2.perspectiveTransform(corners, H).reshape(-1, 2)
        # TIGHT box: extent of the inlier scene keypoints (on the box face),
        # oriented by the (stable) projected template edge.
        inlier_pts = dst.reshape(-1, 2)[mask.ravel() == 1]
        tight_quad = _tight_quad_from_inliers(
            inlier_pts, template_quad, p.box_inflate, p.min_box_side_px)
        return [_box_from_quad(tight_quad, float(inliers), template_corners=template_quad)]


def detect_box_orb(bgr: np.ndarray, template_bgr: np.ndarray,
                   params: BoxOrbParams = BoxOrbParams()) -> list[Box]:
    """One-shot convenience: match ``template_bgr`` into ``bgr`` (see
    :class:`OrbBoxMatcher`). For repeated calls build an :class:`OrbBoxMatcher`
    once (it caches the template features) instead of calling this per frame."""
    return OrbBoxMatcher(template_bgr, params).detect(bgr)
